#!/usr/bin/env python3
"""Import policy: aggregating surfaces are for users, internals stay explicit.

Rules, applied to `src/` non-test code:

1. `crate::prelude` is the public onboarding surface. Nothing *inside* the
   library may import from it. Internal code names the module it actually uses,
   so trimming the prelude can never silently reshape the internals.
2. A "facade" is a module that re-exports items from its own submodules (e.g.
   `pipeline::output` re-exporting `display_backend::DisplayBackend`). Facades
   are read-only public API; internal code imports the item from the submodule
   that defines it. Facade item lists are derived from the `pub use` statements
   themselves, so this check cannot drift from the code.
3. No glob imports (`use foo::*;`) outside test-gated modules.

Both `use` statements and fully-qualified inline paths are checked.

Exempt, because they are consumers of the public API rather than internals:

* `#[cfg(test)]` / `#[cfg(all(test, ..))]` modules inside `src/`
* `tests/`, `examples/`, `benches/`
* the `prelude` block itself, which is the designated aggregator

Exit status is non-zero when a violation is found.
"""

from __future__ import annotations

import pathlib
import re
import sys

SRC = pathlib.Path("src")


# ── source scanning helpers ──────────────────────────────────────────────────


def strip_comments_and_strings(line: str) -> str:
    """Best-effort removal of comments and string/char literals."""
    line = re.sub(r"//.*$", "", line)
    line = re.sub(r'"(?:\\.|[^"\\])*"', '""', line)
    line = re.sub(r"'(?:\\.|[^'\\])'", "''", line)
    return line


def module_path_of(path: pathlib.Path) -> str:
    """`src/pipeline/output/mod.rs` -> `crate::pipeline::output`."""
    parts = list(path.relative_to(SRC).parts)
    if parts[-1] == "lib.rs":
        return "crate"
    if parts[-1] == "mod.rs":
        parts = parts[:-1]
    else:
        parts[-1] = parts[-1][:-3]
    return "::".join(["crate", *parts])


def balanced_attribute_end(lines: list[str], start: int) -> int:
    """Last line index of the `#[...]` attribute beginning at `start`."""
    text, k = "", start
    while k < len(lines):
        text += strip_comments_and_strings(lines[k])
        if text.count("[") == text.count("]"):
            break
        k += 1
    return k


def attribute_mentions_test(lines: list[str], start: int) -> bool:
    """True for `#[cfg(test)]`, `#[cfg(all(test, feature = "x"))]`, …"""
    end = balanced_attribute_end(lines, start)
    text = "".join(strip_comments_and_strings(l) for l in lines[start : end + 1])
    if "cfg" not in text:
        return False
    if re.search(r"not\s*\(\s*test\s*\)", text):
        return False
    return re.search(r"\btest\b", text) is not None


def gated_scope_lines(path: pathlib.Path) -> set[int]:
    """Line numbers (0-based) inside a test-gated item."""
    lines = path.read_text().splitlines()
    scoped: set[int] = set()
    i = 0
    while i < len(lines):
        code = strip_comments_and_strings(lines[i]).strip()
        if not code.startswith("#[") or not attribute_mentions_test(lines, i):
            i += 1
            continue

        # Skip any further attributes, then find the item's opening brace.
        j = balanced_attribute_end(lines, i) + 1
        while j < len(lines) and strip_comments_and_strings(lines[j]).strip().startswith("#["):
            j = balanced_attribute_end(lines, j) + 1
        open_line = next(
            (k for k in range(j, len(lines)) if "{" in strip_comments_and_strings(lines[k])),
            None,
        )
        if open_line is None:
            scoped.update(range(i, min(j + 1, len(lines))))
            i = j + 1
            continue

        depth, k = 0, open_line
        while k < len(lines):
            brace_code = strip_comments_and_strings(lines[k])
            depth += brace_code.count("{") - brace_code.count("}")
            if depth <= 0:
                break
            k += 1
        scoped.update(range(i, k + 1))
        i = k + 1
    return scoped


def prelude_block_lines(path: pathlib.Path) -> set[int]:
    """Lines inside `pub mod prelude { .. }` — the one sanctioned aggregator."""
    if path.name != "lib.rs":
        return set()
    lines = path.read_text().splitlines()
    start = next(
        (i for i, l in enumerate(lines) if re.match(r"\s*pub mod prelude\b", l)),
        None,
    )
    if start is None:
        return set()
    depth, k = 0, start
    while k < len(lines):
        code = strip_comments_and_strings(lines[k])
        depth += code.count("{") - code.count("}")
        if depth <= 0 and k > start:
            break
        k += 1
    return set(range(start, k + 1))


def split_top_level(text: str) -> list[str]:
    parts, depth, current = [], 0, []
    for ch in text:
        if ch == "{":
            depth += 1
        elif ch == "}":
            depth -= 1
        if ch == "," and depth == 0:
            parts.append("".join(current))
            current = []
        else:
            current.append(ch)
    if current:
        parts.append("".join(current))
    return parts


def expand_use_tree(body: str) -> list[str]:
    """`a::{b, c::d}` -> ["a::b", "a::c::d"] (aliases dropped)."""
    body = body.strip()
    if "{" not in body:
        return [body.split(" as ")[0].strip()]

    prefix, rest = body.split("{", 1)
    depth, i = 1, 0
    while i < len(rest) and depth > 0:
        if rest[i] == "{":
            depth += 1
        elif rest[i] == "}":
            depth -= 1
        i += 1
    inner = rest[: i - 1]

    prefix = prefix.rstrip(": ").strip()
    out: list[str] = []
    for part in split_top_level(inner):
        part = part.strip()
        if not part:
            continue
        if part == "self":
            out.append(prefix)
            continue
        for sub in expand_use_tree(part):
            out.append(f"{prefix}::{sub}" if prefix else sub)
    return out


def iter_use_statements(path: pathlib.Path):
    """Yield (start_line, end_line, text) for each `use` statement."""
    lines = path.read_text().splitlines()
    i = 0
    while i < len(lines):
        code = strip_comments_and_strings(lines[i])
        if not re.match(r"\s*(?:pub(?:\s*\([^)]*\))?\s+)?use\b", code):
            i += 1
            continue
        start = i
        chunks: list[str] = []
        while i < len(lines):
            chunk = strip_comments_and_strings(lines[i])
            chunks.append(chunk.strip())
            if ";" in chunk:
                break
            i += 1
        yield start, i, " ".join(c for c in chunks if c)
        i += 1


# ── policy evaluation ────────────────────────────────────────────────────────


def build_index():
    files = sorted(SRC.rglob("*.rs"))
    module_paths = {module_path_of(p) for p in files}
    return files, module_paths


def discover_facades(files, module_paths):
    """Facade module path -> (owner file, {re-exported item names})."""
    facades: dict[str, tuple[pathlib.Path, set[str]]] = {}
    for path in files:
        mod_path = module_path_of(path)
        for _start, _end, stmt in iter_use_statements(path):
            if not stmt.lstrip().startswith("pub use"):
                continue
            body = stmt.split("use", 1)[1].strip().rstrip(";").strip()
            if "::" not in body:
                continue
            first = body.split("::", 1)[0].strip()
            if first in ("crate", "super", "self"):
                continue
            if f"{mod_path}::{first}" not in module_paths:
                continue
            owner, known = facades.setdefault(mod_path, (path, set()))
            known.update(
                p.split("::")[-1].split(" as ")[0].strip() for p in expand_use_tree(body)
            )
    return facades


def normalize(path: str, mod_path: str, module_paths: set[str]):
    """Resolve a `use` path to an absolute `crate::…` path when possible."""
    segs = path.split("::")
    if segs[0] == "crate":
        base, segs = ["crate"], segs[1:]
    elif segs[0] == "super":
        base = mod_path.split("::")
        while segs and segs[0] in ("super", "self"):
            if segs[0] == "super" and base:
                base.pop()
            segs = segs[1:]
    elif segs[0] == "self":
        base, segs = mod_path.split("::"), segs[1:]
    else:
        if f"{mod_path}::{segs[0]}" not in module_paths:
            return None  # external crate
        base, segs = mod_path.split("::"), segs
    return "::".join([*base, *segs])


def facade_hit(resolved: str, facades, own_file, path) -> str | None:
    """Return the facade path if `resolved` reaches a facade item."""
    for facade_path, (owner, items) in facades.items():
        if path == owner:
            continue
        prefix = facade_path + "::"
        if resolved.startswith(prefix) and resolved[len(prefix):] in items:
            return facade_path
    return None


def main() -> int:
    files, module_paths = build_index()
    facades = discover_facades(files, module_paths)
    violations: list[str] = []

    for path in files:
        mod_path = module_path_of(path)
        exempt = gated_scope_lines(path) | prelude_block_lines(path)
        lines = path.read_text().splitlines()

        covered: set[int] = set()
        for start, end, stmt in iter_use_statements(path):
            covered.update(range(start, end + 1))
            if start in exempt:
                continue
            where = f"{path}:{start + 1}"

            if re.search(r"::\s*\*", stmt):
                violations.append(f"{where}: glob import in non-test code: {stmt.strip()}")

            body = stmt.split("use", 1)[1].strip().rstrip(";").strip()
            for raw in expand_use_tree(body):
                resolved = normalize(raw, mod_path, module_paths)
                if resolved is None:
                    continue
                if resolved == "crate::prelude" or resolved.startswith("crate::prelude::"):
                    violations.append(
                        f"{where}: library code imports the prelude (`{raw}`) — "
                        f"name the real module instead"
                    )
                    continue
                facade = facade_hit(resolved, facades, path, path)
                if facade:
                    violations.append(
                        f"{where}: imports `{raw}` through the `{facade}` facade — "
                        f"import it from the submodule that defines it"
                    )

        # Fully-qualified inline paths, e.g. `pub(crate) x: crate::a::b::Item`.
        for i, line in enumerate(lines):
            if i in exempt or i in covered:
                continue
            code = strip_comments_and_strings(line)
            for m in re.finditer(r"crate(?:::[A-Za-z_][A-Za-z0-9_]*)+", code):
                resolved = m.group(0)
                if resolved == "crate::prelude" or resolved.startswith("crate::prelude::"):
                    violations.append(
                        f"{path}:{i + 1}: library code refers to the prelude (`{resolved}`)"
                    )
                    continue
                facade = facade_hit(resolved, facades, path, path)
                if facade:
                    violations.append(
                        f"{path}:{i + 1}: refers to `{resolved}` through the `{facade}` "
                        f"facade — name the defining submodule"
                    )

    unique = sorted(set(violations))
    if unique:
        print("import policy violations:\n")
        for v in unique:
            print("  " + v)
        print(f"\n{len(unique)} violation(s).")
        return 1

    print(
        f"import policy OK ({len(facades)} facade module(s), {len(files)} file(s) scanned)"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
