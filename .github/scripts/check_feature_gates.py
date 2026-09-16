#!/usr/bin/env python3
"""Reject feature gates that restate a condition already guaranteed by the
module tree.

A module reached only through `#[cfg(feature = "F")] mod x;` cannot be compiled
without `F`, so a `#[cfg(feature = "F")]` inside it is a no-op. Those accumulate
silently - they read as if they were doing something - and hide which gates are
actually load-bearing.

Only gates that are *provably* redundant are reported:

  * the enclosing declaration is a bare `feature = "F"`, or an `all(...)`
    containing it - in both cases `F` holds for everything inside;
  * `any(...)` establishes nothing (a sibling feature could be the one enabled),
    and `not(...)` is not reasoned about at all;
  * only single-feature gates are reported. `all(feature = "F", feature = "G")`
    inside an `F`-gated module still says something about `G`, so it is kept.

Run from the repository root. Exits non-zero when a redundant gate is found.
"""

from __future__ import annotations

import pathlib
import re
import sys

SRC = pathlib.Path("src")
ROOT = SRC / "lib.rs"

# `#[cfg(...)]` lines (possibly several) directly above a `mod x;` declaration
MOD_DECL = re.compile(
    r"(?P<attrs>(?:^[ \t]*#\[[^\]]*\][ \t]*\n)*)"
    r"^[ \t]*(?:pub(?:\([^)]*\))?[ \t]+)?mod[ \t]+(?P<name>[A-Za-z_][A-Za-z0-9_]*)[ \t]*;",
    re.M,
)
FEATURE = re.compile(r'feature[ \t]*=[ \t]*"([^"]+)"')


def guaranteed_features(attrs: str) -> set[str]:
    """Features that certainly hold inside a module declared with `attrs`."""
    attrs = attrs.strip()
    if not attrs.startswith("#[cfg("):
        return set()
    body = attrs[len("#[cfg(") :].rstrip().rstrip("]").rstrip(")")
    # `any(...)` guarantees nothing; `not(...)` is not reasoned about
    if body.startswith("any(") or body.startswith("not("):
        return set()
    return set(FEATURE.findall(body))


def module_file(parent: pathlib.Path, name: str) -> pathlib.Path | None:
    """Resolve `mod name;` declared in `parent`."""
    base = parent.parent / name
    for candidate in (base.with_suffix(".rs"), base / "mod.rs"):
        if candidate.is_file():
            return candidate
    return None


def walk() -> dict[pathlib.Path, dict[str, tuple[pathlib.Path, int]]]:
    """Map each module file to the features guaranteed by its ancestors,
    remembering which declaration established each one."""
    if not ROOT.is_file():
        sys.exit(f"error: {ROOT} not found - run from the repository root")

    guaranteed: dict[pathlib.Path, dict[str, tuple[pathlib.Path, int]]] = {ROOT: {}}
    queue = [ROOT]
    while queue:
        parent = queue.pop()
        text = parent.read_text(encoding="utf-8")
        for match in MOD_DECL.finditer(text):
            child = module_file(parent, match.group("name"))
            if child is None:
                continue  # `#[path]`-only or cfg'd-out module
            line = text[: match.start("attrs")].count("\n") + 1
            inherited = dict(guaranteed[parent])
            for feature in guaranteed_features(match.group("attrs")):
                inherited.setdefault(feature, (parent, line))
            existing = guaranteed.setdefault(child, {})
            existing.update({k: v for k, v in inherited.items() if k not in existing})
            queue.append(child)
    return guaranteed


def main() -> int:
    guaranteed = walk()
    violations: list[tuple[pathlib.Path, int, str, tuple[pathlib.Path, int]]] = []

    for path, inherited in sorted(guaranteed.items()):
        if not inherited:
            continue
        for lineno, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
            stripped = line.strip()
            if not stripped.startswith("#[cfg(") or stripped.startswith("#[cfg(any("):
                continue
            if stripped.startswith("#[cfg(all(") or stripped.startswith("#[cfg(not("):
                continue
            features = FEATURE.findall(stripped)
            if len(features) != 1:
                continue  # only bare `feature = "F"` gates are provably no-ops
            if features[0] in inherited:
                violations.append((path, lineno, stripped, inherited[features[0]]))

    if not violations:
        print(f"feature gates OK ({len(guaranteed)} module(s) checked)")
        return 0

    print("redundant feature gates:\n")
    for path, lineno, gate, (declared_in, decl_line) in violations:
        print(f"  {path}:{lineno}: {gate}")
        print(f"      `{path.name}` is only reachable under this feature, "
              f"guaranteed at {declared_in}:{decl_line}")
    print(f"\n{len(violations)} redundant gate(s). Remove each `#[cfg(...)]`, or "
          f"move it to where the condition can still vary.")
    return 1


if __name__ == "__main__":
    sys.exit(main())
