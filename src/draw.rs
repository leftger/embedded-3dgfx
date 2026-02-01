use core::fmt::Debug;
use embedded_graphics_core::draw_target::DrawTarget;
use embedded_graphics_core::pixelcolor::RgbColor;
use embedded_graphics_core::prelude::Point;
use heapless::Vec;

use crate::DrawPrimitive;

#[inline]
pub fn draw<D: DrawTarget<Color = embedded_graphics_core::pixelcolor::Rgb565>>(
    primitive: DrawPrimitive,
    fb: &mut D,
) where
    <D as DrawTarget>::Error: Debug,
{
    match primitive {
        DrawPrimitive::Line([p1, p2], color) => {
            fb.draw_iter(
                line_drawing::Bresenham::new((p1.x, p1.y), (p2.x, p2.y))
                    .map(|(x, y)| embedded_graphics_core::Pixel(Point::new(x, y), color)),
            )
            .unwrap();
        }
        DrawPrimitive::ColoredPoint(p, c) => {
            let p = embedded_graphics_core::geometry::Point::new(p.x, p.y);

            fb.draw_iter([embedded_graphics_core::Pixel(p, c)]).unwrap();
        }
        DrawPrimitive::ColoredTriangle(mut vertices, color) => {
            // sort vertices by y using bubble sort (since there are exactly 3 elements)
            if vertices[0].y > vertices[1].y {
                vertices.swap(0, 1);
            }
            if vertices[0].y > vertices[2].y {
                vertices.swap(0, 2);
            }
            if vertices[1].y > vertices[2].y {
                vertices.swap(1, 2);
            }

            let mut buf: Vec<_, 3> = Vec::new();
            for p in vertices.iter() {
                buf.push(embedded_graphics_core::geometry::Point::new(p.x, p.y))
                    .unwrap();
            }
            let [p1, p2, p3] = buf.into_array().unwrap();

            if p2.y == p3.y {
                fill_bottom_flat_triangle(p1, p2, p3, color, fb);
            } else if p1.y == p2.y {
                fill_top_flat_triangle(p1, p2, p3, color, fb);
            } else {
                let p4 = Point::new(
                    (p1.x as f32
                        + ((p2.y - p1.y) as f32 / (p3.y - p1.y) as f32) * (p3.x - p1.x) as f32)
                        as i32,
                    p2.y,
                );

                fill_bottom_flat_triangle(p1, p2, p4, color, fb);
                fill_top_flat_triangle(p2, p4, p3, color, fb);
            }
        }
        DrawPrimitive::ColoredTriangleWithDepth { points, depths: _, color } => {
            // This variant should use draw_zbuffered() instead
            // For compatibility, render without Z-buffering (ignoring depths)
            let mut vertices = points;
            if vertices[0].y > vertices[1].y {
                vertices.swap(0, 1);
            }
            if vertices[0].y > vertices[2].y {
                vertices.swap(0, 2);
            }
            if vertices[1].y > vertices[2].y {
                vertices.swap(1, 2);
            }

            let mut buf: Vec<_, 3> = Vec::new();
            for p in vertices.iter() {
                buf.push(embedded_graphics_core::geometry::Point::new(p.x, p.y))
                    .unwrap();
            }
            let [p1, p2, p3] = buf.into_array().unwrap();

            if p2.y == p3.y {
                fill_bottom_flat_triangle(p1, p2, p3, color, fb);
            } else if p1.y == p2.y {
                fill_top_flat_triangle(p1, p2, p3, color, fb);
            } else {
                let p4 = Point::new(
                    (p1.x as f32
                        + ((p2.y - p1.y) as f32 / (p3.y - p1.y) as f32) * (p3.x - p1.x) as f32)
                        as i32,
                    p2.y,
                );

                fill_bottom_flat_triangle(p1, p2, p4, color, fb);
                fill_top_flat_triangle(p2, p4, p3, color, fb);
            }
        }
        DrawPrimitive::GouraudTriangle { mut points, mut colors } => {
            // Sort vertices by y coordinate (and corresponding colors)
            if points[0].y > points[1].y {
                points.swap(0, 1);
                colors.swap(0, 1);
            }
            if points[0].y > points[2].y {
                points.swap(0, 2);
                colors.swap(0, 2);
            }
            if points[1].y > points[2].y {
                points.swap(1, 2);
                colors.swap(1, 2);
            }

            let mut buf: Vec<_, 3> = Vec::new();
            for p in points.iter() {
                buf.push(embedded_graphics_core::geometry::Point::new(p.x, p.y))
                    .unwrap();
            }
            let [p1, p2, p3] = buf.into_array().unwrap();
            let [c1, c2, c3] = colors;

            if p2.y == p3.y {
                fill_bottom_flat_gouraud(p1, p2, p3, c1, c2, c3, fb);
            } else if p1.y == p2.y {
                fill_top_flat_gouraud(p1, p2, p3, c1, c2, c3, fb);
            } else {
                // Split triangle into two flat triangles
                let t = (p2.y - p1.y) as f32 / (p3.y - p1.y) as f32;
                let p4 = Point::new(
                    (p1.x as f32 + t * (p3.x - p1.x) as f32) as i32,
                    p2.y,
                );
                // Interpolate color at split point
                let c4 = interpolate_color(c1, c3, t);

                fill_bottom_flat_gouraud(p1, p2, p4, c1, c2, c4, fb);
                fill_top_flat_gouraud(p2, p4, p3, c2, c4, c3, fb);
            }
        }
        DrawPrimitive::GouraudTriangleWithDepth { points, depths: _, colors } => {
            // This variant should use draw_zbuffered() instead
            // For compatibility, render without Z-buffering (ignoring depths)
            let prim = DrawPrimitive::GouraudTriangle { points, colors };
            draw(prim, fb);
        }
    }
}

fn fill_bottom_flat_triangle<D: DrawTarget<Color = embedded_graphics_core::pixelcolor::Rgb565>>(
    p1: Point,
    p2: Point,
    p3: Point,
    color: embedded_graphics_core::pixelcolor::Rgb565,
    fb: &mut D,
) where
    <D as DrawTarget>::Error: Debug,
{
    let invslope1 = (p2.x - p1.x) as f32 / (p2.y - p1.y) as f32;
    let invslope2 = (p3.x - p1.x) as f32 / (p3.y - p1.y) as f32;

    let mut curx1 = p1.x as f32;
    let mut curx2 = p1.x as f32;

    for scanline_y in p1.y..=p2.y {
        draw_horizontal_line(
            Point::new(curx1 as i32, scanline_y),
            Point::new(curx2 as i32, scanline_y),
            color,
            fb,
        );

        curx1 += invslope1;
        curx2 += invslope2;
    }
}

fn fill_top_flat_triangle<D: DrawTarget<Color = embedded_graphics_core::pixelcolor::Rgb565>>(
    p1: Point,
    p2: Point,
    p3: Point,
    color: embedded_graphics_core::pixelcolor::Rgb565,
    fb: &mut D,
) where
    <D as DrawTarget>::Error: Debug,
{
    let invslope1 = (p3.x - p1.x) as f32 / (p3.y - p1.y) as f32;
    let invslope2 = (p3.x - p2.x) as f32 / (p3.y - p2.y) as f32;

    let mut curx1 = p3.x as f32;
    let mut curx2 = p3.x as f32;

    for scanline_y in (p1.y..=p3.y).rev() {
        draw_horizontal_line(
            Point::new(curx1 as i32, scanline_y),
            Point::new(curx2 as i32, scanline_y),
            color,
            fb,
        );

        curx1 -= invslope1;
        curx2 -= invslope2;
    }
}

fn draw_horizontal_line<D: DrawTarget<Color = embedded_graphics_core::pixelcolor::Rgb565>>(
    p1: Point,
    p2: Point,
    color: embedded_graphics_core::pixelcolor::Rgb565,
    fb: &mut D,
) where
    <D as DrawTarget>::Error: Debug,
{
    let start = p1.x.min(p2.x);
    let end = p1.x.max(p2.x);

    for x in start..=end {
        fb.draw_iter([embedded_graphics_core::Pixel(Point::new(x, p1.y), color)])
            .unwrap();
    }
}

// Interpolate between two colors
#[inline]
fn interpolate_color(
    c1: embedded_graphics_core::pixelcolor::Rgb565,
    c2: embedded_graphics_core::pixelcolor::Rgb565,
    t: f32,
) -> embedded_graphics_core::pixelcolor::Rgb565 {
    let r1 = c1.r() as f32;
    let g1 = c1.g() as f32;
    let b1 = c1.b() as f32;

    let r2 = c2.r() as f32;
    let g2 = c2.g() as f32;
    let b2 = c2.b() as f32;

    let r = (r1 + t * (r2 - r1)) as u8;
    let g = (g1 + t * (g2 - g1)) as u8;
    let b = (b1 + t * (b2 - b1)) as u8;

    embedded_graphics_core::pixelcolor::Rgb565::new(r, g, b)
}

// Gouraud shading - bottom flat triangle with color interpolation
fn fill_bottom_flat_gouraud<D: DrawTarget<Color = embedded_graphics_core::pixelcolor::Rgb565>>(
    p1: Point,
    p2: Point,
    p3: Point,
    c1: embedded_graphics_core::pixelcolor::Rgb565,
    c2: embedded_graphics_core::pixelcolor::Rgb565,
    c3: embedded_graphics_core::pixelcolor::Rgb565,
    fb: &mut D,
) where
    <D as DrawTarget>::Error: Debug,
{
    let height = (p2.y - p1.y) as f32;
    if height == 0.0 {
        return;
    }

    let invslope1 = (p2.x - p1.x) as f32 / height;
    let invslope2 = (p3.x - p1.x) as f32 / height;

    let mut curx1 = p1.x as f32;
    let mut curx2 = p1.x as f32;

    for scanline_y in p1.y..=p2.y {
        let t = (scanline_y - p1.y) as f32 / height;
        let color_left = interpolate_color(c1, c2, t);
        let color_right = interpolate_color(c1, c3, t);

        draw_horizontal_line_gouraud(
            Point::new(curx1 as i32, scanline_y),
            Point::new(curx2 as i32, scanline_y),
            color_left,
            color_right,
            fb,
        );

        curx1 += invslope1;
        curx2 += invslope2;
    }
}

// Gouraud shading - top flat triangle with color interpolation
fn fill_top_flat_gouraud<D: DrawTarget<Color = embedded_graphics_core::pixelcolor::Rgb565>>(
    p1: Point,
    p2: Point,
    p3: Point,
    c1: embedded_graphics_core::pixelcolor::Rgb565,
    c2: embedded_graphics_core::pixelcolor::Rgb565,
    c3: embedded_graphics_core::pixelcolor::Rgb565,
    fb: &mut D,
) where
    <D as DrawTarget>::Error: Debug,
{
    let height = (p3.y - p1.y) as f32;
    if height == 0.0 {
        return;
    }

    let invslope1 = (p3.x - p1.x) as f32 / height;
    let invslope2 = (p3.x - p2.x) as f32 / height;

    let mut curx1 = p3.x as f32;
    let mut curx2 = p3.x as f32;

    for scanline_y in (p1.y..=p3.y).rev() {
        let t = (scanline_y - p1.y) as f32 / height;
        let color_left = interpolate_color(c1, c3, t);
        let color_right = interpolate_color(c2, c3, t);

        draw_horizontal_line_gouraud(
            Point::new(curx1 as i32, scanline_y),
            Point::new(curx2 as i32, scanline_y),
            color_left,
            color_right,
            fb,
        );

        curx1 -= invslope1;
        curx2 -= invslope2;
    }
}

// Draw a horizontal line with color interpolation (Gouraud)
fn draw_horizontal_line_gouraud<D: DrawTarget<Color = embedded_graphics_core::pixelcolor::Rgb565>>(
    p1: Point,
    p2: Point,
    color1: embedded_graphics_core::pixelcolor::Rgb565,
    color2: embedded_graphics_core::pixelcolor::Rgb565,
    fb: &mut D,
) where
    <D as DrawTarget>::Error: Debug,
{
    let start = p1.x.min(p2.x);
    let end = p1.x.max(p2.x);
    let width = (end - start) as f32;

    if width == 0.0 {
        fb.draw_iter([embedded_graphics_core::Pixel(Point::new(start, p1.y), color1)])
            .unwrap();
        return;
    }

    for x in start..=end {
        let t = (x - start) as f32 / width;
        let color = interpolate_color(color1, color2, t);
        fb.draw_iter([embedded_graphics_core::Pixel(Point::new(x, p1.y), color)])
            .unwrap();
    }
}

// Z-buffered drawing function
// Using u32 for Z-buffer is much faster on embedded systems without FPU
#[inline]
pub fn draw_zbuffered<D: DrawTarget<Color = embedded_graphics_core::pixelcolor::Rgb565>>(
    primitive: DrawPrimitive,
    fb: &mut D,
    zbuffer: &mut [u32],
    width: usize,
) where
    <D as DrawTarget>::Error: Debug,
{
    match primitive {
        DrawPrimitive::ColoredTriangleWithDepth { mut points, mut depths, color } => {
            // Sort vertices by y coordinate (and corresponding depths)
            if points[0].y > points[1].y {
                points.swap(0, 1);
                depths.swap(0, 1);
            }
            if points[0].y > points[2].y {
                points.swap(0, 2);
                depths.swap(0, 2);
            }
            if points[1].y > points[2].y {
                points.swap(1, 2);
                depths.swap(1, 2);
            }

            let [p1, p2, p3] = points;
            let [z1, z2, z3] = depths;

            fill_triangle_zbuffered(p1, p2, p3, z1, z2, z3, color, fb, zbuffer, width);
        }
        DrawPrimitive::GouraudTriangleWithDepth { mut points, mut depths, mut colors } => {
            // Sort vertices by y coordinate (and corresponding depths and colors)
            if points[0].y > points[1].y {
                points.swap(0, 1);
                depths.swap(0, 1);
                colors.swap(0, 1);
            }
            if points[0].y > points[2].y {
                points.swap(0, 2);
                depths.swap(0, 2);
                colors.swap(0, 2);
            }
            if points[1].y > points[2].y {
                points.swap(1, 2);
                depths.swap(1, 2);
                colors.swap(1, 2);
            }

            let [p1, p2, p3] = points;
            let [z1, z2, z3] = depths;
            let [c1, c2, c3] = colors;

            fill_triangle_zbuffered_gouraud(p1, p2, p3, z1, z2, z3, c1, c2, c3, fb, zbuffer, width);
        }
        // For other primitives, fall back to regular drawing
        _ => draw(primitive, fb),
    }
}

#[inline(always)]
fn fill_triangle_zbuffered<D: DrawTarget<Color = embedded_graphics_core::pixelcolor::Rgb565>>(
    p1: nalgebra::Point2<i32>,
    p2: nalgebra::Point2<i32>,
    p3: nalgebra::Point2<i32>,
    z1: f32,
    z2: f32,
    z3: f32,
    color: embedded_graphics_core::pixelcolor::Rgb565,
    fb: &mut D,
    zbuffer: &mut [u32],
    width: usize,
) where
    <D as DrawTarget>::Error: Debug,
{
    // Convert float depths to fixed-point integers (16.16 format)
    // This avoids floating-point operations in the inner loop
    let z1_int = (z1 * 65536.0) as u32;
    let z2_int = (z2 * 65536.0) as u32;
    let z3_int = (z3 * 65536.0) as u32;
    // Convert to embedded_graphics Points
    let p1_eg = Point::new(p1.x, p1.y);
    let p2_eg = Point::new(p2.x, p2.y);
    let p3_eg = Point::new(p3.x, p3.y);

    // Handle flat triangles
    if p2_eg.y == p3_eg.y {
        fill_bottom_flat_triangle_zbuffered(p1_eg, p2_eg, p3_eg, z1_int, z2_int, z3_int, color, fb, zbuffer, width);
    } else if p1_eg.y == p2_eg.y {
        fill_top_flat_triangle_zbuffered(p1_eg, p2_eg, p3_eg, z1_int, z2_int, z3_int, color, fb, zbuffer, width);
    } else {
        // Split into two flat triangles
        let t = (p2_eg.y - p1_eg.y) as f32 / (p3_eg.y - p1_eg.y) as f32;
        let p4 = Point::new(
            (p1_eg.x as f32 + t * (p3_eg.x - p1_eg.x) as f32) as i32,
            p2_eg.y,
        );
        let z4_int = (z1_int as i64 + (t * (z3_int as i64 - z1_int as i64) as f32) as i64) as u32;

        fill_bottom_flat_triangle_zbuffered(p1_eg, p2_eg, p4, z1_int, z2_int, z4_int, color, fb, zbuffer, width);
        fill_top_flat_triangle_zbuffered(p2_eg, p4, p3_eg, z2_int, z4_int, z3_int, color, fb, zbuffer, width);
    }
}

// Gouraud-shaded triangle with z-buffering
#[inline(always)]
fn fill_triangle_zbuffered_gouraud<D: DrawTarget<Color = embedded_graphics_core::pixelcolor::Rgb565>>(
    p1: nalgebra::Point2<i32>,
    p2: nalgebra::Point2<i32>,
    p3: nalgebra::Point2<i32>,
    z1: f32,
    z2: f32,
    z3: f32,
    c1: embedded_graphics_core::pixelcolor::Rgb565,
    c2: embedded_graphics_core::pixelcolor::Rgb565,
    c3: embedded_graphics_core::pixelcolor::Rgb565,
    fb: &mut D,
    zbuffer: &mut [u32],
    width: usize,
) where
    <D as DrawTarget>::Error: Debug,
{
    // Convert float depths to fixed-point integers (16.16 format)
    let z1_int = (z1 * 65536.0) as u32;
    let z2_int = (z2 * 65536.0) as u32;
    let z3_int = (z3 * 65536.0) as u32;

    // Convert to embedded_graphics Points
    let p1_eg = Point::new(p1.x, p1.y);
    let p2_eg = Point::new(p2.x, p2.y);
    let p3_eg = Point::new(p3.x, p3.y);

    // Handle flat triangles
    if p2_eg.y == p3_eg.y {
        fill_bottom_flat_triangle_zbuffered_gouraud(p1_eg, p2_eg, p3_eg, z1_int, z2_int, z3_int, c1, c2, c3, fb, zbuffer, width);
    } else if p1_eg.y == p2_eg.y {
        fill_top_flat_triangle_zbuffered_gouraud(p1_eg, p2_eg, p3_eg, z1_int, z2_int, z3_int, c1, c2, c3, fb, zbuffer, width);
    } else {
        // Split into two flat triangles
        let t = (p2_eg.y - p1_eg.y) as f32 / (p3_eg.y - p1_eg.y) as f32;
        let p4 = Point::new(
            (p1_eg.x as f32 + t * (p3_eg.x - p1_eg.x) as f32) as i32,
            p2_eg.y,
        );
        let z4_int = (z1_int as i64 + (t * (z3_int as i64 - z1_int as i64) as f32) as i64) as u32;
        let c4 = interpolate_color(c1, c3, t);

        fill_bottom_flat_triangle_zbuffered_gouraud(p1_eg, p2_eg, p4, z1_int, z2_int, z4_int, c1, c2, c4, fb, zbuffer, width);
        fill_top_flat_triangle_zbuffered_gouraud(p2_eg, p4, p3_eg, z2_int, z4_int, z3_int, c2, c4, c3, fb, zbuffer, width);
    }
}

#[inline(always)]
fn fill_bottom_flat_triangle_zbuffered<D: DrawTarget<Color = embedded_graphics_core::pixelcolor::Rgb565>>(
    p1: Point,
    p2: Point,
    p3: Point,
    z1: u32,
    z2: u32,
    z3: u32,
    color: embedded_graphics_core::pixelcolor::Rgb565,
    fb: &mut D,
    zbuffer: &mut [u32],
    width: usize,
) where
    <D as DrawTarget>::Error: Debug,
{
    let height = p2.y - p1.y;
    if height == 0 {
        return;
    }

    // Use fixed-point arithmetic (16.16 format) for edge slopes
    // This avoids floating-point operations entirely
    let invslope1 = ((p2.x - p1.x) << 16) / height;
    let invslope2 = ((p3.x - p1.x) << 16) / height;

    let mut curx1 = p1.x << 16; // Fixed-point
    let mut curx2 = p1.x << 16; // Fixed-point

    for scanline_y in p1.y..=p2.y {
        let dy = scanline_y - p1.y;
        // Integer interpolation for Z using only integer math
        let z_left = if height > 0 {
            (z1 as i64 + ((z2 as i64 - z1 as i64) * dy as i64 / height as i64)) as u32
        } else {
            z1
        };
        let z_right = if height > 0 {
            (z1 as i64 + ((z3 as i64 - z1 as i64) * dy as i64 / height as i64)) as u32
        } else {
            z1
        };

        draw_scanline_zbuffered(
            curx1 >> 16, // Convert back from fixed-point
            curx2 >> 16, // Convert back from fixed-point
            scanline_y,
            z_left,
            z_right,
            color,
            fb,
            zbuffer,
            width,
        );

        curx1 += invslope1;
        curx2 += invslope2;
    }
}

#[inline(always)]
fn fill_top_flat_triangle_zbuffered<D: DrawTarget<Color = embedded_graphics_core::pixelcolor::Rgb565>>(
    p1: Point,
    p2: Point,
    p3: Point,
    z1: u32,
    z2: u32,
    z3: u32,
    color: embedded_graphics_core::pixelcolor::Rgb565,
    fb: &mut D,
    zbuffer: &mut [u32],
    width: usize,
) where
    <D as DrawTarget>::Error: Debug,
{
    let height = p3.y - p1.y;
    if height == 0 {
        return;
    }

    // Use fixed-point arithmetic (16.16 format) for edge slopes
    let invslope1 = ((p3.x - p1.x) << 16) / height;
    let invslope2 = ((p3.x - p2.x) << 16) / height;

    let mut curx1 = p3.x << 16; // Fixed-point
    let mut curx2 = p3.x << 16; // Fixed-point

    for scanline_y in (p1.y..=p3.y).rev() {
        let dy = scanline_y - p1.y;
        // Integer interpolation for Z using only integer math
        let z_left = if height > 0 {
            (z1 as i64 + ((z3 as i64 - z1 as i64) * dy as i64 / height as i64)) as u32
        } else {
            z1
        };
        let z_right = if height > 0 {
            (z2 as i64 + ((z3 as i64 - z2 as i64) * dy as i64 / height as i64)) as u32
        } else {
            z2
        };

        draw_scanline_zbuffered(
            curx1 >> 16, // Convert back from fixed-point
            curx2 >> 16, // Convert back from fixed-point
            scanline_y,
            z_left,
            z_right,
            color,
            fb,
            zbuffer,
            width,
        );

        curx1 -= invslope1;
        curx2 -= invslope2;
    }
}

// Gouraud shaded bottom-flat triangle with z-buffering
#[inline(always)]
fn fill_bottom_flat_triangle_zbuffered_gouraud<D: DrawTarget<Color = embedded_graphics_core::pixelcolor::Rgb565>>(
    p1: Point,
    p2: Point,
    p3: Point,
    z1: u32,
    z2: u32,
    z3: u32,
    c1: embedded_graphics_core::pixelcolor::Rgb565,
    c2: embedded_graphics_core::pixelcolor::Rgb565,
    c3: embedded_graphics_core::pixelcolor::Rgb565,
    fb: &mut D,
    zbuffer: &mut [u32],
    width: usize,
) where
    <D as DrawTarget>::Error: Debug,
{
    let height = p2.y - p1.y;
    if height == 0 {
        return;
    }

    let invslope1 = ((p2.x - p1.x) << 16) / height;
    let invslope2 = ((p3.x - p1.x) << 16) / height;

    let mut curx1 = p1.x << 16;
    let mut curx2 = p1.x << 16;

    for scanline_y in p1.y..=p2.y {
        let dy = scanline_y - p1.y;
        let t = dy as f32 / height as f32;

        // Interpolate Z values
        let z_left = if height > 0 {
            (z1 as i64 + ((z2 as i64 - z1 as i64) * dy as i64 / height as i64)) as u32
        } else {
            z1
        };
        let z_right = if height > 0 {
            (z1 as i64 + ((z3 as i64 - z1 as i64) * dy as i64 / height as i64)) as u32
        } else {
            z1
        };

        // Interpolate colors
        let color_left = interpolate_color(c1, c2, t);
        let color_right = interpolate_color(c1, c3, t);

        draw_scanline_zbuffered_gouraud(
            curx1 >> 16,
            curx2 >> 16,
            scanline_y,
            z_left,
            z_right,
            color_left,
            color_right,
            fb,
            zbuffer,
            width,
        );

        curx1 += invslope1;
        curx2 += invslope2;
    }
}

// Gouraud shaded top-flat triangle with z-buffering
#[inline(always)]
fn fill_top_flat_triangle_zbuffered_gouraud<D: DrawTarget<Color = embedded_graphics_core::pixelcolor::Rgb565>>(
    p1: Point,
    p2: Point,
    p3: Point,
    z1: u32,
    z2: u32,
    z3: u32,
    c1: embedded_graphics_core::pixelcolor::Rgb565,
    c2: embedded_graphics_core::pixelcolor::Rgb565,
    c3: embedded_graphics_core::pixelcolor::Rgb565,
    fb: &mut D,
    zbuffer: &mut [u32],
    width: usize,
) where
    <D as DrawTarget>::Error: Debug,
{
    let height = p3.y - p1.y;
    if height == 0 {
        return;
    }

    let invslope1 = ((p3.x - p1.x) << 16) / height;
    let invslope2 = ((p3.x - p2.x) << 16) / height;

    let mut curx1 = p3.x << 16;
    let mut curx2 = p3.x << 16;

    for scanline_y in (p1.y..=p3.y).rev() {
        let dy = scanline_y - p1.y;
        let t = dy as f32 / height as f32;

        // Interpolate Z values
        let z_left = if height > 0 {
            (z1 as i64 + ((z3 as i64 - z1 as i64) * dy as i64 / height as i64)) as u32
        } else {
            z1
        };
        let z_right = if height > 0 {
            (z2 as i64 + ((z3 as i64 - z2 as i64) * dy as i64 / height as i64)) as u32
        } else {
            z2
        };

        // Interpolate colors
        let color_left = interpolate_color(c1, c3, t);
        let color_right = interpolate_color(c2, c3, t);

        draw_scanline_zbuffered_gouraud(
            curx1 >> 16,
            curx2 >> 16,
            scanline_y,
            z_left,
            z_right,
            color_left,
            color_right,
            fb,
            zbuffer,
            width,
        );

        curx1 -= invslope1;
        curx2 -= invslope2;
    }
}

// Draw scanline with Gouraud shading and z-buffering
#[inline(always)]
fn draw_scanline_zbuffered_gouraud<D: DrawTarget<Color = embedded_graphics_core::pixelcolor::Rgb565>>(
    x1: i32,
    x2: i32,
    y: i32,
    z1: u32,
    z2: u32,
    color1: embedded_graphics_core::pixelcolor::Rgb565,
    color2: embedded_graphics_core::pixelcolor::Rgb565,
    fb: &mut D,
    zbuffer: &mut [u32],
    width: usize,
) where
    <D as DrawTarget>::Error: Debug,
{
    let start = x1.min(x2);
    let end = x1.max(x2);
    let span_width = end - start;

    for x in start..=end {
        if x < 0 || y < 0 {
            continue;
        }

        let zbuffer_idx = y as usize * width + x as usize;
        if zbuffer_idx >= zbuffer.len() {
            continue;
        }

        // Interpolate Z value
        let t = if span_width > 0 {
            (x - start) as f32 / span_width as f32
        } else {
            0.0
        };
        let z = if span_width > 0 {
            (z1 as i64 + ((z2 as i64 - z1 as i64) * (x - start) as i64 / span_width as i64)) as u32
        } else {
            z1
        };

        // Z-test
        if z < zbuffer[zbuffer_idx] {
            zbuffer[zbuffer_idx] = z;

            // Interpolate color
            let color = interpolate_color(color1, color2, t);
            fb.draw_iter([embedded_graphics_core::Pixel(Point::new(x, y), color)])
                .unwrap();
        }
    }
}

#[inline(always)]
fn draw_scanline_zbuffered<D: DrawTarget<Color = embedded_graphics_core::pixelcolor::Rgb565>>(
    x1: i32,
    x2: i32,
    y: i32,
    z1: u32,
    z2: u32,
    color: embedded_graphics_core::pixelcolor::Rgb565,
    fb: &mut D,
    zbuffer: &mut [u32],
    width: usize,
) where
    <D as DrawTarget>::Error: Debug,
{
    let start = x1.min(x2);
    let end = x1.max(x2);

    for x in start..=end {
        if x < 0 || y < 0 {
            continue;
        }

        let zbuffer_idx = y as usize * width + x as usize;
        if zbuffer_idx >= zbuffer.len() {
            continue;
        }

        // Integer interpolation for Z (fixed-point 16.16 format)
        // This avoids floating-point in the inner loop
        let span = end - start;
        let z = if span == 0 {
            z1
        } else {
            // Linear interpolation using integer math
            let t_num = (x - start) as i64;
            let t_den = span as i64;
            let z_diff = z2 as i64 - z1 as i64;
            (z1 as i64 + (t_num * z_diff / t_den)) as u32
        };

        // Z-buffer test (closer = smaller depth value)
        if z < zbuffer[zbuffer_idx] {
            zbuffer[zbuffer_idx] = z;
            fb.draw_iter([embedded_graphics_core::Pixel(Point::new(x, y), color)])
                .unwrap();
        }
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use embedded_graphics_core::pixelcolor::Rgb565;
    use embedded_graphics_core::prelude::*;
    use nalgebra::Point2;

    // Mock framebuffer for testing
    struct MockFramebuffer {
        pixels: std::vec::Vec<(i32, i32, Rgb565)>,
    }

    impl MockFramebuffer {
        fn new() -> Self {
            Self {
                pixels: std::vec::Vec::new(),
            }
        }

        fn contains_pixel(&self, x: i32, y: i32) -> bool {
            self.pixels.iter().any(|(px, py, _)| *px == x && *py == y)
        }

        fn pixel_count(&self) -> usize {
            self.pixels.len()
        }
    }

    impl DrawTarget for MockFramebuffer {
        type Color = Rgb565;
        type Error = core::convert::Infallible;

        fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
        where
            I: IntoIterator<Item = embedded_graphics_core::Pixel<Self::Color>>,
        {
            for pixel in pixels {
                self.pixels.push((pixel.0.x, pixel.0.y, pixel.1));
            }
            Ok(())
        }
    }

    impl OriginDimensions for MockFramebuffer {
        fn size(&self) -> Size {
            Size::new(640, 480)
        }
    }

    #[test]
    fn test_draw_point() {
        let mut fb = MockFramebuffer::new();
        let point = Point2::new(10, 20);
        let color = Rgb565::CSS_RED;

        draw(DrawPrimitive::ColoredPoint(point, color), &mut fb);

        assert_eq!(fb.pixel_count(), 1);
        assert!(fb.contains_pixel(10, 20));
    }

    #[test]
    fn test_draw_line_horizontal() {
        let mut fb = MockFramebuffer::new();
        let p1 = Point2::new(10, 20);
        let p2 = Point2::new(20, 20);
        let color = Rgb565::CSS_GREEN;

        draw(DrawPrimitive::Line([p1, p2], color), &mut fb);

        // Should draw pixels along the horizontal line
        assert!(fb.pixel_count() >= 10); // At least 10 pixels
        assert!(fb.contains_pixel(10, 20));
        assert!(fb.contains_pixel(20, 20));
    }

    #[test]
    fn test_draw_line_vertical() {
        let mut fb = MockFramebuffer::new();
        let p1 = Point2::new(10, 10);
        let p2 = Point2::new(10, 20);
        let color = Rgb565::CSS_BLUE;

        draw(DrawPrimitive::Line([p1, p2], color), &mut fb);

        // Should draw pixels along the vertical line
        assert!(fb.pixel_count() >= 10);
        assert!(fb.contains_pixel(10, 10));
        assert!(fb.contains_pixel(10, 20));
    }

    #[test]
    fn test_draw_line_diagonal() {
        let mut fb = MockFramebuffer::new();
        let p1 = Point2::new(0, 0);
        let p2 = Point2::new(10, 10);
        let color = Rgb565::CSS_WHITE;

        draw(DrawPrimitive::Line([p1, p2], color), &mut fb);

        // Should draw pixels along the diagonal
        assert!(fb.pixel_count() >= 10);
        assert!(fb.contains_pixel(0, 0));
        assert!(fb.contains_pixel(10, 10));
    }

    #[test]
    fn test_draw_triangle_flat_bottom() {
        let mut fb = MockFramebuffer::new();
        let vertices = [
            Point2::new(50, 10),  // Top vertex
            Point2::new(30, 30),  // Bottom left
            Point2::new(70, 30),  // Bottom right
        ];
        let color = Rgb565::CSS_YELLOW;

        draw(DrawPrimitive::ColoredTriangle(vertices, color), &mut fb);

        // Should draw multiple pixels for the filled triangle
        assert!(fb.pixel_count() > 20);
        // Top vertex should be drawn
        assert!(fb.contains_pixel(50, 10));
    }

    #[test]
    fn test_draw_triangle_flat_top() {
        let mut fb = MockFramebuffer::new();
        let vertices = [
            Point2::new(30, 10),  // Top left
            Point2::new(70, 10),  // Top right
            Point2::new(50, 30),  // Bottom vertex
        ];
        let color = Rgb565::CSS_CYAN;

        draw(DrawPrimitive::ColoredTriangle(vertices, color), &mut fb);

        // Should draw multiple pixels for the filled triangle
        assert!(fb.pixel_count() > 20);
        assert!(fb.contains_pixel(50, 30));
    }

    #[test]
    fn test_draw_triangle_general() {
        let mut fb = MockFramebuffer::new();
        let vertices = [
            Point2::new(50, 10),
            Point2::new(30, 30),
            Point2::new(80, 40),
        ];
        let color = Rgb565::CSS_MAGENTA;

        draw(DrawPrimitive::ColoredTriangle(vertices, color), &mut fb);

        // Should draw many pixels for the filled triangle
        assert!(fb.pixel_count() > 30);
    }

    #[test]
    fn test_triangle_vertex_sorting() {
        let mut fb = MockFramebuffer::new();
        // Vertices in reverse y order
        let vertices = [
            Point2::new(50, 30),  // Bottom (will be sorted to top)
            Point2::new(30, 10),  // Top
            Point2::new(70, 20),  // Middle
        ];
        let color = Rgb565::CSS_WHITE;

        // Should not panic and should draw the triangle correctly
        draw(DrawPrimitive::ColoredTriangle(vertices, color), &mut fb);

        assert!(fb.pixel_count() > 10);
    }

    #[test]
    fn test_draw_multiple_primitives() {
        let mut fb = MockFramebuffer::new();

        draw(DrawPrimitive::ColoredPoint(Point2::new(5, 5), Rgb565::CSS_RED), &mut fb);
        draw(DrawPrimitive::Line([Point2::new(10, 10), Point2::new(20, 20)], Rgb565::CSS_GREEN), &mut fb);

        // Should have pixels from both primitives
        assert!(fb.pixel_count() > 11); // 1 point + at least 10 from line
        assert!(fb.contains_pixel(5, 5));
    }
}
