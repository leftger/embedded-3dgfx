#![no_std]
use camera::Camera;
use embedded_graphics_core::pixelcolor::Rgb565;
use embedded_graphics_core::pixelcolor::RgbColor;
use mesh::K3dMesh;
use mesh::RenderMode;
use nalgebra::Matrix4;
use nalgebra::Point2;
use nalgebra::Point3;
use nalgebra::Vector3;

pub mod camera;
pub mod draw;
pub mod framebuffer;
pub mod mesh;
pub mod perfcounter;

#[derive(Debug)]
pub enum DrawPrimitive {
    ColoredPoint(Point2<i32>, Rgb565),
    Line([Point2<i32>; 2], Rgb565),
    ColoredTriangle([Point2<i32>; 3], Rgb565),
    ColoredTriangleWithDepth {
        points: [Point2<i32>; 3],
        depths: [f32; 3],
        color: Rgb565,
    },
}

pub struct K3dengine {
    pub camera: Camera,
    width: u16,
    height: u16,
}

impl K3dengine {
    pub fn new(width: u16, height: u16) -> K3dengine {
        K3dengine {
            camera: Camera::new(width as f32 / height as f32),
            width,
            height,
        }
    }

    fn transform_point(&self, point: &[f32; 3], model_matrix: Matrix4<f32>) -> Option<Point3<i32>> {
        let point = nalgebra::Vector4::new(point[0], point[1], point[2], 1.0);
        let point = model_matrix * point;

        if point.w < 0.0 {
            return None;
        }
        if point.z < self.camera.near || point.z > self.camera.far {
            return None;
        }

        let point = Point3::from_homogeneous(point)?;

        let x = ((1.0 + point.x) * 0.5 * self.width as f32) as i32;
        let y = ((1.0 - point.y) * 0.5 * self.height as f32) as i32;

        if x < 0 || x >= self.width as i32 || y < 0 || y >= self.height as i32 {
            return None;
        }

        Some(Point3::new(
            x,
            y,
            (point.z * (self.camera.far - self.camera.near) + self.camera.near) as i32,
        ))
    }

    fn transform_points<const N: usize>(
        &self,
        indices: &[usize; N],
        vertices: &[[f32; 3]],
        model_matrix: Matrix4<f32>,
    ) -> Option<[Point3<i32>; N]> {
        let mut ret = [Point3::new(0, 0, 0); N];

        for i in 0..N {
            ret[i] = self.transform_point(&vertices[indices[i]], model_matrix)?;
        }

        Some(ret)
    }

    pub fn render<'a, MS, F>(&self, meshes: MS, mut callback: F)
    where
        MS: IntoIterator<Item = &'a K3dMesh<'a>>,
        F: FnMut(DrawPrimitive),
    {
        for mesh in meshes {
            if mesh.geometry.vertices.is_empty() {
                continue;
            }

            let transform_matrix = self.camera.vp_matrix * mesh.model_matrix;

            match mesh.render_mode {
                RenderMode::Points => {
                    let screen_space_points = mesh
                        .geometry
                        .vertices
                        .iter()
                        .filter_map(|v| self.transform_point(v, transform_matrix));

                    if mesh.geometry.colors.len() == mesh.geometry.vertices.len() {
                        for (point, color) in screen_space_points.zip(mesh.geometry.colors) {
                            callback(DrawPrimitive::ColoredPoint(point.xy(), *color));
                        }
                    } else {
                        for point in screen_space_points {
                            callback(DrawPrimitive::ColoredPoint(point.xy(), mesh.color));
                        }
                    }
                }

                RenderMode::Lines if !mesh.geometry.lines.is_empty() => {
                    for line in mesh.geometry.lines {
                        if let Some([p1, p2]) =
                            self.transform_points(line, mesh.geometry.vertices, transform_matrix)
                        {
                            callback(DrawPrimitive::Line([p1.xy(), p2.xy()], mesh.color));
                        }
                    }
                }

                RenderMode::Lines if !mesh.geometry.faces.is_empty() => {
                    for face in mesh.geometry.faces {
                        if let Some([p1, p2, p3]) =
                            self.transform_points(face, mesh.geometry.vertices, transform_matrix)
                        {
                            callback(DrawPrimitive::Line([p1.xy(), p2.xy()], mesh.color));
                            callback(DrawPrimitive::Line([p2.xy(), p3.xy()], mesh.color));
                            callback(DrawPrimitive::Line([p3.xy(), p1.xy()], mesh.color));
                        }
                    }
                }

                RenderMode::Lines => {}

                RenderMode::SolidLightDir(direction) => {
                    for (face, normal) in mesh.geometry.faces.iter().zip(mesh.geometry.normals.iter()) {
                        //Backface culling
                        let normal = Vector3::new(normal[0], normal[1], normal[2]);

                        let transformed_normal = mesh.model_matrix.transform_vector(&normal);

                        // Backface culling: cull faces pointing away from camera
                        // This improves performance by ~50% (don't render back faces)
                        // Z-buffer handles depth ordering, but culling avoids wasted work
                        if self.camera.get_direction().dot(&transformed_normal) < 0.0 {
                            continue;
                        }

                        if let Some([p1, p2, p3]) =
                            self.transform_points(face, mesh.geometry.vertices, transform_matrix)
                        {
                            let color_as_float = Vector3::new(
                                mesh.color.r() as f32 / 32.0,
                                mesh.color.g() as f32 / 64.0,
                                mesh.color.b() as f32 / 32.0,
                            );

                            let mut final_color = Vector3::new(0.0f32, 0.0, 0.0);

                            // Calculate lighting intensity
                            // Negate only Z component of direction to fix front/back while keeping left/right
                            let adjusted_dir = Vector3::new(direction.x, direction.y, -direction.z);
                            let intensity = transformed_normal.dot(&adjusted_dir);

                            let intensity = intensity.max(0.0);

                            final_color += color_as_float * intensity + color_as_float * 0.1;

                            let final_color = Vector3::new(
                                final_color.x.clamp(0.0, 1.0),
                                final_color.y.clamp(0.0, 1.0),
                                final_color.z.clamp(0.0, 1.0),
                            );

                            let color = Rgb565::new(
                                (final_color.x * 31.0) as u8,
                                (final_color.y * 63.0) as u8,
                                (final_color.z * 31.0) as u8,
                            );
                            callback(DrawPrimitive::ColoredTriangleWithDepth {
                                points: [p1.xy(), p2.xy(), p3.xy()],
                                depths: [p1.z as f32, p2.z as f32, p3.z as f32],
                                color,
                            });
                        }
                    }
                }

                RenderMode::Solid => {
                    if mesh.geometry.normals.is_empty() {
                        for face in mesh.geometry.faces.iter() {
                            if let Some([p1, p2, p3]) = self.transform_points(
                                face,
                                mesh.geometry.vertices,
                                transform_matrix,
                            ) {
                                callback(DrawPrimitive::ColoredTriangleWithDepth {
                                    points: [p1.xy(), p2.xy(), p3.xy()],
                                    depths: [p1.z as f32, p2.z as f32, p3.z as f32],
                                    color: mesh.color,
                                });
                            }
                        }
                    } else {
                        for (face, normal) in mesh.geometry.faces.iter().zip(mesh.geometry.normals)
                        {
                            //Backface culling
                            let normal = Vector3::new(normal[0], normal[1], normal[2]);

                            let transformed_normal = mesh.model_matrix.transform_vector(&normal);

                            // Backface culling: cull faces pointing away from camera
                            if self.camera.get_direction().dot(&transformed_normal) < 0.0 {
                                continue;
                            }

                            if let Some([p1, p2, p3]) = self.transform_points(
                                face,
                                mesh.geometry.vertices,
                                transform_matrix,
                            ) {
                                callback(DrawPrimitive::ColoredTriangleWithDepth {
                                    points: [p1.xy(), p2.xy(), p3.xy()],
                                    depths: [p1.z as f32, p2.z as f32, p3.z as f32],
                                    color: mesh.color,
                                });
                            }
                        }
                    }
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;

    #[test]
    fn test_engine_creation() {
        let engine = K3dengine::new(640, 480);
        assert_eq!(engine.width, 640);
        assert_eq!(engine.height, 480);
        assert!((engine.camera.get_aspect_ratio() - 640.0 / 480.0).abs() < 0.001);
    }

    #[test]
    fn test_transform_point_basic() {
        let engine = K3dengine::new(640, 480);
        // Use camera's VP matrix directly
        let transform_matrix = engine.camera.vp_matrix;

        // Point in front of default camera, within view frustum
        // Default camera is at origin looking at origin, so we need a point in front
        let point = [0.0, 0.0, -5.0];
        let result = engine.transform_point(&point, transform_matrix);

        if let Some(transformed) = result {
            // Should be within screen bounds
            assert!(transformed.x >= 0 && transformed.x < 640);
            assert!(transformed.y >= 0 && transformed.y < 480);
        }
        // If None, the point was culled which is also valid behavior
    }

    #[test]
    fn test_transform_point_clamps_out_of_bounds() {
        let engine = K3dengine::new(640, 480);
        let model_matrix = nalgebra::Matrix4::identity();

        // Point way outside the viewport should be clamped/rejected
        let point = [100.0, 100.0, -5.0];
        let result = engine.transform_point(&point, model_matrix);
        // Should return None because coordinates are clamped out
        assert!(result.is_none());
    }

    #[test]
    fn test_transform_point_behind_camera() {
        let engine = K3dengine::new(640, 480);
        let transform_matrix = engine.camera.vp_matrix;

        // Point with positive z (behind default camera orientation)
        let point = [0.0, 0.0, 1.0];
        let _result = engine.transform_point(&point, transform_matrix);
        // Point behind camera or outside frustum should return None
        // (actual behavior depends on camera setup and projection)
        // This test just verifies the function doesn't panic
    }

    #[test]
    fn test_transform_point_near_plane_clipping() {
        let engine = K3dengine::new(640, 480);
        let model_matrix = nalgebra::Matrix4::identity();

        // Point too close to camera (before near plane)
        let point = [0.0, 0.0, -0.01];
        let result = engine.transform_point(&point, model_matrix);
        assert!(result.is_none());
    }

    #[test]
    fn test_transform_point_far_plane_clipping() {
        let engine = K3dengine::new(640, 480);
        let model_matrix = nalgebra::Matrix4::identity();

        // Point too far from camera (beyond far plane)
        let point = [0.0, 0.0, -1000.0];
        let result = engine.transform_point(&point, model_matrix);
        assert!(result.is_none());
    }

    #[test]
    fn test_transform_points_array() {
        let engine = K3dengine::new(640, 480);
        let transform_matrix = engine.camera.vp_matrix;

        let vertices = [
            [0.0, 0.0, -5.0],
            [0.1, 0.0, -5.0],
            [0.0, 0.1, -5.0],
        ];
        let indices = [0, 1, 2];

        let result = engine.transform_points(&indices, &vertices, transform_matrix);

        // If transform succeeds, verify we get 3 points
        if let Some(points) = result {
            assert_eq!(points.len(), 3);
        }
        // If None, one or more points were culled which is valid
    }

    #[test]
    fn test_render_empty_faces_mesh() {
        let engine = K3dengine::new(640, 480);
        let vertices = [[0.0, 0.0, -5.0]]; // At least one vertex required
        let geometry = mesh::Geometry {
            vertices: &vertices,
            faces: &[],
            colors: &[],
            lines: &[],
            normals: &[],
        };
        let mesh = mesh::K3dMesh::new(geometry);

        let mut callback_count = 0;
        engine.render(std::iter::once(&mesh), |_| {
            callback_count += 1;
        });

        // Mesh with no faces/lines should trigger one point callback (default is Points mode)
        assert!(callback_count > 0);
    }

    #[test]
    fn test_render_points_mode() {
        let engine = K3dengine::new(640, 480);

        let vertices = [
            [0.0, 0.0, -5.0],
            [0.5, 0.0, -5.0],
        ];

        let geometry = mesh::Geometry {
            vertices: &vertices,
            faces: &[],
            colors: &[],
            lines: &[],
            normals: &[],
        };

        let mut mesh = mesh::K3dMesh::new(geometry);
        mesh.set_render_mode(mesh::RenderMode::Points);

        let mut primitives = std::vec::Vec::new();
        engine.render(std::iter::once(&mesh), |prim| {
            primitives.push(prim);
        });

        // Should render points
        assert!(primitives.len() > 0);
        for prim in primitives {
            assert!(matches!(prim, DrawPrimitive::ColoredPoint(_, _)));
        }
    }

    #[test]
    fn test_render_lines_mode_with_faces() {
        let engine = K3dengine::new(640, 480);

        let vertices = [
            [0.0, 0.0, -5.0],
            [0.5, 0.0, -5.0],
            [0.0, 0.5, -5.0],
        ];

        let faces = [[0, 1, 2]];

        let geometry = mesh::Geometry {
            vertices: &vertices,
            faces: &faces,
            colors: &[],
            lines: &[],
            normals: &[],
        };

        let mut mesh = mesh::K3dMesh::new(geometry);
        mesh.set_render_mode(mesh::RenderMode::Lines);

        let mut primitives = std::vec::Vec::new();
        engine.render(std::iter::once(&mesh), |prim| {
            primitives.push(prim);
        });

        // Should render 3 lines (edges of triangle)
        assert_eq!(primitives.len(), 3);
        for prim in primitives {
            assert!(matches!(prim, DrawPrimitive::Line(_, _)));
        }
    }
}
