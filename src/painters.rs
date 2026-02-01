//! Painter's Algorithm Implementation
//!
//! Renders triangles sorted by depth (back-to-front) without a Z-buffer.
//! This trades sorting overhead for significant memory savings (no Z-buffer needed).
//!
//! Benefits:
//! - Saves ~1.92MB RAM for 800x600 resolution (u32 Z-buffer)
//! - Good for simple scenes with few overlapping triangles
//! - O(n log n) sorting cost, acceptable when n is small
//!
//! Limitations:
//! - Doesn't handle cyclic overlaps perfectly
//! - Sorting cost increases with triangle count
//! - Best for scenes with ~1000-5000 triangles
//!
//! Note: This module is only available when building with std (examples, tests)
//! as it requires Vec for dynamic triangle collection.

// This module requires std::vec::Vec for dynamic triangle collection
// It's designed for use in std environments (examples, tests, PC builds)
extern crate std;

use crate::mesh::{K3dMesh, RenderMode};
use crate::{DrawPrimitive, K3dengine};
use embedded_graphics_core::pixelcolor::{Rgb565, RgbColor};
use std::cmp::Ordering;
use std::vec::Vec;

/// A triangle with its average depth for sorting
#[derive(Debug, Clone)]
pub struct DepthSortedTriangle {
    pub primitive: DrawPrimitive,
    pub avg_depth: f32,
}

impl DepthSortedTriangle {
    /// Create a new depth-sorted triangle
    pub fn new(primitive: DrawPrimitive, avg_depth: f32) -> Self {
        Self {
            primitive,
            avg_depth,
        }
    }
}

impl K3dengine {
    /// Render using Painter's Algorithm (back-to-front sorting, no Z-buffer)
    ///
    /// Collects all triangles, sorts them by depth, and renders back-to-front.
    /// This eliminates the need for a Z-buffer, saving significant memory.
    ///
    /// # Arguments
    /// * `meshes` - Iterator of meshes to render
    /// * `triangles` - Buffer to store sorted triangles (must be large enough!)
    /// * `callback` - Drawing callback for each primitive
    ///
    /// # Returns
    /// Number of triangles rendered
    pub fn render_painters_algorithm<'a, MS, F>(
        &self,
        meshes: MS,
        triangles: &mut Vec<DepthSortedTriangle>,
        mut callback: F,
    ) -> usize
    where
        MS: IntoIterator<Item = &'a K3dMesh<'a>>,
        F: FnMut(DrawPrimitive),
    {
        triangles.clear();

        // Collect all triangles with their depths
        for mesh in meshes {
            if mesh.geometry.vertices.is_empty() {
                continue;
            }

            // Frustum culling
            if self.should_cull_mesh(mesh) {
                continue;
            }

            // LOD Selection
            let mesh_pos = mesh.get_position();
            let distance = (mesh_pos - self.camera.position).norm();
            let geometry = mesh.select_lod(distance);

            let transform_matrix = self.camera.vp_matrix * mesh.model_matrix;

            // Only collect renderable triangles (Solid modes)
            match mesh.render_mode {
                RenderMode::Solid
                | RenderMode::SolidLightDir(_)
                | RenderMode::BlinnPhong { .. } => {
                    for face in geometry.faces {
                        if let Some([p1, p2, p3]) =
                            self.transform_points(face, geometry.vertices, transform_matrix)
                        {
                            // Backface culling
                            let v1 = (p2.x - p1.x, p2.y - p1.y);
                            let v2 = (p3.x - p1.x, p3.y - p1.y);
                            let cross = v1.0 * v2.1 - v1.1 * v2.0;
                            if cross <= 0 {
                                continue;
                            }

                            // Calculate average depth for sorting
                            let avg_depth = (p1.z + p2.z + p3.z) as f32 / 3.0;

                            // Determine color based on render mode
                            let color = match mesh.render_mode {
                                RenderMode::SolidLightDir(light_dir) => self.calculate_lit_color(
                                    face,
                                    geometry.vertices,
                                    geometry.normals,
                                    mesh.color,
                                    light_dir,
                                ),
                                RenderMode::BlinnPhong { .. } => {
                                    // For Painter's Algorithm, fall back to simple lighting
                                    // Full Blinn-Phong requires per-pixel depth
                                    mesh.color
                                }
                                _ => mesh.color,
                            };

                            let primitive =
                                DrawPrimitive::ColoredTriangle([p1.xy(), p2.xy(), p3.xy()], color);

                            triangles.push(DepthSortedTriangle::new(primitive, avg_depth));
                        }
                    }
                }
                // Lines and Points don't need depth sorting
                _ => {}
            }
        }

        // Sort triangles by depth (back-to-front = largest depth first)
        triangles.sort_by(|a, b| {
            b.avg_depth
                .partial_cmp(&a.avg_depth)
                .unwrap_or(Ordering::Equal)
        });

        // Render sorted triangles
        let count = triangles.len();
        for triangle in triangles.iter() {
            callback(triangle.primitive.clone());
        }

        count
    }

    /// Helper to calculate lit color for directional lighting
    #[allow(dead_code)]
    fn calculate_lit_color(
        &self,
        face: &[usize; 3],
        vertices: &[[f32; 3]],
        normals: &[[f32; 3]],
        base_color: Rgb565,
        light_dir: nalgebra::Vector3<f32>,
    ) -> Rgb565 {
        // Calculate face normal if not provided
        let normal = if !normals.is_empty() && face[0] < normals.len() {
            nalgebra::Vector3::new(
                normals[face[0]][0],
                normals[face[0]][1],
                normals[face[0]][2],
            )
        } else {
            // Compute face normal from vertices
            let v0 = &vertices[face[0]];
            let v1 = &vertices[face[1]];
            let v2 = &vertices[face[2]];

            let edge1 = nalgebra::Vector3::new(v1[0] - v0[0], v1[1] - v0[1], v1[2] - v0[2]);
            let edge2 = nalgebra::Vector3::new(v2[0] - v0[0], v2[1] - v0[1], v2[2] - v0[2]);

            let normal = edge1.cross(&edge2);
            if normal.norm() > 0.0 {
                normal.normalize()
            } else {
                nalgebra::Vector3::new(0.0, 1.0, 0.0)
            }
        };

        // Simple diffuse lighting
        let light_intensity = normal.dot(&light_dir.normalize()).max(0.0);
        let ambient = 0.3;
        let final_intensity = (ambient + (1.0 - ambient) * light_intensity).clamp(0.0, 1.0);

        // Apply lighting to color
        let r = (base_color.r() as f32 * final_intensity) as u8;
        let g = (base_color.g() as f32 * final_intensity) as u8;
        let b = (base_color.b() as f32 * final_intensity) as u8;

        Rgb565::new(r, g, b)
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use embedded_graphics_core::pixelcolor::Rgb565;
    use std::cmp::Ordering;

    #[test]
    fn test_sorting_by_depth() {
        let mut triangles = std::vec![
            DepthSortedTriangle {
                primitive: DrawPrimitive::Line(
                    [nalgebra::Point2::new(0, 0), nalgebra::Point2::new(1, 1)],
                    Rgb565::new(31, 0, 0),
                ),
                avg_depth: 10.0,
            },
            DepthSortedTriangle {
                primitive: DrawPrimitive::Line(
                    [nalgebra::Point2::new(0, 0), nalgebra::Point2::new(1, 1)],
                    Rgb565::new(0, 63, 0),
                ),
                avg_depth: 5.0,
            },
            DepthSortedTriangle {
                primitive: DrawPrimitive::Line(
                    [nalgebra::Point2::new(0, 0), nalgebra::Point2::new(1, 1)],
                    Rgb565::new(0, 0, 31),
                ),
                avg_depth: 15.0,
            },
        ];

        triangles.sort_by(|a, b| {
            b.avg_depth
                .partial_cmp(&a.avg_depth)
                .unwrap_or(Ordering::Equal)
        });

        // Should be sorted furthest to nearest (15, 10, 5)
        assert_eq!(triangles[0].avg_depth, 15.0);
        assert_eq!(triangles[1].avg_depth, 10.0);
        assert_eq!(triangles[2].avg_depth, 5.0);
    }
}
