//! Geometry helpers shared by the traversal.

#[allow(unused_imports)]
use embedded_graphics_core::pixelcolor::{Rgb565, RgbColor};
#[allow(unused_imports)]
use nalgebra::{Matrix4, Point3, Vector3, Vector4};

#[allow(unused_imports)]
use micromath::F32Ext;

#[allow(unused_imports)]
use crate::pipeline::vertex::transform::{transform_point, transform_point_with_w};

#[cfg(feature = "lighting")]
#[inline]
pub(crate) fn face_world_center(
    face: &[usize; 3],
    vertices: &[[f32; 3]],
    model_matrix: Matrix4<f32>,
) -> Point3<f32> {
    let v0 = vertices[face[0]];
    let v1 = vertices[face[1]];
    let v2 = vertices[face[2]];
    let cx = (v0[0] + v1[0] + v2[0]) / 3.0;
    let cy = (v0[1] + v1[1] + v2[1]) / 3.0;
    let cz = (v0[2] + v1[2] + v2[2]) / 3.0;
    model_matrix.transform_point(&Point3::new(cx, cy, cz))
}
