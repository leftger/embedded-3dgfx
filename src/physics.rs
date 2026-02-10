//! Physics engine foundation for embedded 3D graphics.
//!
//! Provides rigid body dynamics with linear motion (position, velocity, acceleration),
//! gravity, force accumulation, and semi-implicit Euler integration.
//!
//! Designed for `no_std` environments using fixed-capacity `heapless` collections.
//!
//! # Example
//! ```
//! use embedded_3dgfx::physics::{PhysicsWorld, RigidBody, BodyType};
//! use nalgebra::Vector3;
//!
//! let mut world = PhysicsWorld::<16>::new();
//! world.set_gravity(Vector3::new(0.0, -9.81, 0.0));
//!
//! let body = RigidBody::new(1.0)
//!     .with_position(Vector3::new(0.0, 10.0, 0.0));
//! let id = world.add_body(body).unwrap();
//!
//! // Advance simulation by 16ms
//! world.step(0.016);
//!
//! let pos = world.body(id).unwrap().position;
//! // Body has fallen due to gravity
//! assert!(pos.y < 10.0);
//! ```

use nalgebra::Vector3;

// ComplexField provides sqrt() for f32 in no_std via libm
#[allow(unused_imports)]
use nalgebra::ComplexField;

/// Unique identifier for a rigid body within a [`PhysicsWorld`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BodyId(usize);

/// Determines how a body participates in the simulation.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum BodyType {
    /// Fully simulated: affected by forces, gravity, and velocity.
    Dynamic,
    /// Not affected by forces or gravity. Useful for floors, walls, and platforms.
    /// Can still be repositioned manually.
    Static,
}

/// A rigid body with linear dynamics.
///
/// Tracks position, velocity, accumulated forces, and mass.
/// Angular dynamics (rotation, torque) are planned for a future phase.
#[derive(Debug, Clone)]
pub struct RigidBody {
    pub position: Vector3<f32>,
    pub velocity: Vector3<f32>,
    pub mass: f32,
    pub inv_mass: f32,
    pub body_type: BodyType,
    pub restitution: f32,

    /// Accumulated forces applied this frame. Cleared after each `step()`.
    force_accumulator: Vector3<f32>,

    /// Linear damping factor (0.0 = no damping, 1.0 = full stop).
    /// Applied each step as `velocity *= (1.0 - damping)`.
    pub damping: f32,
}

impl RigidBody {
    /// Create a new dynamic rigid body with the given mass (in kg).
    ///
    /// # Panics
    /// Panics if `mass` is not positive and finite.
    pub fn new(mass: f32) -> Self {
        assert!(mass > 0.0 && mass.is_finite(), "mass must be positive and finite");
        Self {
            position: Vector3::zeros(),
            velocity: Vector3::zeros(),
            mass,
            inv_mass: 1.0 / mass,
            body_type: BodyType::Dynamic,
            restitution: 0.5,
            force_accumulator: Vector3::zeros(),
            damping: 0.01,
        }
    }

    /// Create a new static rigid body (infinite mass, unaffected by forces).
    pub fn new_static() -> Self {
        Self {
            position: Vector3::zeros(),
            velocity: Vector3::zeros(),
            mass: f32::INFINITY,
            inv_mass: 0.0,
            body_type: BodyType::Static,
            restitution: 0.5,
            force_accumulator: Vector3::zeros(),
            damping: 0.0,
        }
    }

    /// Builder: set initial position.
    pub fn with_position(mut self, position: Vector3<f32>) -> Self {
        self.position = position;
        self
    }

    /// Builder: set initial velocity.
    pub fn with_velocity(mut self, velocity: Vector3<f32>) -> Self {
        self.velocity = velocity;
        self
    }

    /// Builder: set restitution (bounciness, 0.0..=1.0).
    pub fn with_restitution(mut self, restitution: f32) -> Self {
        self.restitution = restitution.clamp(0.0, 1.0);
        self
    }

    /// Builder: set linear damping (0.0..=1.0).
    pub fn with_damping(mut self, damping: f32) -> Self {
        self.damping = damping.clamp(0.0, 1.0);
        self
    }

    /// Apply a force (in Newtons) to this body. Forces accumulate until the next `step()`.
    #[inline]
    pub fn apply_force(&mut self, force: Vector3<f32>) {
        self.force_accumulator += force;
    }

    /// Apply an instantaneous impulse (change in momentum) to this body.
    /// Directly modifies velocity: `delta_v = impulse / mass`.
    #[inline]
    pub fn apply_impulse(&mut self, impulse: Vector3<f32>) {
        if self.body_type == BodyType::Dynamic {
            self.velocity += impulse * self.inv_mass;
        }
    }

    /// Returns the current speed (magnitude of velocity).
    #[inline]
    pub fn speed(&self) -> f32 {
        self.velocity.norm()
    }

    /// Returns the kinetic energy of this body: `0.5 * m * v^2`.
    #[inline]
    pub fn kinetic_energy(&self) -> f32 {
        0.5 * self.mass * self.velocity.norm_squared()
    }

    /// Integrate this body forward by `dt` seconds using semi-implicit Euler.
    ///
    /// Semi-implicit Euler updates velocity first, then position, which provides
    /// better energy conservation than explicit (forward) Euler.
    fn integrate(&mut self, dt: f32, gravity: Vector3<f32>) {
        if self.body_type != BodyType::Dynamic {
            self.force_accumulator = Vector3::zeros();
            return;
        }

        // acceleration = gravity + (accumulated forces / mass)
        let acceleration = gravity + self.force_accumulator * self.inv_mass;

        // Semi-implicit Euler: update velocity first, then position
        self.velocity += acceleration * dt;

        // Apply damping
        self.velocity *= 1.0 - self.damping;

        self.position += self.velocity * dt;

        // Clear accumulated forces for next frame
        self.force_accumulator = Vector3::zeros();
    }
}

/// The physics simulation world.
///
/// Manages a fixed-capacity set of rigid bodies and steps the simulation forward.
///
/// # Type Parameters
/// * `N` - Maximum number of bodies (compile-time capacity for `heapless::Vec`).
///
/// # Example
/// ```
/// use embedded_3dgfx::physics::{PhysicsWorld, RigidBody};
/// use nalgebra::Vector3;
///
/// let mut world = PhysicsWorld::<8>::new();
/// world.set_gravity(Vector3::new(0.0, -9.81, 0.0));
///
/// let body = RigidBody::new(2.0)
///     .with_position(Vector3::new(0.0, 5.0, 0.0));
/// let id = world.add_body(body).unwrap();
///
/// world.step(1.0 / 60.0);
/// ```
pub struct PhysicsWorld<const N: usize> {
    bodies: heapless::Vec<RigidBody, N>,
    gravity: Vector3<f32>,
}

impl<const N: usize> PhysicsWorld<N> {
    /// Create a new physics world with no gravity.
    pub fn new() -> Self {
        Self {
            bodies: heapless::Vec::new(),
            gravity: Vector3::zeros(),
        }
    }

    /// Set the gravity vector (e.g., `Vector3::new(0.0, -9.81, 0.0)`).
    pub fn set_gravity(&mut self, gravity: Vector3<f32>) {
        self.gravity = gravity;
    }

    /// Returns the current gravity vector.
    pub fn gravity(&self) -> Vector3<f32> {
        self.gravity
    }

    /// Add a body to the world. Returns its [`BodyId`], or `None` if at capacity.
    pub fn add_body(&mut self, body: RigidBody) -> Option<BodyId> {
        let id = BodyId(self.bodies.len());
        self.bodies.push(body).ok()?;
        Some(id)
    }

    /// Get an immutable reference to a body by its ID.
    pub fn body(&self, id: BodyId) -> Option<&RigidBody> {
        self.bodies.get(id.0)
    }

    /// Get a mutable reference to a body by its ID.
    pub fn body_mut(&mut self, id: BodyId) -> Option<&mut RigidBody> {
        self.bodies.get_mut(id.0)
    }

    /// Returns the number of bodies in the world.
    pub fn body_count(&self) -> usize {
        self.bodies.len()
    }

    /// Iterate over all bodies immutably.
    pub fn bodies(&self) -> impl Iterator<Item = (BodyId, &RigidBody)> {
        self.bodies.iter().enumerate().map(|(i, b)| (BodyId(i), b))
    }

    /// Iterate over all bodies mutably.
    pub fn bodies_mut(&mut self) -> impl Iterator<Item = (BodyId, &mut RigidBody)> {
        self.bodies.iter_mut().enumerate().map(|(i, b)| (BodyId(i), b))
    }

    /// Advance the simulation by `dt` seconds.
    ///
    /// Integrates all dynamic bodies using semi-implicit Euler.
    /// Forces accumulated on each body are consumed and cleared.
    pub fn step(&mut self, dt: f32) {
        let gravity = self.gravity;
        for body in self.bodies.iter_mut() {
            body.integrate(dt, gravity);
        }
    }

    /// Advance the simulation using fixed-size substeps for stability.
    ///
    /// Divides `dt` into `substeps` equal intervals. More substeps = more accurate
    /// but more expensive. Useful when `dt` varies (e.g., frame-rate dependent).
    pub fn step_fixed(&mut self, dt: f32, substeps: u32) {
        let sub_dt = dt / substeps as f32;
        for _ in 0..substeps {
            self.step(sub_dt);
        }
    }
}

/// Helper to sync a [`RigidBody`]'s position back to a [`K3dMesh`](crate::mesh::K3dMesh).
///
/// Call this after `PhysicsWorld::step()` to update mesh transforms.
///
/// # Example
/// ```ignore
/// physics_world.step(dt);
/// for (id, body) in physics_world.bodies() {
///     sync_body_to_mesh(body, &mut meshes[id]);
/// }
/// ```
pub fn sync_body_to_mesh(body: &RigidBody, mesh: &mut crate::mesh::K3dMesh<'_>) {
    mesh.set_position(body.position.x, body.position.y, body.position.z);
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;

    const EPSILON: f32 = 1e-4;

    fn approx_eq(a: f32, b: f32) -> bool {
        (a - b).abs() < EPSILON
    }

    fn approx_vec_eq(a: &Vector3<f32>, b: &Vector3<f32>) -> bool {
        approx_eq(a.x, b.x) && approx_eq(a.y, b.y) && approx_eq(a.z, b.z)
    }

    // -- RigidBody tests --

    #[test]
    fn test_body_creation() {
        let body = RigidBody::new(5.0);
        assert_eq!(body.mass, 5.0);
        assert!(approx_eq(body.inv_mass, 0.2));
        assert_eq!(body.body_type, BodyType::Dynamic);
        assert!(approx_vec_eq(&body.position, &Vector3::zeros()));
        assert!(approx_vec_eq(&body.velocity, &Vector3::zeros()));
    }

    #[test]
    fn test_static_body() {
        let body = RigidBody::new_static();
        assert_eq!(body.body_type, BodyType::Static);
        assert_eq!(body.inv_mass, 0.0);
        assert!(body.mass.is_infinite());
    }

    #[test]
    #[should_panic]
    fn test_body_zero_mass_panics() {
        RigidBody::new(0.0);
    }

    #[test]
    #[should_panic]
    fn test_body_negative_mass_panics() {
        RigidBody::new(-1.0);
    }

    #[test]
    fn test_builder_pattern() {
        let body = RigidBody::new(1.0)
            .with_position(Vector3::new(1.0, 2.0, 3.0))
            .with_velocity(Vector3::new(0.0, 5.0, 0.0))
            .with_restitution(0.8)
            .with_damping(0.05);

        assert!(approx_vec_eq(&body.position, &Vector3::new(1.0, 2.0, 3.0)));
        assert!(approx_vec_eq(&body.velocity, &Vector3::new(0.0, 5.0, 0.0)));
        assert!(approx_eq(body.restitution, 0.8));
        assert!(approx_eq(body.damping, 0.05));
    }

    #[test]
    fn test_apply_force() {
        let mut body = RigidBody::new(1.0);
        body.apply_force(Vector3::new(10.0, 0.0, 0.0));
        body.apply_force(Vector3::new(0.0, 5.0, 0.0));

        // Forces accumulate
        assert!(approx_vec_eq(
            &body.force_accumulator,
            &Vector3::new(10.0, 5.0, 0.0)
        ));
    }

    #[test]
    fn test_apply_impulse() {
        let mut body = RigidBody::new(2.0);
        body.apply_impulse(Vector3::new(10.0, 0.0, 0.0));

        // delta_v = impulse / mass = 10 / 2 = 5
        assert!(approx_vec_eq(&body.velocity, &Vector3::new(5.0, 0.0, 0.0)));
    }

    #[test]
    fn test_impulse_on_static_body_ignored() {
        let mut body = RigidBody::new_static();
        body.apply_impulse(Vector3::new(100.0, 0.0, 0.0));
        assert!(approx_vec_eq(&body.velocity, &Vector3::zeros()));
    }

    #[test]
    fn test_speed() {
        let body = RigidBody::new(1.0)
            .with_velocity(Vector3::new(3.0, 4.0, 0.0));
        assert!(approx_eq(body.speed(), 5.0));
    }

    #[test]
    fn test_kinetic_energy() {
        let body = RigidBody::new(2.0)
            .with_velocity(Vector3::new(3.0, 0.0, 0.0));
        // KE = 0.5 * 2 * 9 = 9
        assert!(approx_eq(body.kinetic_energy(), 9.0));
    }

    // -- PhysicsWorld tests --

    #[test]
    fn test_world_creation() {
        let world = PhysicsWorld::<8>::new();
        assert_eq!(world.body_count(), 0);
        assert!(approx_vec_eq(&world.gravity(), &Vector3::zeros()));
    }

    #[test]
    fn test_add_and_get_body() {
        let mut world = PhysicsWorld::<8>::new();
        let body = RigidBody::new(1.0).with_position(Vector3::new(1.0, 2.0, 3.0));
        let id = world.add_body(body).unwrap();

        assert_eq!(world.body_count(), 1);
        let b = world.body(id).unwrap();
        assert!(approx_vec_eq(&b.position, &Vector3::new(1.0, 2.0, 3.0)));
    }

    #[test]
    fn test_add_body_at_capacity() {
        let mut world = PhysicsWorld::<2>::new();
        assert!(world.add_body(RigidBody::new(1.0)).is_some());
        assert!(world.add_body(RigidBody::new(1.0)).is_some());
        assert!(world.add_body(RigidBody::new(1.0)).is_none()); // at capacity
    }

    #[test]
    fn test_gravity_freefall() {
        let mut world = PhysicsWorld::<4>::new();
        world.set_gravity(Vector3::new(0.0, -10.0, 0.0));

        let body = RigidBody::new(1.0)
            .with_position(Vector3::new(0.0, 100.0, 0.0))
            .with_damping(0.0);
        let id = world.add_body(body).unwrap();

        // Step 1 second
        world.step(1.0);

        let b = world.body(id).unwrap();
        // After 1s of freefall at -10 m/s²:
        // v = 0 + (-10)*1 = -10 m/s
        // p = 100 + (-10)*1 = 90 m  (semi-implicit: velocity updated first, then position)
        assert!(approx_eq(b.velocity.y, -10.0));
        assert!(approx_eq(b.position.y, 90.0));
    }

    #[test]
    fn test_static_body_not_affected_by_gravity() {
        let mut world = PhysicsWorld::<4>::new();
        world.set_gravity(Vector3::new(0.0, -9.81, 0.0));

        let body = RigidBody::new_static()
            .with_position(Vector3::new(0.0, 0.0, 0.0));
        let id = world.add_body(body).unwrap();

        world.step(1.0);

        let b = world.body(id).unwrap();
        assert!(approx_vec_eq(&b.position, &Vector3::zeros()));
        assert!(approx_vec_eq(&b.velocity, &Vector3::zeros()));
    }

    #[test]
    fn test_force_accumulation_and_clearing() {
        let mut world = PhysicsWorld::<4>::new();

        let body = RigidBody::new(1.0).with_damping(0.0);
        let id = world.add_body(body).unwrap();

        // Apply a force and step
        world.body_mut(id).unwrap().apply_force(Vector3::new(10.0, 0.0, 0.0));
        world.step(1.0);

        let b = world.body(id).unwrap();
        assert!(approx_eq(b.velocity.x, 10.0));

        // Forces should be cleared — another step with no forces should not accelerate further
        world.step(1.0);
        let b = world.body(id).unwrap();
        assert!(approx_eq(b.velocity.x, 10.0)); // unchanged (no damping, no forces)
    }

    #[test]
    fn test_damping_reduces_velocity() {
        let mut world = PhysicsWorld::<4>::new();

        let body = RigidBody::new(1.0)
            .with_velocity(Vector3::new(10.0, 0.0, 0.0))
            .with_damping(0.1);
        let id = world.add_body(body).unwrap();

        world.step(1.0);

        let b = world.body(id).unwrap();
        // velocity should be reduced by damping
        assert!(b.velocity.x < 10.0);
        assert!(b.velocity.x > 0.0);
    }

    #[test]
    fn test_step_fixed_substeps() {
        let mut world_single = PhysicsWorld::<4>::new();
        let mut world_sub = PhysicsWorld::<4>::new();

        world_single.set_gravity(Vector3::new(0.0, -10.0, 0.0));
        world_sub.set_gravity(Vector3::new(0.0, -10.0, 0.0));

        let body = RigidBody::new(1.0)
            .with_position(Vector3::new(0.0, 100.0, 0.0))
            .with_damping(0.0);

        let id1 = world_single.add_body(body.clone()).unwrap();
        let id2 = world_sub.add_body(body).unwrap();

        // Single large step vs 10 substeps
        world_single.step(1.0);
        world_sub.step_fixed(1.0, 10);

        // Both should give similar results (substeps are more accurate)
        let b1 = world_single.body(id1).unwrap();
        let b2 = world_sub.body(id2).unwrap();

        // Velocities should be the same (gravity is constant)
        assert!(approx_eq(b1.velocity.y, b2.velocity.y));
    }

    #[test]
    fn test_bodies_iterator() {
        let mut world = PhysicsWorld::<4>::new();
        world.add_body(RigidBody::new(1.0).with_position(Vector3::new(1.0, 0.0, 0.0))).unwrap();
        world.add_body(RigidBody::new(2.0).with_position(Vector3::new(2.0, 0.0, 0.0))).unwrap();

        let positions: std::vec::Vec<f32> = world.bodies().map(|(_, b)| b.position.x).collect();
        assert_eq!(positions, std::vec![1.0, 2.0]);
    }

    #[test]
    fn test_projectile_motion() {
        let mut world = PhysicsWorld::<4>::new();
        world.set_gravity(Vector3::new(0.0, -10.0, 0.0));

        // Launch at 45 degrees: vx=10, vy=10
        let body = RigidBody::new(1.0)
            .with_velocity(Vector3::new(10.0, 10.0, 0.0))
            .with_damping(0.0);
        let id = world.add_body(body).unwrap();

        // Step for 2 seconds (at t=2, vy should be 10 - 20 = -10)
        for _ in 0..200 {
            world.step(0.01);
        }

        let b = world.body(id).unwrap();
        // x position: ~20m (10 m/s * 2s)
        assert!((b.position.x - 20.0).abs() < 0.5);
        // y velocity: ~-10 m/s
        assert!((b.velocity.y - (-10.0)).abs() < 0.5);
    }
}
