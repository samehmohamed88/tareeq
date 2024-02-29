use nalgebra::{Vector2, Rotation2};

pub struct Waypoint {
    name: String,
    position: Vector2<f64>,     // Translation (x, y)
    orientation: Rotation2<f64>,// Rotation (angle)
}

impl Waypoint {

    /// Creates a new SE(2) waypoint with the given position and orientation.
    pub fn new(name:String, position: Vector2<f64>, orientation: Rotation2<f64>) -> Self {
        Waypoint { name, position, orientation }
    }

    /// Getter for position.
    pub fn position(&self) -> &Vector2<f64> {
        &self.position
    }

    /// Getter for orientation.
    pub fn orientation(&self) -> &Rotation2<f64> {
        &self.orientation
    }
}
