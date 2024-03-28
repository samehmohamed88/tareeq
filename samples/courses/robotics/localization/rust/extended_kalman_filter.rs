use nalgebra::{DMatrix, DVector};

fn main() {
    // Assuming deg2rad is defined elsewhere
    let radians = deg2rad(1.0);

    // Create a diagonal matrix
    let matrix = DMatrix::from_diagonal(&DVector::from_vec(vec![0.1, 0.1, radians, 1.0]));

    // Square each element individually
    let squared_matrix = matrix.map(|elem| elem.powi(2));

    // Print results
    println!("Squared matrix: \n{}", squared_matrix);
}

// Dummy deg2rad function for completeness
fn deg2rad(deg: f64) -> f64 {
    deg * std::f64::consts::PI / 180.0
}