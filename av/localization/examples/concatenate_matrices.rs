use nalgebra::{Matrix2, Matrix4x2, DMatrix};

fn main() {
    let a = Matrix2::new(1, 2,
                         3, 4);
    let b = Matrix2::new(5, 6,
                         7, 8);
    let c = Matrix2::new(9, 10,
                         11, 12);
    let d = Matrix2::new(13, 14,
                         15, 16);

    let result = DMatrix::from_vec(a.len(), a.len() + b.len(), [a.as_slice(), b.as_slice()].concat());

    println!("Result:\n {}", result);
}
