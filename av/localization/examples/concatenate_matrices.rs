use nalgebra::{DMatrix, Dynamic, Matrix};

fn hstack<T: nalgebra::Scalar + Copy>(matrices: &[DMatrix<T>]) -> Option<DMatrix<T>> {
    if matrices.is_empty() {
        return None;
    }

    let num_rows = matrices[0].nrows();
    let total_cols = matrices.iter().map(|m| m.ncols()).sum();

    if !matrices.iter().all(|m| m.nrows() == num_rows) {
        return None;
    }

    let first_matrix = &matrices[0];
    let mut result = DMatrix::repeat(first_matrix.nrows(), total_cols, first_matrix[(0, 0)]);
    let mut col_offset = 0;

    for matrix in matrices {
        result.slice_mut((0, col_offset), (num_rows, matrix.ncols())).copy_from(matrix);
        col_offset += matrix.ncols();
    }

    Some(result)
}

fn main() {
    let a = DMatrix::<f64>::from_vec(2, 2, vec![1.0, 2.0, 3.0, 4.0]);
    let b = DMatrix::<f64>::from_vec(2, 2, vec![7.0, 8.0, 9.0, 10.0]);

    let c = hstack(&[a, b]).unwrap();

    println!("{}", c);
}