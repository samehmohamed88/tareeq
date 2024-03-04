use std::arch::x86_64::_fxrstor64;
use nalgebra::{DVector, Matrix4, Matrix2, Matrix4x2, Matrix2x4, Matrix4x1, Matrix2x1};
use std::f64::consts::PI;
use std::os::raw::c_double;

static DT: f64 = 0.1;

fn deg2rad(deg: f64) -> f64 {
    deg * PI / 180.0
}

fn calc_input() -> Matrix2x1<f64> {
    let v = 1.0; // [m/s]
    let yaw_rate = 0.1; // [rad/s]
    // This creates a 2-row, 1-column matrix with the values of v and yaw_rate.
    Matrix2x1::from_column_slice(&[v, yaw_rate])
}

fn observation(
    x_true: &Matrix4x1<f64>,
    xd: &Matrix4x1<f64>,
    u: &Matrix2x1<f64>,
) {
    let x_true = motion_model(x_true, u);
    // let z = observation_model(&x_true);
}

fn motion_model(
    x: &Matrix4x1<f64>,
    u: &Matrix2x1<f64>,
) -> Matrix4x1<f64> {
    let f = Matrix4::new(
        1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    );

    let b: Matrix4x2<f64> = Matrix4x2::new(
        DT * x[(2, 0)].cos(),
        0.0,
        DT * x[(2, 0)].sin(),
        0.0,
        0.0,
        DT,
        1.0,
        0.0,
    );

    f * x + b * u
}

fn observation_model(x : &Matrix4x1<f64>) -> Matrix2x1<f64>  {
    let h :Matrix2x4<f64> = Matrix2x4::new(
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0
    );

    x * h
}

// fn observation_model(x: &DVector<f64>) -> DVector<f64> {
//     DVector::from_vec(vec![x[0], x[1]]) // only observe x and y
// }
//
// fn jacob_f(x: &DVector<f64>, u: &DVector<f64>) -> DMatrix<f64> {
//     let dt = 0.1; // time tick [s]
//     let v = u[0];
//     let yaw = x[2];
//     DMatrix::from_row_slice(4, 4, &[
//         1.0, 0.0, -dt * v * yaw.sin(), dt * yaw.cos(),
//         0.0, 1.0, dt * v * yaw.cos(), dt * yaw.sin(),
//         0.0, 0.0, 1.0, 0.0,
//         0.0, 0.0, 0.0, 1.0,
//     ])
// }
//
// fn jacob_h() -> DMatrix<f64> {
//     DMatrix::from_row_slice(2, 4, &[
//         1.0, 0.0, 0.0, 0.0,
//         0.0, 1.0, 0.0, 0.0,
//     ])
// }
//
// fn ekf_estimation(x_est: &DVector<f64>, p_est: &DMatrix<f64>, z: &DVector<f64>, u: &DVector<f64>, q: &DMatrix<f64>, r: &DMatrix<f64>) -> (DVector<f64>, DMatrix<f64>) {
//     // Predict
//     let x_pred = motion_model(x_est, u);
//     let j_f = jacob_f(x_est, u);
//     let p_pred = &j_f * p_est * j_f.transpose() + q;
//
//     // Update
//     let j_h = jacob_h();
//     let z_pred = observation_model(&x_pred);
//     let y = z - &z_pred;
//     let s = &j_h * &p_pred * j_h.transpose() + r;
//     let k = p_pred * j_h.transpose() * s.try_inverse().unwrap();
//     let x_est = &x_pred + &k * y;
//     let p_est = (DMatrix::identity(4, 4) - &k * j_h) * p_pred;
//     (x_est, p_est)
// }

fn main() {
    // Define covariance matrices
    let q_matrix = Matrix4::from_diagonal(&DVector::from_vec(vec![0.1, 0.1, deg2rad(1.0), 1.0]));
    // Square each element individually
    let q = q_matrix.map(|elem: f64| elem.powi(2));
    println!("Q: {}", q);

    let r_matrix = Matrix4::from_diagonal(&DVector::from_vec(vec![1.0, 1.0]));
    let r = r_matrix.map(|elem: f64| elem.powi(2));
    println!("R: {}", r);

    let input_noise_matrix = Matrix2::from_diagonal(&DVector::from_vec(vec![1.0, deg2rad(30.0)]));
    let input_noise = input_noise_matrix.map(|elem: f64| elem.powi(2));

    let gps_noise_matrix = Matrix2::from_diagonal(&DVector::from_vec(vec![0.5, 0.5]));
    let gps_noise = gps_noise_matrix.map(|elem: f64| elem.powi(2));

    let sim_time = 50.0;

    let mut time = 0.0;

    let mut x_estimate = Matrix4x1::<f64>::zeros();
    let mut x_true= Matrix4x1::<f64>::zeros();
    let mut p_estimate = Matrix4x1::<f64>::identity();
    // println!("P estimate : {}", p_estimate)

    let x_dead_reckoning = Matrix4x1::<f64>::zeros();

    // history
    let mut hx_estimate = &x_estimate;
    let mut hx_true = &x_true;
    let mut hx_dead_reckoning = &x_true;
    let hz = Matrix2x1::zeros();

    while (sim_time >= time) {
        time += DT;
        let u = calc_input();
    }

    // Initial state
    // let mut x_est = DVector::from_vec(vec![0.0, 0.0, 0.0, 0.0]); // State vector [x y yaw v]'
    // let mut p_est: Matrix<f64, Dynamic, Dynamic, VecStorage<f64, Dynamic, Dynamic>> = DMatrix::identity(4, 4); // State covariance matrix
    //
    // // Simulate sensor measurements and EKF updates
    // for _ in 0..500 { // 50 seconds with dt = 0.1
    //     let u = calc_input();
    //     let z = observation_model(&motion_model(&x_est, &u)); // Simulate observation
    //
    //     let (new_x_est, new_p_est) = ekf_estimation(&x_est, &p_est, &z, &u, &q, &r);
    //     x_est = new_x_est;
    //     p_est = new_p_est;
    //
    //     // Here you would insert plotting or logging of the state
    //     println!("State: {:?}", x_est);
    // }
}
