use nalgebra::{DMatrix, DVector, Dynamic, Matrix, OMatrix, VecStorage};
use std::f64::consts::PI;
use std::os::raw::c_double;

fn deg2rad(deg: f64) -> f64 {
    deg * PI / 180.0
}

fn calc_input() -> DMatrix<f64> {
    let v = 1.0; // [m/s]
    let yaw_rate = 0.1; // [rad/s]
    DMatrix::from_column_slice(2, 1, &[v, yaw_rate])
    // This creates a 2-row, 1-column matrix with the values of v and yaw_rate.
}

fn observation(x_true :OMatrix<f64, Dynamic, Dynamic>, xd :OMatrix<f64, Dynamic, Dynamic>, u :DMatrix<f64>) {

}

fn motion_model(x: &DVector<f64>, u: &DVector<f64>) -> DVector<f64> {
    let dt = 0.1; // time tick [s]
    let mut x = x.clone();
    x[0] += u[0] * dt * x[2].cos(); // x position
    x[1] += u[0] * dt * x[2].sin(); // y position
    x[2] += u[1] * dt; // yaw angle
    x // return new state
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
    let q_matrix = DMatrix::from_diagonal(&DVector::from_vec(vec![0.1, 0.1, deg2rad(1.0), 1.0]));
    // Square each element individually
    let q = q_matrix.map(|elem :f64 | elem.powi(2));
    println!("Q: {}", q);

    let r_matrix = DMatrix::from_diagonal(&DVector::from_vec(vec![1.0, 1.0]));
    let r = r_matrix.map(|elem :f64 | elem.powi(2));
    println!("Q: {}", r);

    let input_noise_matrix = DMatrix::from_diagonal(&DVector::from_vec(vec![1.0, deg2rad(30.0)]));
    let input_noise = input_noise_matrix.map(|elem :f64 | elem.powi(2));

    let gps_noise_matrix = DMatrix::from_diagonal(&DVector::from_vec(vec![0.5, 0.5]));
    let gps_noise = gps_noise_matrix.map(|elem :f64 | elem.powi(2));

    let dt = 0.1;
    let sim_time = 50.0;

    let mut time = 0.0;
    let mut x_estimate = DMatrix::<f64>::zeros(4, 1);
    let mut x_true = DMatrix::<f64>::zeros(4, 1);
    let mut p_estimate = DMatrix::<f64>::identity(4, 4);

    let x_dead_reckoning = DMatrix::<f64>::zeros(4, 1);

    // history
    let mut hx_estimate = &x_estimate;
    let mut hx_true = &x_true;
    let mut hx_dead_reckoning = &x_true;
    let hz = DMatrix::<f64>::zeros(2, 1);

    while (sim_time >= time) {
        time += dt;
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