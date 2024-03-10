
use nalgebra::{DMatrix, Vector2, Vector4, Matrix4, Matrix2, Matrix4x2, Matrix2x4, Matrix4x1, Matrix2x1, OMatrix, U2, U1};
use std::f64::consts::PI;
use rand::thread_rng;
use rand::prelude::*;
use rand_distr::StandardNormal;

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
    input_noise: &Matrix2<f64>,
    gpu_noise: &Matrix2<f64>
) -> (Matrix4x1<f64>, Matrix2x1<f64>, Matrix4x1<f64>, Matrix2x1<f64>) {

    // println!("BEFORE {}", x_true);

    let x_true = motion_model(x_true, u);

    let mut rng = thread_rng();

    let input_random_matrix: DMatrix<f64> =
        DMatrix::from_iterator(2, 1, StandardNormal.sample_iter(&mut rng).take(2));

    // let gpu_random_matrix: OMatrix<f64, U2, U1> = OMatrix::from_fn(|_, _| rng.sample(StandardNormal));
    let gpu_random_matrix: DMatrix<f64> =
        DMatrix::from_iterator(2, 1, StandardNormal.sample_iter(&mut rng).take(2));

    let z_sum = observation_model(&x_true) + (gpu_noise  * gpu_random_matrix);
    // let z = z_sum.into();

    let ud_sum = u + (input_noise * input_random_matrix);
    // let ud = ud_sum.into();
    let xd = motion_model(xd, &ud_sum);

    (x_true, z_sum, xd, ud_sum)
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

    h * x
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
    let q_matrix = Matrix4::from_diagonal(&Vector4::from_vec(vec![0.1, 0.1, deg2rad(1.0), 1.0]));
    // Square each element individually
    let q = q_matrix.map(|elem: f64| elem.powi(2));
    // println!("Q: {}", q);

    let r_matrix = Matrix2::from_diagonal(&Vector2::from_vec(vec![1.0, 1.0]));
    let r = r_matrix.map(|elem: f64| elem.powi(2));
    // println!("R: {}", r);

    let input_noise_matrix = Matrix2::from_diagonal(&Vector2::from_vec(vec![1.0, deg2rad(30.0)]));
    let input_noise = input_noise_matrix.map(|elem: f64| elem.powi(2));

    let gps_noise_matrix = Matrix2::from_diagonal(&Vector2::from_vec(vec![0.5, 0.5]));
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
    let hz = Matrix2x1::<f64>::zeros();

    //Matrix2x1<f64>, Matrix4x1<f64>, Matrix2x1<f64>)
    let mut z_sum : Matrix2x1<f64> = Matrix2x1::<f64>::zeros();
    let mut xd : Matrix4x1<f64> = Matrix4x1::<f64>::zeros();
    let mut ud_sum : Matrix2x1<f64> = Matrix2x1::<f64>::zeros();

    while (sim_time >= time) {
        //println!("BEGIN LOOP {}", x_true);
        time += DT;
        let u = calc_input();
        //observation(xTrue, xDR, u)
        (x_true, z_sum, xd, ud_sum) = observation(&x_true, &x_dead_reckoning, &u, &input_noise, &gps_noise);
        // println!("AFTER {}", x_true);
    }

    // Initial state
    // let mut x_est = Vector4::from_vec(vec![0.0, 0.0, 0.0, 0.0]); // State vector [x y yaw v]'
    // let mut p_est: OMatrix<f64, U4, U4> = OMatrix::identity(); // State covariance matrix
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
