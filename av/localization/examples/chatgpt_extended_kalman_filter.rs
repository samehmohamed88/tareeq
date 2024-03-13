use nalgebra::{DMatrix, Vector2, Vector4, Matrix4, Matrix2, Matrix4x2, Matrix2x4, Matrix4x1, Matrix2x1, OMatrix};
use std::f64::consts::PI;
use rand::thread_rng;
use rand::prelude::*;
use rand_distr::StandardNormal;

use gtk::{glib, prelude::*, Application, ApplicationWindow, DrawingArea};
use plotters::prelude::*;
use plotters_cairo::CairoBackend;
use std::rc::Rc;
use std::cell::RefCell;

const DT: f64 = 0.1;

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

fn jacob_f(x: &Matrix4x1<f64>, u: &Matrix2x1<f64>) -> Matrix4<f64> {
    let dt = 0.1; // time tick [s]
    let v = u[0];
    let yaw = x[2];
    Matrix4::<f64>::from_row_slice(&[
        1.0, 0.0, -dt * v * yaw.sin(), dt * yaw.cos(),
        0.0, 1.0, dt * v * yaw.cos(), dt * yaw.sin(),
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0,
    ])
}

fn jacob_h() -> Matrix2x4<f64> {
    Matrix2x4::<f64>::from_row_slice( &[
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
    ])
}

fn ekf_estimation(x_est: &Matrix4x1<f64>, p_est: &Matrix4<f64>, z: &Matrix2x1<f64>, ud: &Matrix2x1<f64>, q: &Matrix4<f64>, r: &Matrix2<f64>) -> (Matrix4x1<f64>, Matrix4<f64>) {
    // Predict
    let x_pred = motion_model(x_est, ud);
    let j_f = jacob_f(x_est, ud);
    let p_pred = &j_f * p_est * j_f.transpose() + q;

    // Update
    let j_h = jacob_h();
    let z_pred = observation_model(&x_pred);
    let y = z - &z_pred;
    let s = &j_h * &p_pred * j_h.transpose() + r;
    let k = p_pred * j_h.transpose() * s.try_inverse().unwrap();
    let x_est = &x_pred + &k * y;
    let p_est = (Matrix4::identity() - &k * j_h) * p_pred;
    (x_est, p_est)
}

fn run_ekf_estimate() -> (Matrix4x1<f64>, Matrix4<f64>) {
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

    let sim_time = 5.0;

    let mut time = 0.0;

    let mut x_estimate = Matrix4x1::<f64>::zeros();
    let mut x_true= Matrix4x1::<f64>::zeros();
    let mut p_estimate = Matrix4::<f64>::identity();
    // println!("P estimate : {}", p_estimate)

    let x_dead_reckoning = Matrix4x1::<f64>::zeros();

    // history
    let mut hx_estimate = Matrix4x1::<f64>::zeros();
    let mut hx_true = Matrix4x1::<f64>::zeros();
    let mut hx_dead_reckoning =  Matrix4x1::<f64>::zeros();
    let hz = Matrix2x1::<f64>::zeros();

    //Matrix2x1<f64>, Matrix4x1<f64>, Matrix2x1<f64>)
    let mut z_sum : Matrix2x1<f64> = Matrix2x1::<f64>::zeros();
    let mut xd : Matrix4x1<f64> = Matrix4x1::<f64>::zeros();
    let mut ud_sum : Matrix2x1<f64> = Matrix2x1::<f64>::zeros();
    let mut u : Matrix2x1<f64>  = Matrix2x1::<f64>::zeros();

    u = calc_input();
    (x_true, z_sum, xd, ud_sum) = observation(&x_true, &x_dead_reckoning, &u, &input_noise, &gps_noise);
    (x_estimate, p_estimate) = ekf_estimation(&x_estimate, &p_estimate, &z_sum, &ud_sum, &q, &r);
    println!("AFTER {}", x_estimate);

    (x_estimate, p_estimate)
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let application = Application::new(Some("com.example.Graph"), Default::default());
    application.connect_activate(|app| {
        let (drawing_area, data) = build_ui(app);
        update_data_and_redraw(drawing_area, data);
    });
    application.run();
    Ok(())
}


fn build_ui(app: &gtk::Application) -> (DrawingArea, Rc<RefCell<Matrix4x1<f64>>>){
    let window = ApplicationWindow::new(app);
    window.set_default_size(800, 600);

    let drawing_area = DrawingArea::new();
    window.set_child(Some(&drawing_area));

    // Initialize data
    // let data = Rc::new(RefCell::new(vec![]));
    let (x_estimate, p_estimate) = run_ekf_estimate();
    let data = Rc::new(RefCell::new(x_estimate));

    let data_clone = data.clone();
    drawing_area.set_draw_func(move |_, cr, width, height| {
        let root = CairoBackend::new(cr, (800, 600)).unwrap().into_drawing_area();
        root.fill(&WHITE).unwrap();
        let data = data_clone.borrow();

        if !data.is_empty() {
            let mut chart = ChartBuilder::on(&root)
                .margin(5)
                .x_label_area_size(30)
                .y_label_area_size(30)
                .build_cartesian_2d(0f64..10f64, -1.5f64..1.5f64)
                .unwrap();

            chart.configure_mesh().draw().unwrap();

            chart.draw_series(LineSeries::new(
                data.iter().cloned(),
                &RED,
            )).unwrap();
        }
    });

    window.show();

    (drawing_area, data)
}

fn update_data_and_redraw(drawing_area: DrawingArea, data: Rc<RefCell<Matrix4x1<f64>>>){
    let mut x = 0f64;
    glib::timeout_add_local(std::time::Duration::from_millis(100), move || { // Updates every 100 milliseconds
        // Simulate new data
        x += 0.1; // Adjust this value as needed for your simulation speed
        let y = (x / 10.0).sin();
        // data.borrow_mut().push((x, y));

        // Keep the data vector within a reasonable size
        // if data.borrow().len() > 100 { // Assuming we want to display the last 100 points
        //     data.borrow_mut().remove(0);
        // }

        drawing_area.queue_draw();

        glib::Continue(true)
    });
}
