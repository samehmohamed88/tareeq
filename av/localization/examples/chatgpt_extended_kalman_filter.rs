use nalgebra::{DMatrix, Vector2, Vector4, Matrix4, Matrix2, Matrix4x2, Matrix2x4, Matrix4x1, Matrix2x1};
use std::f64::consts::PI;
use rand::thread_rng;
use rand::prelude::*;
use rand_distr::StandardNormal;
use plotters::prelude::*;
use minifb::{Key, KeyRepeat, Window, WindowOptions};
use plotters::prelude::*;
use plotters_bitmap::bitmap_pixel::BGRXPixel;
use plotters_bitmap::BitMapBackend;
use std::collections::VecDeque;
use std::error::Error;
use std::time::SystemTime;
use std::borrow::{Borrow, BorrowMut};


const DT: f64 = 0.1;
const W: usize = 800;
const H: usize = 600;

const SAMPLE_RATE: f64 = 10_000.0;
const FRAME_RATE: f64 = 30.0;

struct BufferWrapper(Vec<u32>);
impl Borrow<[u8]> for BufferWrapper {
    fn borrow(&self) -> &[u8] {
        // Safe for alignment: align_of(u8) <= align_of(u32)
        // Safe for cast: u32 can be thought of as being transparent over [u8; 4]
        unsafe {
            std::slice::from_raw_parts(
                self.0.as_ptr() as *const u8,
                self.0.len() * 4
            )
        }
    }
}
impl BorrowMut<[u8]> for BufferWrapper {
    fn borrow_mut(&mut self) -> &mut [u8] {
        // Safe for alignment: align_of(u8) <= align_of(u32)
        // Safe for cast: u32 can be thought of as being transparent over [u8; 4]
        unsafe {
            std::slice::from_raw_parts_mut(
                self.0.as_mut_ptr() as *mut u8,
                self.0.len() * 4
            )
        }
    }
}
impl Borrow<[u32]> for BufferWrapper {
    fn borrow(&self) -> &[u32] {
        self.0.as_slice()
    }
}
impl BorrowMut<[u32]> for BufferWrapper {
    fn borrow_mut(&mut self) -> &mut [u32] {
        self.0.as_mut_slice()
    }
}

fn get_window_title(fx: f64, fy: f64, iphase: f64) -> String {
    format!(
        "x={:.1}Hz, y={:.1}Hz, phase={:.1} +/-=Adjust y 9/0=Adjust x <Esc>=Exit",
        fx, fy, iphase
    )
}


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

fn main() -> Result<(), Box<dyn std::error::Error>> {
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

    let mut fx: f64 = 1.0;
    let mut fy: f64 = 1.1;
    let mut xphase: f64 = 0.0;
    let mut yphase: f64 = 0.1;

    let mut buf = BufferWrapper(vec![0u32; W * H]);

    let mut window = Window::new(
        &get_window_title(fx, fy, yphase - xphase),
        W,
        H,
        WindowOptions::default(),
    )?;
    let cs = {
        let root =
            BitMapBackend::<BGRXPixel>::with_buffer_and_format(buf.borrow_mut(), (W as u32, H as u32))?
                .into_drawing_area();
        root.fill(&BLACK)?;

        let mut chart = ChartBuilder::on(&root)
            .margin(10)
            .set_all_label_area_size(30)
            .build_cartesian_2d(-1.2..1.2, -1.2..1.2)?;

        chart
            .configure_mesh()
            .label_style(("sans-serif", 15).into_font().color(&GREEN))
            .axis_style(&GREEN)
            .draw()?;

        let cs = chart.into_chart_state();
        root.present()?;
        cs
    };

    let mut data = VecDeque::new();
    let start_ts = SystemTime::now();
    let mut last_flushed = 0.0;

    let epoch = SystemTime::now()
        .duration_since(start_ts)
        .unwrap()
        .as_secs_f64();

    while (sim_time >= time) {
        time += DT;
        u = calc_input();

        (x_true, z_sum, xd, ud_sum) = observation(&x_true, &x_dead_reckoning, &u, &input_noise, &gps_noise);
        (x_estimate, p_estimate) = ekf_estimation(&x_estimate, &p_estimate, &z_sum, &ud_sum, &q, &r);
        println!("AFTER {}", x_estimate);

        // while window.is_open() && !window.is_key_down(Key::Escape) {


            if let Some((ts, _, _)) = data.back() {
                if epoch - ts < 1.0 / SAMPLE_RATE {
                    std::thread::sleep(std::time::Duration::from_secs_f64(epoch - ts));
                    continue;
                }
                let mut ts = *ts;
                while ts < epoch {
                    ts += 1.0 / SAMPLE_RATE;
                    let phase_x: f64 = 2.0 * ts * std::f64::consts::PI * fx + xphase;
                    let phase_y: f64 = 2.0 * ts * std::f64::consts::PI * fy + yphase;
                    data.push_back((ts, phase_x.sin(), phase_y.sin()));
                }
            }

            let phase_x = 2.0 * epoch * std::f64::consts::PI * fx + xphase;
            let phase_y = 2.0 * epoch * std::f64::consts::PI * fy + yphase;
            data.push_back((epoch, phase_x.sin(), phase_y.sin()));

            if epoch - last_flushed > 1.0 / FRAME_RATE {
                {
                    let root = BitMapBackend::<BGRXPixel>::with_buffer_and_format(
                        buf.borrow_mut(),
                        (W as u32, H as u32),
                    )?
                        .into_drawing_area();
                    {
                        let mut chart = cs.clone().restore(&root);
                        chart.plotting_area().fill(&BLACK)?;

                        chart
                            .configure_mesh()
                            .bold_line_style(&GREEN.mix(0.2))
                            .light_line_style(&TRANSPARENT)
                            .draw()?;

                        chart.draw_series(data.iter().zip(data.iter().skip(1)).map(
                            |(&(e, x0, y0), &(_, x1, y1))| {
                                PathElement::new(
                                    vec![(x0, y0), (x1, y1)],
                                    &GREEN.mix(((e - epoch) * 20.0).exp()),
                                )
                            },
                        ))?;
                    }
                    root.present()?;

                    let keys = window.get_keys_pressed(KeyRepeat::Yes);
                    for key in keys {
                        let old_fx = fx;
                        let old_fy = fy;
                        match key {
                            Key::Equal => {
                                fy += 0.1;
                            }
                            Key::Minus => {
                                fy -= 0.1;
                            }
                            Key::Key0 => {
                                fx += 0.1;
                            }
                            Key::Key9 => {
                                fx -= 0.1;
                            }
                            _ => {
                                continue;
                            }
                        }
                        xphase += 2.0 * epoch * std::f64::consts::PI * (old_fx - fx);
                        yphase += 2.0 * epoch * std::f64::consts::PI * (old_fy - fy);
                        window.set_title(&get_window_title(fx, fy, yphase - xphase));
                    }
                }

                window.update_with_buffer(buf.borrow(), W, H)?;
                last_flushed = epoch;
            }

            while let Some((e, _, _)) = data.front() {
                if ((e - epoch) * 20.0).exp() > 0.1 {
                    break;
                }
                data.pop_front();
            }
        // }

    }
    Ok(())
}
