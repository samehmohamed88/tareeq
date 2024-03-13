use gtk::{prelude::*, DrawingArea};
use plotters::prelude::*;
use plotters_cairo::CairoBackend;

fn main() {
    // Initialize GTK.
    let application = gtk::Application::new(Some("com.example.Graph"), Default::default());
    application.connect_activate(build_ui);
    application.run();
}

fn build_ui(app: &gtk::Application) {
    let window = gtk::ApplicationWindow::new(app);
    // window.set_title("Sine and Cosine Graph");
    window.set_default_size(800, 600);

    let drawing_area = DrawingArea::new();
    window.set_child(Some(&drawing_area));

    drawing_area.set_draw_func(|_, cr, width, height| {
        let root = CairoBackend::new(cr, (800, 600)).unwrap().into_drawing_area();
        root.fill(&WHITE).unwrap();

        let mut chart = ChartBuilder::on(&root)
            .margin(5)
            .x_label_area_size(30)
            .y_label_area_size(30)
            .build_cartesian_2d(-3.14..3.14, -1.5..1.5)
            .unwrap();

        chart.configure_mesh().draw().unwrap();

        chart
            .draw_series(LineSeries::new(
                (-314..314).map(|x| x as f64 / 100.0).map(|x| (x, x.sin())),
                &RED,
            ))
            .unwrap();

        chart
            .draw_series(LineSeries::new(
                (-314..314).map(|x| x as f64 / 100.0).map(|x| (x, x.cos())),
                &BLUE,
            ))
            .unwrap();
    });

    window.show();
}
