use gtk::{glib, prelude::*, Application, ApplicationWindow, DrawingArea};
use plotters::prelude::*;
use plotters_cairo::CairoBackend;
use std::rc::Rc;
use std::cell::RefCell;

const DT: f64 = 0.1;

fn main() {
    let application = Application::new(Some("com.example.Graph"), Default::default());
    application.connect_activate(|app| {
        let (drawing_area, data) = build_ui(app);
        update_data_and_redraw(drawing_area, data);
    });
    application.run();
}

fn build_ui(app: &gtk::Application) -> (DrawingArea, Rc<RefCell<Vec<(f64, f64)>>>){
    let window = ApplicationWindow::new(app);
    window.set_default_size(800, 600);

    let drawing_area = DrawingArea::new();
    window.set_child(Some(&drawing_area));

    // Initialize data
    let data = Rc::new(RefCell::new(vec![]));

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

fn update_data_and_redraw(drawing_area: DrawingArea, data: Rc<RefCell<Vec<(f64, f64)>>>){
    let mut x = 0f64;
    glib::timeout_add_seconds_local(1, move || {
        // Simulate new data
        x += 1.0;
        let y = (x / 10.0).sin();
        data.borrow_mut().push((x, y));

        // Keep the data vector within a reasonable size
        if x > 10.0 {
            data.borrow_mut().remove(0);
        }

        drawing_area.queue_draw();

        glib::Continue(true)
    });
}
