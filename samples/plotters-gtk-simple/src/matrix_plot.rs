use gtk::{glib, prelude::*, Application, ApplicationWindow, DrawingArea};
use nalgebra::Matrix4x1;
use plotters::prelude::*;
use plotters_cairo::CairoBackend;
use std::cell::RefCell;
use std::rc::Rc;

fn main() {
    let application = Application::new(Some("com.example.Graph"), Default::default());
    application.connect_activate(|app| {
        let (drawing_area, data) = build_ui(app);
        update_data_and_redraw(drawing_area, data);
    });
    application.run();
}

fn build_ui(app: &gtk::Application) -> (DrawingArea, Rc<RefCell<Matrix4x1<f64>>>) {
    let window = ApplicationWindow::new(app);
    window.set_default_size(800, 600);

    let drawing_area = DrawingArea::new();
    window.set_child(Some(&drawing_area));

    // Initialize data with a 4x1 matrix filled with zeros
    let data = Rc::new(RefCell::new(Matrix4x1::zeros()));

    let data_clone = data.clone();
    drawing_area.set_draw_func(move |_, cr, width, height| {
        let root = CairoBackend::new(cr, (800, 600)).unwrap().into_drawing_area();
        root.fill(&WHITE).unwrap();
        let data = data_clone.borrow();

        let mut chart = ChartBuilder::on(&root)
            .margin(5)
            .x_label_area_size(30)
            .y_label_area_size(30)
            .build_cartesian_2d(0f64..4f64, -1.5f64..1.5f64)
            .unwrap();

        chart.configure_mesh().draw().unwrap();

        chart.draw_series(LineSeries::new(
            (0..4).map(|i| (i as f64, data[i])),
            &RED,
        )).unwrap();
    });

    window.show();

    (drawing_area, data)
}

fn update_data_and_redraw(drawing_area: DrawingArea, data: Rc<RefCell<Matrix4x1<f64>>>) {
    let mut counter = 0f64;
    glib::timeout_add_local(std::time::Duration::from_millis(100), move || {
        counter += 0.1;
        let y = counter.sin();

        // Shift data and add the new value
        let mut data_borrowed = data.borrow_mut();
        *data_borrowed = Matrix4x1::new(data_borrowed[1], data_borrowed[2], data_borrowed[3], y);

        drawing_area.queue_draw();

        glib::Continue(true)
    });
}
