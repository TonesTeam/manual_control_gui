mod app;
mod config;
mod pump;
mod selector_valve;
mod utils;
mod worker;

fn main() -> eframe::Result<()> {
    let native_options = eframe::NativeOptions {
        viewport: eframe::egui::ViewportBuilder::default().with_inner_size([720.0, 640.0]),
        ..Default::default()
    };
    eframe::run_native(
        "Test Stand Controller",
        native_options,
        Box::new(|cc| Ok(Box::new(app::App::new(cc)))),
    )
}
