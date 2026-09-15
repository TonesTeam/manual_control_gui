// Release builds on Windows are GUI apps: no console window behind the main window.
#![cfg_attr(all(not(debug_assertions), target_os = "windows"), windows_subsystem = "windows")]

mod app;
mod bus;
mod config;
mod controller_api;
mod devices;
mod protocol;
mod schematic;
mod tracking;

/// wgpu by default; `TSTAND_RENDERER=glow` switches to OpenGL where wgpu can't start.
fn renderer() -> eframe::Renderer {
    match std::env::var("TSTAND_RENDERER").map(|v| v.to_ascii_lowercase()).as_deref() {
        Ok("glow" | "opengl" | "gl") => eframe::Renderer::Glow,
        _ => eframe::Renderer::Wgpu,
    }
}

fn main() -> eframe::Result<()> {
    let native_options = eframe::NativeOptions {
        // Sizes are in points; eframe maps them through the monitor's scale factor,
        // and the window is shrunk to fit smaller or heavily scaled screens.
        viewport: eframe::egui::ViewportBuilder::default()
            .with_inner_size([1480.0, 900.0])
            .with_min_inner_size([800.0, 520.0])
            .with_clamp_size_to_monitor_size(true)
            .with_app_id("tones-liquid-processing")
            .with_title("Tones Liquid Processing — Monitor & Control"),
        centered: true,
        renderer: renderer(),
        ..Default::default()
    };
    eframe::run_native(
        "Tones Liquid Processing",
        native_options,
        Box::new(|cc| Ok(Box::new(app::App::new(cc)))),
    )
}
