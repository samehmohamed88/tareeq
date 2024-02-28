use gdal::vector::Dataset;
use gdal::vector::Geometry;

fn main() -> Result<(), gdal::errors::Error> {
    // gdal::config::set_config_option("GDAL_DATA", "/path/to/gdal/data/directory");

    let dataset = Dataset::open("/home/sameh.mohamed/Development/tareeq/av/maps/indoor-loop/indoor_loop.gpkg")?;
    for layer in dataset.layers() {
        for feature in layer.features() {
            let geometry: Geometry = feature.geometry()?;
            println!("Geometry WKT: {}", geometry.wkt()?);
            // Additional processing...
        }
    }
    Ok(())
}
