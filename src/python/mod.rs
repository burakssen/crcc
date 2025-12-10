use pyo3::prelude::*;

mod collision_object;
mod isometry;
mod road_boundary;

#[pyfunction]
fn hello() -> PyResult<()> {
    println!("Hello from Rust!");
    Ok(())
}

#[pymodule]
mod _core {
    #[pymodule_export]
    use super::hello;

    #[pymodule_export]
    use super::collision_object::collision_object;
    #[pymodule_export]
    use super::isometry::isometry;
    #[pymodule_export]
    use super::road_boundary::road_boundary;
}
