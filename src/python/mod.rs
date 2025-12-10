use pyo3::prelude::*;

mod collision_checker;
mod collision_object;
mod isometry;

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
    use super::collision_checker::collision_checker;
    #[pymodule_export]
    use super::collision_object::collision_object;
    #[pymodule_export]
    use super::isometry::isometry;
}
