use pyo3::prelude::*;

#[pyfunction]
fn hello() -> PyResult<()> {
    println!("Hello from Rust!");
    Ok(())
}

#[pymodule]
fn commonroad_collision_checker(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(hello, m)?)?;
    Ok(())
}
