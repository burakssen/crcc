use pyo3::prelude::*;

mod road_boundary;

#[pyfunction]
fn hello() -> PyResult<()> {
    println!("Hello from Rust!");
    Ok(())
}

#[pymodule]
fn commonroad_collision_checker(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(hello, m)?)?;
    m.add_class::<road_boundary::RoadBoundaryChecker>()?;
    Ok(())
}
