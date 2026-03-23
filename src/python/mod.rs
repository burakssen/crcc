use crate::error::CrccError;
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;

mod collision_checker;
mod collision_object;
mod dynamic_obstacle;
mod pose;

impl From<CrccError> for PyErr {
    fn from(value: CrccError) -> Self {
        match value {
            CrccError::Unsupported => PyValueError::new_err("Unsupported shape combination"),
            CrccError::InvalidRadius(r) => {
                PyValueError::new_err(format!("Circle radius must be positive, got {}.", r))
            }
            CrccError::NotConvex => PyValueError::new_err("Shape must be convex."),
            CrccError::HasHoles => PyValueError::new_err("Shape may not have holes."),
            CrccError::EmptyShape => PyValueError::new_err("Shape must not be empty."),
        }
    }
}

#[cfg(not(any(feature = "parry", feature = "rhusics")))]
compile_error!(
    "you must enable at least one collision engine when building Python bindings, e.g., `parry`"
);

#[pymodule]
mod _core {
    #[pymodule_export]
    use super::collision_checker::collision_checker;
    #[pymodule_export]
    use super::collision_object::collision_object;
    #[pymodule_export]
    use super::dynamic_obstacle::dynamic_obstacle;
    #[pymodule_export]
    use super::pose::pose;
}
