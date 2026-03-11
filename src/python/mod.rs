use pyo3::prelude::*;

mod collision_checker;
mod collision_object;
mod dynamic_obstacle;
mod pose;

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
