use pyo3::prelude::*;

mod collision_checker;
mod collision_object;
mod dynamic_obstacle;
mod pose;

#[cfg(not(feature = "default-engine"))]
compile_error!(
    "you must choose a default collision engine when building Python bindings, e.g., `parry-default-engine`"
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
