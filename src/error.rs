use derive_more::Display;

#[derive(Debug, Clone, PartialEq, Display)]
pub enum CrccError {
    #[display("Circle radius must be positive, got {_0}.")]
    InvalidRadius(f64),
    #[display("Shape must be convex.")]
    NotConvex,
    #[display("Shape may not have holes.")]
    HasHoles,
    #[display("Shape must not be empty.")]
    EmptyShape,
    #[display("Collision checking of shape combination is not supported.")]
    Unsupported,
}

impl std::error::Error for CrccError {}

pub type CrccResult<T> = Result<T, CrccError>;
