use std::error::Error;
use std::fmt::{Display, Formatter};

#[derive(Debug, Clone, PartialEq)]
pub enum CrccError {
    InvalidRadius(f64),
    NotConvex,
    HasHoles,
    EmptyShape,
    Unsupported,
}

impl Display for CrccError {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        match self {
            CrccError::InvalidRadius(r) => write!(f, "Circle radius must be positive, got {}.", r),
            CrccError::NotConvex => write!(f, "Shape must be convex."),
            CrccError::HasHoles => write!(f, "Shape may not have holes."),
            CrccError::EmptyShape => write!(f, "Shape must not be empty."),
            CrccError::Unsupported => write!(
                f,
                "Collision checking of shape combination is not supported."
            ),
        }
    }
}

impl Error for CrccError {}

pub type CrccResult<T> = Result<T, CrccError>;
