use derive_more::Display;

#[derive(Debug, Clone, PartialEq, Display)]
/// An error returned when geometry is invalid or an engine cannot perform a query.
pub enum CrccError {
    /// A circle radius was non-finite or not strictly positive.
    #[display("Circle radius must be positive, got {_0}.")]
    InvalidRadius(f64),
    /// A query required convex geometry but received a non-convex shape.
    #[display("Shape must be convex.")]
    NotConvex,
    /// A query required a polygon without interior rings.
    #[display("Shape may not have holes.")]
    HasHoles,
    /// A shape has no area or vertices.
    #[display("Shape must not be empty.")]
    EmptyShape,
    /// Geometry is degenerate, non-finite, or topologically invalid.
    #[display("Invalid geometry: {_0}.")]
    InvalidGeometry(&'static str),
    /// The selected engine or shape-operation combination is unavailable.
    #[display("Collision checking of shape combination is not supported.")]
    Unsupported,
}

impl std::error::Error for CrccError {}

/// The result type used by CRCC operations.
pub type CrccResult<T> = Result<T, CrccError>;
