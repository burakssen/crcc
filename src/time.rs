use derive_more::{Add, Mul, Sub, Sum};
use std::fmt::Display;

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Add, Mul, Sub, Sum)]
pub struct TimeStep(pub i32);

impl TimeStep {
    pub fn zero() -> TimeStep {
        TimeStep(0)
    }

    pub fn prev(&self) -> TimeStep {
        TimeStep(self.0 - 1)
    }

    pub fn next(&self) -> TimeStep {
        TimeStep(self.0 + 1)
    }
}

impl Display for TimeStep {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "t_{}", self.0)
    }
}
