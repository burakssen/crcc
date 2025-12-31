pub use crate::time::set::TimeStepSet;
use derive_more::{Add, Mul, Sub, Sum};
use std::fmt::Display;
use std::ops::RangeBounds;

mod set;

pub(crate) type TimeStepInner = i32;

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Add, Mul, Sub, Sum)]
pub struct TimeStep(pub TimeStepInner);

impl TimeStep {
    pub const MIN: Self = Self(i32::MIN);
    pub const MAX: Self = Self(i32::MAX);
    pub const ZERO: Self = Self(0);

    pub fn pred(&self) -> Self {
        Self(self.0.saturating_sub(1))
    }

    pub fn succ(&self) -> Self {
        Self(self.0.saturating_add(1))
    }

    pub fn iter_range(range: impl RangeBounds<Self>) -> impl Iterator<Item = Self> {
        let start = match range.start_bound() {
            std::ops::Bound::Included(t) => *t,
            std::ops::Bound::Excluded(t) => t.succ(),
            std::ops::Bound::Unbounded => Self::MIN,
        };
        let end = match range.end_bound() {
            std::ops::Bound::Included(t) => *t,
            std::ops::Bound::Excluded(t) => t.pred(),
            std::ops::Bound::Unbounded => Self::MAX,
        };
        (start.0..=end.0).map(TimeStep)
    }
}

impl Display for TimeStep {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "t_{}", self.0)
    }
}

impl From<TimeStepInner> for TimeStep {
    fn from(value: TimeStepInner) -> Self {
        Self(value)
    }
}
