pub type TimeStepSet = std::collections::BTreeSet<TimeStep>;
use derive_more::{Add, From, Mul, Sub, Sum};
use std::fmt::Display;
use std::ops::RangeBounds;

pub(crate) type TimeStepInner = i32;

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Add, Mul, Sub, Sum, From)]
/// A discrete trajectory time step.
pub struct TimeStep(pub TimeStepInner);

impl TimeStep {
    /// The smallest representable time step.
    pub const MIN: Self = Self(i32::MIN);
    /// The largest representable time step.
    pub const MAX: Self = Self(i32::MAX);
    /// Time step zero.
    pub const ZERO: Self = Self(0);

    /// Returns the preceding step, saturating at [`TimeStep::MIN`].
    pub fn pred(&self) -> Self {
        Self(self.0.saturating_sub(1))
    }

    /// Returns the following step, saturating at [`TimeStep::MAX`].
    pub fn succ(&self) -> Self {
        Self(self.0.saturating_add(1))
    }

    /// Advances by `steps`, saturating at [`TimeStep::MAX`].
    pub fn add_steps(&self, steps: usize) -> Self {
        let steps = i64::try_from(steps).unwrap_or(i64::MAX);
        Self((i64::from(self.0) + steps).min(i64::from(i32::MAX)) as i32)
    }

    /// Iterates over the discrete steps selected by a Rust range.
    ///
    /// Included and excluded bounds follow normal [`RangeBounds`] semantics.
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn iter_range_expands_supported_bounds() {
        let range = TimeStep::iter_range(TimeStep(3)..=TimeStep(7));
        let collected: Vec<TimeStep> = range.collect();
        assert_eq!(
            collected,
            vec![
                TimeStep(3),
                TimeStep(4),
                TimeStep(5),
                TimeStep(6),
                TimeStep(7)
            ]
        );

        let range = TimeStep::iter_range(..TimeStep(TimeStepInner::MIN + 3));
        let collected: Vec<TimeStep> = range.collect();
        assert_eq!(
            collected,
            vec![
                TimeStep(TimeStepInner::MIN),
                TimeStep(TimeStepInner::MIN + 1),
                TimeStep(TimeStepInner::MIN + 2)
            ]
        );

        let range = TimeStep::iter_range(TimeStep(TimeStepInner::MAX - 2)..);
        let collected: Vec<TimeStep> = range.collect();
        assert_eq!(
            collected,
            vec![
                TimeStep(TimeStepInner::MAX - 2),
                TimeStep(TimeStepInner::MAX - 1),
                TimeStep(TimeStepInner::MAX)
            ]
        );
    }

    #[test]
    fn add_steps_saturates_without_wrapping() {
        assert_eq!(TimeStep(0).add_steps(usize::MAX), TimeStep::MAX);
        assert_eq!(TimeStep::MAX.add_steps(1), TimeStep::MAX);
        assert_eq!(TimeStep::MIN.add_steps(i32::MAX as usize + 1), TimeStep(0));
    }
}
