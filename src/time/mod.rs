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
    #[must_use]
    pub const fn pred(&self) -> Self {
        Self(self.0.saturating_sub(1))
    }

    /// Returns the following step, saturating at [`TimeStep::MAX`].
    #[must_use]
    pub const fn succ(&self) -> Self {
        Self(self.0.saturating_add(1))
    }

    /// Returns the following step, or `None` at [`TimeStep::MAX`].
    #[must_use]
    pub const fn checked_succ(&self) -> Option<Self> {
        match self.0.checked_add(1) {
            Some(value) => Some(Self(value)),
            None => None,
        }
    }

    /// Advances by `steps`, saturating at [`TimeStep::MAX`].
    #[must_use]
    pub fn add_steps(&self, steps: usize) -> Self {
        let steps = i64::try_from(steps).unwrap_or(i64::MAX);
        let value = i64::from(self.0).saturating_add(steps);

        Self(i32::try_from(value).unwrap_or(i32::MAX))
    }

    /// Advances by `steps`, returning `None` when the result is not representable.
    #[must_use]
    pub fn checked_add_steps(&self, steps: usize) -> Option<Self> {
        let steps = i64::try_from(steps).ok()?;
        let value = i64::from(self.0).checked_add(steps)?;
        i32::try_from(value).ok().map(Self)
    }

    /// Iterates over the discrete steps selected by a Rust range.
    ///
    /// Included and excluded bounds follow normal [`RangeBounds`] semantics.
    pub fn iter_range(range: impl RangeBounds<Self>) -> impl Iterator<Item = Self> {
        let start = match range.start_bound() {
            std::ops::Bound::Included(t) => i64::from(t.0),
            std::ops::Bound::Excluded(t) => i64::from(t.0).saturating_add(1),
            std::ops::Bound::Unbounded => i64::from(Self::MIN.0),
        };
        let end = match range.end_bound() {
            std::ops::Bound::Included(t) => i64::from(t.0),
            std::ops::Bound::Excluded(t) => i64::from(t.0).saturating_sub(1),
            std::ops::Bound::Unbounded => i64::from(Self::MAX.0),
        };
        (start..=end).filter_map(|value| i32::try_from(value).ok().map(TimeStep))
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

        let steps = usize::try_from(i32::MAX)
            .unwrap_or(usize::MAX)
            .saturating_add(1);

        assert_eq!(TimeStep::MIN.add_steps(steps), TimeStep(0));
    }

    #[test]
    fn checked_steps_reject_overflow() {
        assert_eq!(TimeStep::MAX.checked_succ(), None);
        assert_eq!(TimeStep::MAX.checked_add_steps(1), None);
        assert_eq!(TimeStep(4).checked_add_steps(2), Some(TimeStep(6)));
    }

    #[test]
    fn predecessor_and_successor_saturate_at_extremes() {
        assert_eq!(TimeStep::MIN.pred(), TimeStep::MIN);
        assert_eq!(TimeStep::MAX.succ(), TimeStep::MAX);
        assert_eq!(TimeStep::MIN.succ(), TimeStep(TimeStepInner::MIN + 1));
        assert_eq!(TimeStep::MAX.pred(), TimeStep(TimeStepInner::MAX - 1));
    }

    #[test]
    fn checked_add_spans_the_full_representable_range() {
        let full_span = usize::try_from(u32::MAX).unwrap_or(usize::MAX);

        assert_eq!(
            TimeStep::MIN.checked_add_steps(full_span),
            Some(TimeStep::MAX)
        );
        assert_eq!(
            TimeStep::MIN.checked_add_steps(full_span.saturating_add(1)),
            None
        );
    }

    #[test]
    fn excluded_extreme_bounds_are_empty() {
        use std::ops::Bound::{Excluded, Unbounded};

        assert_eq!(
            TimeStep::iter_range((Excluded(TimeStep::MAX), Unbounded)).count(),
            0
        );
        assert_eq!(
            TimeStep::iter_range((Unbounded, Excluded(TimeStep::MIN))).count(),
            0
        );
        assert_eq!(
            TimeStep::iter_range(TimeStep::MAX..=TimeStep::MAX).collect::<Vec<_>>(),
            vec![TimeStep::MAX],
        );
        assert_eq!(
            TimeStep::iter_range(TimeStep::MIN..=TimeStep::MIN).collect::<Vec<_>>(),
            vec![TimeStep::MIN],
        );
    }
}
