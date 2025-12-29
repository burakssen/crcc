use crate::time::TimeStep;
use itertools::{Itertools, chain};
use std::collections::{BTreeMap, Bound};
use std::ops::{RangeBounds, RangeInclusive};

#[derive(Debug, Clone)]
pub struct TimeStepSet {
    included: BTreeMap<TimeStep, bool>,
}

impl TimeStepSet {
    pub fn new() -> Self {
        let mut included = BTreeMap::new();
        included.insert(TimeStep::MIN, false);
        Self { included }
    }

    pub fn contains(&self, time_step: TimeStep) -> bool {
        *self
            .included
            .range(..=time_step)
            .next_back()
            .expect("At least MIN should be present")
            .1
    }

    pub fn add(&mut self, range: impl RangeBounds<TimeStep>) {
        self.set(range, true)
    }

    pub fn remove(&mut self, range: impl RangeBounds<TimeStep>) {
        self.set(range, false)
    }

    fn set(&mut self, range: impl RangeBounds<TimeStep>, value: bool) {
        // Check if range is empty
        let is_empty = match (range.start_bound(), range.end_bound()) {
            (Bound::Included(start), Bound::Included(end)) => start > end,
            (Bound::Included(start), Bound::Excluded(end))
            | (Bound::Excluded(start), Bound::Included(end))
            | (Bound::Excluded(start), Bound::Excluded(end)) => start >= end,
            _ => false,
        };
        if is_empty {
            return;
        }

        // Find the old value at the upper bound
        let ub_value = match range.end_bound() {
            Bound::Included(t) => {
                if t.next() < TimeStep::MAX {
                    Some((t.next(), self.contains(t.next())))
                } else {
                    None
                }
            }
            Bound::Excluded(t) => {
                if *t < TimeStep::MAX {
                    Some((*t, self.contains(*t)))
                } else {
                    None
                }
            }
            Bound::Unbounded => None,
        };

        // Erase all existing entries in the range and the upper bound (if it has the same value)
        self.included
            .retain(|step, _| !range.contains(step) && ub_value.is_none_or(|(ub, _)| *step != ub));

        // Set to value at the lower bound, if not already set
        let lb = match range.start_bound() {
            Bound::Included(t) => *t,
            Bound::Excluded(t) => t.next(),
            Bound::Unbounded => TimeStep::MIN,
        };
        if lb == TimeStep::MIN || self.contains(lb) != value {
            self.included.insert(lb, value);
        }

        // Restore the upper bound value, if it is different
        if let Some((ub, ub_included)) = ub_value
            && ub_included != value
        {
            self.included.insert(ub, ub_included);
        }
    }

    pub fn union(&mut self, other: &Self) {
        for range in other.included_ranges() {
            self.add(range);
        }
    }

    pub fn intersect(&mut self, other: &Self) {
        for (range, included) in other.ranges_tf() {
            if !included {
                self.remove(range);
            }
        }
    }

    pub fn included_ranges(&self) -> impl Iterator<Item = RangeInclusive<TimeStep>> + '_ {
        self.ranges_tf().filter_map(
            |(range, included)| {
                if included { Some(range) } else { None }
            },
        )
    }

    fn ranges_tf(&self) -> impl Iterator<Item = (RangeInclusive<TimeStep>, bool)> {
        chain!(self.included.iter().map(Some), std::iter::once(None))
            .tuple_windows()
            .map(|(start, opt_last)| {
                let Some((start, start_included)) = start else {
                    unreachable!("Only the last item should be None");
                };
                let end = opt_last.map(|(end, _)| end.prev()).unwrap_or(TimeStep::MAX);
                (*start..=end, *start_included)
            })
    }

    pub fn iter(&self) -> impl Iterator<Item = TimeStep> {
        let mut ranges = self.included_ranges();
        let mut current_range = ranges.next().map(TimeStep::iter_range);
        std::iter::from_fn(move || {
            loop {
                if let Some(r) = current_range.as_mut() {
                    if let Some(t) = r.next() {
                        return Some(t);
                    } else {
                        current_range = ranges.next().map(TimeStep::iter_range)
                    }
                } else {
                    return None;
                }
            }
        })
    }
}

impl Default for TimeStepSet {
    fn default() -> Self {
        Self::new()
    }
}

impl<T: RangeBounds<TimeStep>> From<T> for TimeStepSet {
    fn from(range: T) -> Self {
        let mut set = TimeStepSet::new();
        set.add(range);
        set
    }
}

impl<T: RangeBounds<TimeStep>> FromIterator<T> for TimeStepSet {
    fn from_iter<I: IntoIterator<Item = T>>(iter: I) -> Self {
        let mut set = TimeStepSet::new();
        for range in iter {
            set.add(range);
        }
        set
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_time_step_set() {
        let mut set = TimeStepSet::new();
        assert!(!set.contains(TimeStep(0)));
        assert!(!set.contains(TimeStep(10)));

        set.add(TimeStep(5)..TimeStep(15));
        assert!(!set.contains(TimeStep(4)));
        assert!(set.contains(TimeStep(5)));
        assert!(set.contains(TimeStep(10)));
        assert!(!set.contains(TimeStep(15)));
        assert!(!set.contains(TimeStep(16)));

        set.remove(TimeStep(10)..=TimeStep(12));
        assert!(set.contains(TimeStep(9)));
        assert!(!set.contains(TimeStep(10)));
        assert!(!set.contains(TimeStep(11)));
        assert!(!set.contains(TimeStep(12)));
        assert!(set.contains(TimeStep(13)));

        let included_ranges = set.included_ranges().collect_vec();
        assert_eq!(
            included_ranges,
            vec![TimeStep(5)..=TimeStep(9), TimeStep(13)..=TimeStep(14)]
        );

        let included_steps = set.iter().collect_vec();
        assert_eq!(
            included_steps,
            vec![
                TimeStep(5),
                TimeStep(6),
                TimeStep(7),
                TimeStep(8),
                TimeStep(9),
                TimeStep(13),
                TimeStep(14),
            ]
        );
    }

    #[test]
    fn test_time_step_set_unbounded() {
        let mut set = TimeStepSet::new();
        assert!(!set.contains(TimeStep(0)));
        assert!(!set.contains(TimeStep(10)));

        set.add(..);
        assert!(set.contains(TimeStep::MIN));
        assert!(set.contains(TimeStep(-1234567)));
        assert!(set.contains(TimeStep(-10)));
        assert!(set.contains(TimeStep::ZERO));
        assert!(set.contains(TimeStep(10)));
        assert!(set.contains(TimeStep(1234567)));
        assert!(set.contains(TimeStep::MAX));

        set.remove(..TimeStep::ZERO);
        assert!(!set.contains(TimeStep::MIN));
        assert!(!set.contains(TimeStep(-1234567)));
        assert!(!set.contains(TimeStep(-10)));
        assert!(set.contains(TimeStep::ZERO));
        assert!(set.contains(TimeStep(10)));
        assert!(set.contains(TimeStep(1234567)));
        assert!(set.contains(TimeStep::MAX));

        let included_ranges = set.included_ranges().collect_vec();
        assert_eq!(included_ranges, vec![TimeStep(0)..=TimeStep::MAX]);
    }

    #[test]
    fn test_time_step_set_union() {
        let mut set1 = TimeStepSet::from(TimeStep(5)..TimeStep(15));
        let set2 = TimeStepSet::from(TimeStep(10)..=TimeStep(20));
        let set3 = TimeStepSet::from(TimeStep(0)..=TimeStep(4));

        set1.union(&set2);
        let included_ranges = set1.included_ranges().collect_vec();
        assert_eq!(included_ranges, vec![TimeStep(5)..=TimeStep(20)]);

        set1.union(&set3);
        let included_ranges = set1.included_ranges().collect_vec();
        assert_eq!(included_ranges, vec![TimeStep(0)..=TimeStep(20)]);
    }

    #[test]
    fn test_time_step_set_intersect() {
        let mut set1 = TimeStepSet::from(TimeStep(5)..TimeStep(15));
        let set2 = TimeStepSet::from(TimeStep(10)..=TimeStep(20));
        let set3 = TimeStepSet::from(TimeStep(0)..=TimeStep(4));

        set1.intersect(&set2);
        let included_ranges = set1.included_ranges().collect_vec();
        assert_eq!(included_ranges, vec![TimeStep(10)..=TimeStep(14)]);

        set1.intersect(&set3);
        let included_ranges = set1.included_ranges().collect_vec();
        assert!(included_ranges.is_empty());
    }
}
