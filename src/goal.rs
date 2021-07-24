
#[derive(Debug, Copy, Clone)]
pub struct ExactGoal<State> {
    pub goal: State
}

pub trait PredicateGoal<State> {
    fn is_goal(&self, candidate: &State) -> bool;
}

impl<State : PartialEq> PredicateGoal<State> for ExactGoal<State> {
    fn is_goal(&self, candidate: &State) -> bool {
        &self.goal == candidate
    }
}