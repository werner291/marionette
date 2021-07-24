use acap::vp::VpTree;
use acap::{NearestNeighbors, Proximity};
use petgraph::graph::{NodeIndex, UnGraph};

struct StateWithNode<State> {
    state: State,
    node_id: NodeIndex,
}

impl<State: Proximity> Proximity for StateWithNode<State> {
    type Distance = State::Distance;

    fn distance(&self, other: &Self) -> Self::Distance {
        self.state.distance(&other.state)
    }
}

struct PRM<State: Proximity> {
    graph: UnGraph<State, ()>,
    lookup_nearest: VpTree<StateWithNode<State>>,
}

impl<State: Proximity> PRM<State> {
    pub fn peek_graph(&self) -> &UnGraph<State, ()> {
        &self.graph
    }
}

impl<State> PRM<State>
where
    State: Copy + Proximity,
{
    fn build_k(
        k: usize,
        num_samples: usize,
        is_valid_state: impl Fn(&State) -> bool,
        is_valid_transition: impl Fn(&State, &State) -> bool,
        mut sample_state: impl FnMut() -> State,
    ) -> Self {
        let mut prm = Self::empty();

        for _ in 0..num_samples {
            let sample = sample_state();

            if is_valid_state(&sample) {
                prm.insert_knn(k, &is_valid_transition, sample);
            }
        }

        prm
    }

    pub fn insert_knn(
        &mut self,
        k: usize,
        is_valid_transition: impl Fn(&State, &State) -> bool,
        sample: State,
    ) -> NodeIndex<u32> {
        let graph_node = self.graph.add_node(sample);

        let lookup_node = StateWithNode {
            state: sample,
            node_id: graph_node,
        };

        self.connect_knn(k, is_valid_transition, &lookup_node);

        self.lookup_nearest.push(lookup_node);

        graph_node
    }

    fn connect_knn(
        &mut self,
        k: usize,
        is_valid_transition: impl Fn(&State, &State) -> bool,
        lookup_node: &StateWithNode<State>,
    ) {
        for neighbour in self.lookup_nearest.k_nearest(lookup_node, k) {
            if is_valid_transition(&lookup_node.state, &neighbour.item.state) {
                self.graph
                    .add_edge(lookup_node.node_id, neighbour.item.node_id, ());
            }
        }
    }

    fn with_temporary_node_knn<T>(
        &mut self,
        sample: State,
        k: usize,
        is_valid_transition: impl Fn(&State, &State) -> bool,
        f: impl FnOnce(&mut Self, NodeIndex) -> T,
    ) -> T {
        let node = self.graph.add_node(sample);

        let lookup_node = StateWithNode {
            state: sample,
            node_id: node,
        };

        self.connect_knn(k, is_valid_transition, &lookup_node);

        let t = f(self, node);

        self.graph.remove_node(node);

        t
    }

    fn empty() -> PRM<State> {
        Self {
            graph: UnGraph::new_undirected(),
            lookup_nearest: VpTree::new(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use acap::Proximity;
    use petgraph::prelude::EdgeRef;
    use rand::{thread_rng, Rng};

    #[test]
    fn prm_point_to_point() {
        #[derive(Copy, Clone, Debug)]
        struct SimpleState(f64);

        impl Proximity for SimpleState {
            type Distance = f64;

            fn distance(&self, other: &Self) -> Self::Distance {
                (self.0 - other.0).abs()
            }
        }

        let mut rng = thread_rng();

        let mut prm = PRM::build_k(
            10,
            100,
            |_| true,
            |_, _| true,
            || SimpleState(rng.gen_range(-100.0..100.0)),
        );

        for _ in 0..100 {
            let start = SimpleState(rng.gen_range(-100.0..100.0));
            let end = SimpleState(rng.gen_range(-100.0..100.0));

            let result = prm.with_temporary_node_knn(
                start,
                10,
                |_, _| true,
                |prm, start_id| {
                    prm.with_temporary_node_knn(
                        end,
                        10,
                        |_, _| true,
                        |prm, end_id| {
                            petgraph::algo::astar(
                                prm.peek_graph(),
                                start_id,
                                |g| g == end_id,
                                |e| {
                                    prm.peek_graph().node_weight(e.source()).unwrap().distance(
                                        &prm.peek_graph().node_weight(e.target()).unwrap(),
                                    )
                                },
                                |n| prm.peek_graph().node_weight(n).unwrap().distance(&end),
                            )
                        },
                    )
                },
            );

            assert!(result.is_some());
        }
    }
}
