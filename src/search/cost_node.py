import copy

class CostNode(object):
    def __init__(self, state, action, parent, total_cost):
        """Initialize the node.
         Note that state must implement h() which is a heuristic function.
        """
        self.state = state
        self.action = action
        self.parent = parent
        self.local_cost = state.h()
        self.total_cost = total_cost + self.local_cost

    def expand(self):
        """Expand the current node returning the neighbors of the node.
        """
        nodes = []
        for action in self.state.actions():
            state = copy.deepcopy(self.state)
            state.apply_action(action)
            nodes.append(CostNode(state, action, self, self.total_cost))
        return nodes

    def is_goal_met(self):
        """Forward message onto the state object.
        Note that state must implement is_goal_meet() which returns a boolean.
        """
        return self.state.is_goal_met()

    def __eq__(self, other):
        """If the state of two nodes are the same, then the nodes are equal.
        Note that the state object probably wants to override the __eq__() operator.
        """
        if isinstance(other, CostNode):
            return self.state == other.state
        else:
            raise ValueError("Cannot compare CostNode to unknown object type")

    def __lt__(self, other):
        if isinstance(other, CostNode):
            return self.local_cost < other.local_cost
        else:
            raise ValueError("Cannot compare CostNode to unknown object type")