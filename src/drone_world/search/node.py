import copy

class Node(object):
    def __init__(self, state, action, parent, node_count):
        """Initialize the node.
         Note that state must implement h() which is a heuristic function.
        """
        self.state = state
        self.action = action
        self.parent = parent
        self.fitness = state.h()
        self.node_count = node_count + 1

    def expand(self):
        """Expand the current node returning the neighbors of the node.
        """
        nodes = []
        for action in self.state.actions():
            state = copy.deepcopy(self.state)
            state.apply_action(action)
            nodes.append(Node(state, action, self, self.node_count))
        return nodes

    def get_actions(self):
        """Return a list of actions.
        """
        actions = []
        node = self
        while node.parent:
            actions.append(node.action)
            node = node.parent
        actions.reverse()
        return actions

    def is_goal_met(self):
        """Return true is the goal is met.
        Note that is_goal_met() must be defined by the state object.
        """
        return self.state.is_goal_met()

    def __eq__(self, other):
        """If the state of two nodes are the same, then the nodes are equal.
        Note that the state object probably wants to override the __eq__() operator.
        """
        if isinstance(other, Node):
            return self.state == other.state
        else:
            raise ValueError("Cannot compare Node to unknown object type")

    def __lt__(self, other):
        if isinstance(other, Node):
            return self.fitness < other.fitness
        else:
            raise ValueError("Cannot compare Node to unknown object type")

    def __gt__(self, other):
        if isinstance(other, Node):
            return self.fitness > other.fitness
        else:
            raise ValueError("Cannot compare Node to unknown object type")