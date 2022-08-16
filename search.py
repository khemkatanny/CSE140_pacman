"""
In this file, you will implement generic search algorithms which are called by Pacman agents.
"""
from pacai.util.priorityQueue import PriorityQueue

def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first [p 85].

    Your search algorithm needs to return a list of actions that reaches the goal.
    Make sure to implement a graph search algorithm [Fig. 3.7].

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:
    ```
    print("Start: %s" % (str(problem.startingState())))
    print("Is the start a goal?: %s" % (problem.isGoal(problem.startingState())))
    print("Start's successors: %s" % (problem.successorStates(problem.startingState())))
    ```
    """

    # *** Your Code Here ***
    # raise NotImplementedError()
    # print("Start: %s" % (str(problem.startingState())))
    # print("Is the start a goal?: %s" % (problem.isGoal(problem.startingState())))
    # print("Start's successors: %s" % (problem.successorStates(problem.startingState())))

    # saving the intitial position
    start_path = []
    # to keep track of visited nodes
    visited = [problem.startingState()]
    # LIFO
    fringe = []
    # to save the entire path from start to goal
    path = []

    for node in problem.successorStates(problem.startingState()):
        fringe.append(node)

    while len(fringe) > 0:
        node = fringe[-1]  # saving last state
        if node[0] in visited:
            fringe.pop()
            if node == start_path[-1]:
                start_path.pop()
        else:
            start_path.append(node)
            visited.append(node[0])
            if (problem.isGoal(node[0])):  # to check if we are reaching goal state
                for _, direction, _ in start_path:
                    path.append(direction)
                return path
            neighbor = problem.successorStates(node[0])  # to save successor state
            for next_path in neighbor:
                if (next_path[0]) not in visited:
                    fringe.append(next_path)
    raise NotImplementedError()


def breadthFirstSearch(problem):
    """
    Search the shallowest nodes in the search tree first. [p 81]
    """
    # *** Your Code Here ***
    # to keep track of visited nodes
    visited = [problem.startingState()]
    # FIFO
    fringe = []

    for node in problem.successorStates(problem.startingState()):
        fringe.append((node, []))

    while len(fringe) > 0:
        # saving last state
        node, prev_node = fringe.pop()
        if node[0] not in visited:
            visited.append(node[0])
            # to check if we are reaching goal state
            if problem.isGoal(node[0]):
                return prev_node + [node[1]]
                # to save successor state
            neighbor = problem.successorStates(node[0])
            for next_path in neighbor:
                if (next_path[0]) not in visited:
                    fringe = [(next_path, prev_node + [node[1]])] + fringe
    raise NotImplementedError()

def uniformCostSearch(problem):
    """
    Search the node of least total cost first.
    """
    # *** Your Code Here ***
    # using priority queue to store initial state
    fringe = PriorityQueue()
    # saving initial position
    start_node = problem.startingState()
    next_node = (start_node, 0, [])
    # need to store state, cost, direction, previous state,
    # prev direction
    fringe.push(next_node, 0)
    visited = []
    while len(fringe) > 0:
        # saving state with smallest priority value
        nodes = fringe.pop()
        if nodes[0] not in visited:
            # check is goal is reached
            if problem.isGoal(nodes[0]):
                return nodes[2]
            visited.append(nodes[0])
            # to keep track of successor's state, cost
            neighbor = problem.successorStates(nodes[0])
            for node in neighbor:
                if node[0] not in visited:
                    fringe.push((node[0], nodes[1] + node[2],
                        nodes[2] + [node[1]]), nodes[1] + node[2])
    raise NotImplementedError()


def aStarSearch(problem, heuristic):
    """
    Search the node that has the lowest combined cost and heuristic first.
    """

    # *** Your Code Here ***
    # using priority queue to store initial state
    fringe = PriorityQueue()
    # saving initial position
    start_node = problem.startingState()
    next_node = (start_node, 0, [])
    # need to store state, cost, direction,
    # previous state, prev direction
    fringe.push(next_node, 0)
    visited = []
    while len(fringe) > 0:
        # saving state with smallest priority value
        nodes = fringe.pop()
        if nodes[0] not in visited:
            # check is goal is reached
            if problem.isGoal(nodes[0]):
                return nodes[2]
        visited.append(nodes[0])
        neighbor = problem.successorStates(nodes[0])  # to keep track of successor's state, cost
        for node in neighbor:
            if node[0] not in visited:
                h_n = heuristic(node[0], problem)
                fringe.push((node[0], nodes[1] + node[2], nodes[2] + [node[1]]),
                nodes[1] + node[2] + h_n)
    raise NotImplementedError()
