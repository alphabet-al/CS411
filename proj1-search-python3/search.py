# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    """
    "*** YOUR CODE HERE ***"
    # visited contains nodes that are popped off of stack, Dictionary structure to quickly call
    # back information on the direction of travel to get to this state. 
    visited = {}
    # stack contains tuple of states to be expanded. tuple is in the form of 
    # (node, direction, cost). 
    fringe = util.Stack()
    # parent is an adjacency list used to determine child parent relationship and store
    # that information without having to expand nodes again
    parent = {}
    # moveset is a list containing the moves to get to the goal state from start state
    moveset = []
    # intitialize root_node in same format as expanded successor state information
    root_node = (problem.getStartState(), 'Start', 0)
    fringe.push(root_node)

    # traversal through graph structure starting at start node
    while not fringe.isEmpty():
        # every time loop starts we pop the next value off of the stack and evaluate
        # if it's the goal state, if so we set the node to goal and break out of the loop
        node = fringe.pop()
        if problem.isGoalState(node[0]):
            goal = node[0]
            visited[node[0]] = node[1]
            break
        # we check if the current node is in the visited list and if not we add to the list
        if node[0] not in visited:
            visited[node[0]] = node[1]
            # evaluate the child nodes of current node and add to stack if isn't in visited
            # we also store the parent-child node information in the parent dictionary
            for child_node in problem.getSuccessors(node[0]):
                if child_node[0] not in visited:
                    fringe.push(child_node)
                    parent[child_node[0]] = node[0]

    # We set current node to goal node and traverse backwards through parent-child dictionary.
    # This gives us the path from start to end and populates our moveset list.            
    curr = goal
    while (curr != root_node[0]):
        moveset.insert(0, visited[curr])
        curr = parent[curr]

    return moveset

    util.raiseNotDefined()

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    
    # visited contains nodes that are popped off of stack, Dictionary structure to quickly call
    # back information on the direction of travel to get to this state. 
    visited = {}
    # stack contains tuple of states to be expanded. tuple is in the form of 
    # (node, direction, cost). 
    fringe = util.Queue()
    # parent is an adjacency list used to determine child parent relationship and store
    # that information without having to expand nodes again
    parent = {}
    # moveset is a list containing the moves to get to the goal state from start state
    moveset = []
    # intitialize root_node in same format as expanded successor state information
    root_node = (problem.getStartState(), 'Start', 0)
    fringe.push(root_node)
    # list to check if child nodes are currently in the fringe.  If so, we don't explore any further.
    is_on_queue = []
    goal = 'False'

    # traversal through graph structure starting at start node
    while not fringe.isEmpty():
        # every time loop starts we pop the next value off of the stack and evaluate
        # if it's the goal state, if so we set the node to goal and break out of the loop
        node = fringe.pop()
        if problem.isGoalState(node[0]):
            goal = node[0]
            visited[node[0]] = node[1]
            break
        # we check if the current node is in the visited list and if not we add to the list
        if node[0] not in visited:
            visited[node[0]] = node[1]
            # evaluate the child nodes of current node and add to stack if isn't in visited
            # we also store the parent-child node information in the parent dictionary
            for child_node in problem.getSuccessors(node[0]):
                if child_node[0] not in visited and child_node[0] not in is_on_queue:
                    fringe.push(child_node)
                    parent[child_node[0]] = node[0]
                    is_on_queue.append(child_node[0])

    # We set current node to goal node and traverse backwards through parent-child dictionary.
    # This gives us the path from start to end and populates our moveset list.            
    curr = goal
    while (curr != root_node[0]):
        moveset.insert(0, visited[curr])
        curr = parent[curr]

    return moveset

    util.raiseNotDefined()

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    
    from util import Counter
    
    # visited contains nodes that are popped off of stack, Dictionary structure to quickly call
    # back information on the direction of travel to get to this state. 
    visited = {}
    # stack contains tuple of states to be expanded. tuple is in the form of 
    # (node, direction, cost). 
    fringe = util.PriorityQueue()
    # parent is an adjacency list used to determine child parent relationship and store
    # that information without having to expand nodes again
    parent = {}
    # moveset is a list containing the moves to get to the goal state from start state
    moveset = []    
    # instantiate Counter object to help keep track of node cost.
    node_cost = Counter()
    # intitialize root_node in same format as expanded successor state information
    root_node = (problem.getStartState(), 'Start', 0)
    node_cost[root_node[0]] = root_node[2]
    fringe.push(root_node, 0)
    # list to check if child nodes are currently in the fringe.  If so, we don't explore any further. 
    is_on_queue = []                                                                                  



    # traversal through graph structure starting at start node
    while not fringe.isEmpty():
        # every time loop starts we pop the next value off of the stack and evaluate
        # if it's the goal state, if so we set the node to goal and break out of the loop
        node = fringe.pop()
        if problem.isGoalState(node[0]):
            goal = node[0]
            visited[node[0]] = node[1]
            break
        # we check if the current node is in the visited list and if not we add to the list
        if node[0] not in visited:
            visited[node[0]] = node[1]
            # evaluate the child nodes of current node and add to stack if isn't in visited
            # we also store the parent-child node information in the parent dictionary
            for child_node in problem.getSuccessors(node[0]):
                # check if child node is in visited and if it has been on the queue.
                if child_node[0] not in visited and child_node[0] not in is_on_queue: 
                    node_cost[child_node[0]] = node_cost[node[0]] + child_node[2]
                    fringe.update(child_node,node_cost[child_node[0]])
                    parent[child_node[0]] = node[0]
                    is_on_queue.append(child_node[0])
                # check if the child node is in visited and if it is on the queue.  we update compare 
                # update if the priority is higher
                if child_node[0] not in visited and child_node[0] in is_on_queue: 
                    if node_cost[child_node[0]] > node_cost[node[0]] + child_node[2]:
                        node_cost[child_node[0]] = node_cost[node[0]] + child_node[2]
                        fringe.update(child_node,node_cost[child_node[0]])
                        parent[child_node[0]] = node[0]
                        is_on_queue.append(child_node[0])

    # We set current node to goal node and traverse backwards through parent-child dictionary.
    # This gives us the path from start to end and populates our moveset list.            
    curr = goal
    while (curr != root_node[0]):
        moveset.insert(0, visited[curr])
        curr = parent[curr]

    return moveset
    util.raiseNotDefined()

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"

    from util import Counter
    
    # visited contains nodes that are popped off of stack, Dictionary structure to quickly call
    # back information on the direction of travel to get to this state. 
    visited = {}
    # stack contains tuple of states to be expanded. tuple is in the form of 
    # (node, direction, cost). 
    fringe = util.PriorityQueue()
    # parent is an adjacency list used to determine child parent relationship and store
    # that information without having to expand nodes again
    parent = {}
    # moveset is a list containing the moves to get to the goal state from start state
    moveset = []    
    # instantiate Counter object to help keep track of node cost.
    g_cost = Counter()
    f_cost = Counter()
    # intitialize root_node in same format as expanded successor state information
    root_node = (problem.getStartState(), 'Start', 0)
    g_cost[root_node[0]] = root_node[2]
    f_cost[root_node[0]] = g_cost[root_node[0]] + heuristic(root_node[0], problem)
    fringe.push(root_node, 0)
    # list to check if child nodes are currently in the fringe.  If so, we don't explore any further. 
    is_on_queue = []                                                                             



    # traversal through graph structure starting at start node
    while not fringe.isEmpty():
        # every time loop starts we pop the next value off of the stack and evaluate
        # if it's the goal state, if so we set the node to goal and break out of the loop
        node = fringe.pop()
        if problem.isGoalState(node[0]):
            goal = node[0]
            visited[node[0]] = node[1]
            break
        # we check if the current node is in the visited list and if not we add to the list
        if node[0] not in visited:
            visited[node[0]] = node[1]
            # evaluate the child nodes of current node and add to stack if isn't in visited
            # we also store the parent-child node information in the parent dictionary
            for child_node in problem.getSuccessors(node[0]):
                # check if child node is in visited and if it has been on the queue.
                if child_node[0] not in visited and child_node[0] not in is_on_queue: 
                    g_cost[child_node[0]] = g_cost[node[0]] + child_node[2]
                    f_cost[child_node[0]] = g_cost[child_node[0]] + heuristic(child_node[0], problem)
                    fringe.update(child_node,f_cost[child_node[0]])
                    parent[child_node[0]] = node[0]
                    is_on_queue.append(child_node[0])
                # check if the child node is in visited and if it is on the queue.  we update compare 
                # update if the priority is higher
                if child_node[0] not in visited and child_node[0] in is_on_queue: 
                    if f_cost[child_node[0]] > g_cost[node[0]] + child_node[2] + heuristic(child_node[0], problem):
                        f_cost[child_node[0]] = g_cost[node[0]] + child_node[2] + heuristic(child_node[0], problem)
                        fringe.update(child_node,f_cost[child_node[0]])
                        parent[child_node[0]] = node[0]
                        is_on_queue.append(child_node[0])

    # We set current node to goal node and traverse backwards through parent-child dictionary.
    # This gives us the path from start to end and populates our moveset list.            
    curr = goal
    while (curr != root_node[0]):
        moveset.insert(0, visited[curr])
        curr = parent[curr]

    return moveset

    util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
