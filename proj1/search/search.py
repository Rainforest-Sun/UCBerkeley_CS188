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

def depthFirstSearch(problem: SearchProblem):
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
    # print("Start:", problem.getStartState())
    # print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    # print("Start's successors:", problem.getSuccessors((1,1)))
    s_path = util.Stack()
    s_action = util.Stack()
    s_leaf = util.Stack()
    last_action = util.Stack()
    s_succs = util.Stack()
    s_leaf.push(problem.getStartState())
    while (not s_leaf.isEmpty()):
        leaf = s_leaf.pop()
        s_path.push(leaf)
        if not last_action.isEmpty():
            s_action.push(last_action.pop())
        if problem.isGoalState(leaf) == True:
            return s_action.list
        tuples = problem.getSuccessors(leaf)
        s_succs.push(tuples)
        flg = False
        for tp in tuples:
            if tp[0] not in s_path.list:
                s_leaf.push(tp[0])
                last_action.push(tp[1])
                flg = True
        if flg == False:
            s_path.pop()
            s_succs.pop()
            if not s_action.isEmpty():
                s_action.pop()
            while (True):
                succs = s_succs.list[-1]
                succ_states = [item[0] for item in succs]
                if s_leaf.list[-1] in succ_states:
                    break
                else:
                    s_path.pop()
                    s_succs.pop()
                    if not s_action.isEmpty():
                        s_action.pop()
    return s_action.list


def breadthFirstSearch(problem: SearchProblem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    s_visited = []
    s_leaf = util.Queue()
    s_action = util.Queue()
    
    start_state = problem.getStartState()
    s_leaf.push(start_state)
    s_action.push([])
    s_visited.append(start_state)

    while (not s_leaf.isEmpty()):
        leaf = s_leaf.pop()
        # s_visited.append(leaf)
        action = s_action.pop()
        # print((leaf,action))
        if (problem.isGoalState(leaf)):
            # action.reverse()
            # print(action)
            return action
        tuples = problem.getSuccessors(leaf)
        for tp in tuples:
            if tp[0] not in s_visited:
                s_leaf.push(tp[0])
                s_visited.append(tp[0])
                new_action = action[:]
                new_action.append(tp[1])
                # print("new action")
                # print(new_action)
                # print(action)
                s_action.push(new_action)

    return action
    

def uniformCostSearch(problem: SearchProblem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    pq = util.PriorityQueue()
    start_state = problem.getStartState()
    pq.push((start_state, [], 0), 0)
    costs = {}
    costs[start_state] = 0
    while not pq.isEmpty():
        pos, action, cost = pq.pop()
        # costs[pos] = cost
        if problem.isGoalState(pos):
            return action
        tuples = problem.getSuccessors(pos)
        for tp in tuples:
            if (tp[0] in costs.keys()) and (tp[2] + cost >= costs[tp[0]]):
                pass
            else:
                costs[tp[0]] = tp[2] + cost
                new_action = action[:]
                new_action.append(tp[1])
                pq.update((tp[0], new_action, tp[2] + cost), tp[2] + cost)


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem: SearchProblem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    # pq = util.PriorityQueue()
    # start_state = problem.getStartState()
    # start_cost = heuristic(start_state, problem)
    # pq.push((start_state, [], 0), start_cost)
    # costs = {}
    # costs[start_state] = start_cost
    # while not pq.isEmpty():
    #     pos, action, cost = pq.pop()
    #     # costs[pos] = cost
    #     if problem.isGoalState(pos):
    #         return action
    #     tuples = problem.getSuccessors(pos)
    #     for tp in tuples:
    #         new_cost = heuristic(tp[0], problem) + tp[2] + cost
    #         if (tp[0] in costs.keys()) and (costs[tp[0]] <= new_cost):
    #             pass
    #         else:
    #             costs[tp[0]] = new_cost
    #             new_action = action[:]
    #             new_action.append(tp[1])
    #             pq.update((tp[0], new_action, cost + tp[2]), new_cost)
    pq = util.PriorityQueue()
    start_state = problem.getStartState()
    start_cost = heuristic(start_state, problem)
    pq.push((start_state, [], 0), start_cost)
    costs = {}
    costs[start_state] = start_cost
    while not pq.isEmpty():
        pos, action, cost = pq.pop()
        # print(cost)
        # costs[pos] = cost
        if problem.isGoalState(pos):
            return action
        tuples = problem.getSuccessors(pos)
        for tp in tuples:
            new_cost = heuristic(tp[0], problem) + tp[2] + cost
            if (tp[0] in costs.keys()) and (costs[tp[0]] <= new_cost):
                pass
            else:
                costs[tp[0]] = new_cost
                new_action = action[:]
                new_action.append(tp[1])
                pq.update((tp[0], new_action, cost + tp[2]), new_cost)


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
