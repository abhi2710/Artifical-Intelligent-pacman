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


#################################################
#################### DFS ########################
#################################################

def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())
    """
    root=problem.getStartState()
    stack=util.Stack()                                      #DFS- fringe list - stack (LIFO)
    stack.push([root,[]])
    visited=set()
    while not stack.isEmpty():
        currentNode,path=stack.pop()
        if currentNode in visited:
            continue                                         #if already visited, ignore
        visited.add(currentNode)
        if problem.isGoalState(currentNode):
            return path                                      #goal state reached
        successors = problem.getSuccessors(currentNode)
        for successor in successors:
            if successor[0] not in visited:
                stack.push([successor[0],path+[successor[1]]])
    return []










#################################################
#################### BFS ########################
#################################################

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""

    root=problem.getStartState()
    queue = util.Queue()                                            #BFS- fringe list - queue (FIFO)
    queue.push([root,[]])
    visited=set()
    while not queue.isEmpty():
        currentState,path=queue.pop()
        if currentState in visited:
            continue                                                #If already visited, ignore
        visited.add(currentState)
        if problem.isGoalState(currentState):
            return path                                             #goal state reached
        successors = problem.getSuccessors(currentState)
        for successor in successors:
            if successor[0] not in visited:
                queue.push([successor[0],path+[successor[1]]])
    return []











#################################################
#################### UCS ########################
#################################################

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    root=problem.getStartState()
    priorityQueue = util.PriorityQueue()                            #UCS- fringe list - Priority queue
    priorityQueue.push([root,[],0],0)
    visited=set([])
    while priorityQueue:
        currentNode,path,cost=priorityQueue.pop()
        currentPosition=currentNode
        if currentPosition in visited:
            continue                                                #if already visited, ignore
        visited.add(currentPosition)
        if problem.isGoalState(currentPosition):
            return path                                             #Goal state reached
        successors = problem.getSuccessors(currentPosition)
        for successor in successors:
            succKey = successor[0]
            if succKey not in visited:
                newcost= cost+ successor[2]                        #cost of reaching till here + cost of new action
                priorityQueue.push([successor[0],path+[successor[1]],newcost],newcost)
    return []






def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0







#################################################
#################### A* ########################
#################################################

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    root=problem.getStartState()
    priorityQueue = util.PriorityQueue()                            #A*- fringe list - priority queue
    priorityQueue.push([root,[],0],0)
    visited=set([])
    while priorityQueue:
        currentNode,path,cost=priorityQueue.pop()
        currentPosition=currentNode
        if currentPosition in visited:
            continue                                                #If already visited, ignore
        visited.add(currentPosition)
        if problem.isGoalState(currentPosition):
            return path                                             #Goal state reached
        successors = problem.getSuccessors(currentPosition)
        for successor in successors:
            succKey = successor[0]
            if succKey not in visited:
                fCost =cost+successor[2]                            #f(n) = Cost of reaching till here + cost of next action
                hCost=heuristic(successor[0], problem)              #h(n) = Cost of heuristic for next action
                totalCost=fCost+hCost                               #total cost = f(n) + h(n)
                priorityQueue.push([successor[0],path+[successor[1]],fCost],totalCost)
    return []




# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
