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
    from util import Stack
    from game import Directions

    st = Stack() #make a stack for search
    st.push(((problem.getStartState()), [])) #initailize

    dfspath = []
    visitedstates = [] #store nodes(states) which have visited before

    while(not st.isEmpty()):

        (curstate, curpath) = st.pop() #get an item at the top of the stack

        if(problem.isGoalState(curstate)):
            dfspath = curpath #break and return the path when we reach to the goal node
            break
        
        elif(curstate not in visitedstates): #explore current node if the node is not visited
            visitedstates.append(curstate)
            sucs = problem.getSuccessors(curstate) #get the list of information include next node, action and cost
            for (nextstate, nextact, nextcost) in sucs:
                if(nextstate not in visitedstates): #add each node to stack if it is not visited yet
                    nextpath = curpath + [nextact]
                    st.push((nextstate, nextpath))

    return dfspath
    util.raiseNotDefined()

    # *****The code of below searches are simmilar as this code, so I will skip the explanation of same parts*****

def breadthFirstSearch(problem):
    from util import Queue
    from game import Directions

    qu = Queue() #make a queue for search, and this is the only difference between DFS and BFS
    qu.push(((problem.getStartState()), []))

    bfspath = []
    visitedstates = []

    while(not qu.isEmpty()):

        (curstate, curpath) = qu.pop()

        if(problem.isGoalState(curstate)):
            bfspath = curpath
            break
        
        elif(curstate not in visitedstates):
            visitedstates.append(curstate)
            sucs = problem.getSuccessors(curstate)
            for (nextstate, nextact, nextcost) in sucs:
                if(nextstate not in visitedstates):
                    nextpath = curpath + [nextact]
                    qu.push((nextstate, nextpath))

    return bfspath
    util.raiseNotDefined()

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    from util import PriorityQueue
    from game import Directions

    pq = PriorityQueue() #make a priority queue for search
    pq.push(((problem.getStartState()), []), 0) #initailize with start state and priority 0

    ucspath = []
    visitedstates = []

    while(not pq.isEmpty()):

        (curstate, curpath) = pq.pop() #get an item from the top of the heap

        if(problem.isGoalState(curstate)):
            ucspath = curpath
            break
        
        elif(curstate not in visitedstates):
            visitedstates.append(curstate)
            sucs = problem.getSuccessors(curstate)
            for (nextstate, nextact, nextcost) in sucs:
                if(nextstate not in visitedstates):
                    nextpath = curpath + [nextact]
                    pq.update((nextstate, nextpath), problem.getCostOfActions(nextpath))
                    #carculate the cost from start state to next state. We use this cost for the priority
                    #update the priority queue with next item and the priority

    return ucspath
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
    from util import PriorityQueue
    from game import Directions

    apq = PriorityQueue()
    apq.push(((problem.getStartState()), []), 0)

    asspath = []
    visitedstates = []

    while(not apq.isEmpty()):

        (curstate, curpath) = apq.pop()

        if(problem.isGoalState(curstate)):
            asspath = curpath
            break
        
        elif(curstate not in visitedstates):
            visitedstates.append(curstate)
            sucs = problem.getSuccessors(curstate)
            for (nextstate, nextact, nextcost) in sucs:
                if(nextstate not in visitedstates):
                    nextpath = curpath + [nextact]
                    apq.update((nextstate, nextpath), problem.getCostOfActions(nextpath) + heuristic(nextstate, problem))
                    #carculate the cost from start state to next state
                    #carculate the expected cost from next state to the goal state with heuristic function
                    #add this two costs and use it for the priority
                    #update the priority queue with next item and the priority
                    #method of carculating priority is the only difference between UCS and A*S

    return asspath
    util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
