# myAgents.py
# ---------------
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

from game import Agent
from searchProblems import PositionSearchProblem

import util
import time
import search

"""
IMPORTANT
`agent` defines which agent you will use. By default, it is set to ClosestDotAgent,
but when you're ready to test your own agent, replace it with MyAgent
"""
def createAgents(num_pacmen, agent='MyAgent'):
    return [eval(agent)(index=i) for i in range(num_pacmen)]

class MyAgent(Agent):

    def findMyPath(self, gameState):
        
        """
        initialize target area for each pacman
        match pacmans and areas which are close each other
        """

        if self.initial==0: #this part runs just one time

            distance = []
            reverse = []

            max = -9999
            min = 9999
            maxindex = 0
            minindex = 0
            for i in range(0, 4):

                initPosition = gameState.getPacmanPosition(i)
                distance.append(initPosition[0] + initPosition[1])
                reverse.append(initPosition[1] - initPosition[0])

                if distance[i]>max:
                    max = distance[i]
                    maxindex = i
                if distance[i]<min:
                    min = distance[i]
                    minindex = i
            self.target[maxindex] = 3
            self.target[minindex] = 1

            max = -9999
            min = 9999
            maxindex = 0
            minindex = 0
            for i in range(0,4):
                if self.target[i] == 0:
                    if reverse[i]>max:
                        max = reverse[i]
                        maxindex = i
                    if reverse[i]<min:
                        min = reverse[i]
                        minindex = i
            self.target[maxindex] = 4
            self.target[minindex] = 2

            self.initial = 1

        """
        make a list of foods which are in their target area
        """

        food = gameState.getFood().asList()
        targetfood = []

        h = gameState.getFood().height/2
        w = gameState.getFood().width/2

        if self.target[self.index]==1:
            for i,j in food:
                if i<w-1 and j<h-1:
                    targetfood.append((i,j))
        elif self.target[self.index]==2:
            for i,j in food:
                if i>=w-1 and j<h-1:
                    targetfood.append((i,j))
        elif self.target[self.index]==3:
            for i,j in food:
                if i>=w-1 and j>=h-1:
                    targetfood.append((i,j))
        else:
            for i,j in food:
                if i<w-1 and j>=h-1:
                    targetfood.append((i,j))

        """
        pacman which finished exploring its area helps others
        """

        if(len(targetfood))==0:

            targetfood = food
            self.target[self.index] = (self.target[self.index])%4 +1

        """
        exploring (based on ClosestDotAgent)
        """

        startPosition = gameState.getPacmanPosition(self.index)
        problem = AnyFoodSearchProblem(gameState, self.index)

        pacmanCurrent = [startPosition, [], 0]
        visitedPosition = set()
        fringe = util.PriorityQueue()
        fringe.push(pacmanCurrent, pacmanCurrent[2])

        while not fringe.isEmpty():
            pacmanCurrent = fringe.pop()
            if pacmanCurrent[0] in visitedPosition:
                continue
            else:
                visitedPosition.add(pacmanCurrent[0])
            if pacmanCurrent[0] in targetfood:
                return pacmanCurrent[1]
            else:
                pacmanSuccessors = problem.getSuccessors(pacmanCurrent[0])
            Successor = []
            for item in pacmanSuccessors:  # item: [(x,y), 'direction', cost]
                if item[0] not in visitedPosition:
                    pacmanRoute = pacmanCurrent[1].copy()
                    pacmanRoute.append(item[1])
                    sumCost = pacmanCurrent[2]

                    """
                    for the node which is not in the pacman's target area,
                    add extra cost for it to reduce the number of node expand
                    which is not neccesary

                    extra costs are based on pacman's x, y location
                    and this makes pacman to move toward it's target area
                    """

                    if self.target[self.index]==1:
                        if(item[0][0]>=w-1):
                            sumCost += item[0][0]*2
                        if(item[0][1]>=h-1):
                            sumCost += item[0][1]*2
                    elif self.target[self.index]==2:
                        if(item[0][0]<w-1):
                            sumCost += (50 - item[0][0])*2
                        if(item[0][1]>=h-1):
                            sumCost += item[0][1]*2
                    elif self.target[self.index]==3:
                        if(item[0][0]<w-1):
                            sumCost += (50 - item[0][0])*2
                        if(item[0][1]<h-1):
                            sumCost += (50 - item[0][1])*2
                    else:
                        if(item[0][0]>=w-1):
                            sumCost += item[0][0]*2
                        if(item[0][1]<h-1):
                            sumCost += (50 - item[0][1])*2

                    Successor.append([item[0], pacmanRoute, sumCost + item[2]])
            for item in Successor:
                fringe.push(item, item[2])

        return pacmanCurrent[1]

    def getAction(self, state):
        
        return self.findMyPath(state)[0]

    def initialize(self):
        """
        Intialize anything you want to here. This function is called
        when the agent is first created. If you don't need to use it, then
        leave it blank
        """

        "*** YOUR CODE HERE ***"

        self.initial = 0
        self.target = [0, 0, 0, 0]
        

"""
Put any other SearchProblems or search methods below. You may also import classes/methods in
search.py and searchProblems.py. (ClosestDotAgent as an example below)
"""

class ClosestDotAgent(Agent):

    def findPathToClosestDot(self, gameState):
        """
        Returns a path (a list of actions) to the closest dot, starting from
        gameState.
        """
        # Here are some useful elements of the startState
        startPosition = gameState.getPacmanPosition(self.index)
        food = gameState.getFood()
        walls = gameState.getWalls()
        problem = AnyFoodSearchProblem(gameState, self.index)


        "*** YOUR CODE HERE ***"

        pacmanCurrent = [problem.getStartState(), [], 0]
        visitedPosition = set()
        # visitedPosition.add(problem.getStartState())
        fringe = util.PriorityQueue()
        fringe.push(pacmanCurrent, pacmanCurrent[2])
        while not fringe.isEmpty():
            pacmanCurrent = fringe.pop()
            if pacmanCurrent[0] in visitedPosition:
                continue
            else:
                visitedPosition.add(pacmanCurrent[0])
            if problem.isGoalState(pacmanCurrent[0]):
                return pacmanCurrent[1]
            else:
                pacmanSuccessors = problem.getSuccessors(pacmanCurrent[0])
            Successor = []
            for item in pacmanSuccessors:  # item: [(x,y), 'direction', cost]
                if item[0] not in visitedPosition:
                    pacmanRoute = pacmanCurrent[1].copy()
                    pacmanRoute.append(item[1])
                    sumCost = pacmanCurrent[2]
                    Successor.append([item[0], pacmanRoute, sumCost + item[2]])
            for item in Successor:
                fringe.push(item, item[2])
        return pacmanCurrent[1]

    def getAction(self, state):
        return self.findPathToClosestDot(state)[0]

class AnyFoodSearchProblem(PositionSearchProblem):
    """
    A search problem for finding a path to any food.

    This search problem is just like the PositionSearchProblem, but has a
    different goal test, which you need to fill in below.  The state space and
    successor function do not need to be changed.

    The class definition above, AnyFoodSearchProblem(PositionSearchProblem),
    inherits the methods of the PositionSearchProblem.

    You can use this search problem to help you fill in the findPathToClosestDot
    method.
    """

    def __init__(self, gameState, agentIndex):
        "Stores information from the gameState.  You don't need to change this."
        # Store the food for later reference
        self.food = gameState.getFood()

        # Store info for the PositionSearchProblem (no need to change this)
        self.walls = gameState.getWalls()
        self.startState = gameState.getPacmanPosition(agentIndex)
        self.costFn = lambda x: 1
        self._visited, self._visitedlist, self._expanded = {}, [], 0 # DO NOT CHANGE

    def isGoalState(self, state):
        """
        The state is Pacman's position. Fill this in with a goal test that will
        complete the problem definition.
        """
        x,y = state
        if self.food[x][y] == True:
            return True
        return False
