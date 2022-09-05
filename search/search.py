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

from tkinter import W
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
    #Need to return a list of all the actions taken to reach the goal (2nd part of the tuple)
    fringe = util.Stack()   #Fringe, contains the node tuple + I could add the previous node's action into a list, thus when the goal is found simply return that list
    visited = {problem.getStartState(): False}
    fringe.push([problem.getStartState(), []])

    while (not fringe.isEmpty()):
        #Get the next state we should explore
        currSuccessor = fringe.pop()
        currState = currSuccessor[0] 
        actions = currSuccessor[1] #List of all the prev actions leading to this node
        visited[currState] = True  #Set visited True for this node
        
        #Check if it's the goal
        if (problem.isGoalState(currState)):
            return actions
        
        #If not the goal, expand the current node and add the possibilities to the search graph
        for state in problem.getSuccessors(currState):
            coord = state[0]
            action = state[1]
            if (coord not in visited):
                fringe.push([coord, actions + [action]])
                visited[coord] = False
            elif (visited[coord] == False):
                fringe.push([coord, actions + [action]])

    #Failure, no possible goal
    return None   

def breadthFirstSearch(problem: SearchProblem):
    """Search the shallowest nodes in the search tree first."""
    #Very similar logic to DFS, except the fringe must be a Queue (FIFO) instead of a Stack
    fringe = util.Queue()  
    visited = {problem.getStartState()}
    fringe.push([problem.getStartState(), []])

    while (not fringe.isEmpty()):
        #Get the next state we should explore
        currSuccessor = fringe.pop()
        currState = currSuccessor[0] 
        actions = currSuccessor[1] #List of all the prev actions leading to this node
        
        #Check if it's the goal
        if (problem.isGoalState(currState)):
            return actions
        
        #If not the goal, expand the current node and add the possibilities to the search graph
        for state in problem.getSuccessors(currState):
            coord = state[0]
            action = state[1]
            if (coord not in visited):
                fringe.push([coord, actions + [action]])
                visited.add(coord)

    #Failure, no possible goal
    return None

def uniformCostSearch(problem: SearchProblem):
    """Search the node of least total cost first."""
    #Again, very similar logic to DFS and BFS but with the fringe being a PQ with cumulative cost as the priority
    fringe = util.PriorityQueue()
    visited = {problem.getStartState()}
    fringe.push((problem.getStartState(), [], 0), 0)
    priority = {problem.getStartState(): 0}

    while (not fringe.isEmpty()):
        #Get the next state we should explore
        currSuccessor = fringe.pop()
        currState = currSuccessor[0] 
        actions = currSuccessor[1] #List of all the prev actions leading to this node
        cumCost = currSuccessor[2] #Cumulative cost of the current path

        #Check if it's the goal
        if (problem.isGoalState(currState)):
            return actions

        #If not the goal, expand the current node and add the possibilities to the search tree
        for successor in problem.getSuccessors(currState):
            #parse successor
            state = successor[0]
            action = actions + [successor[1]]
            cost = cumCost + successor[2] 

            if (state not in visited):
                fringe.push([state, action, cost], cost)
                visited.add(state)
                priority[state] = cost
            elif (priority[state] > cost):
                fringe.update([state, action, cost], cost)
                priority[state] = cost

    #Failure, no possible goal
    return None

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem: SearchProblem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    #We need to sum the cumCost (greedy) and the heuristic value of a node, and that will be our priority
    fringe = util.PriorityQueue()
    visited = {problem.getStartState()}
    hN = heuristic(problem.getStartState(), problem)
    fringe.push((problem.getStartState(), [], 0), hN)
    priority = {problem.getStartState(): hN}

    while (not fringe.isEmpty()):
        #Get the next state we should explore
        currSuccessor = fringe.pop()
        currState = currSuccessor[0] 
        actions = currSuccessor[1] #List of all the prev actions leading to this node
        cumCost = currSuccessor[2] #gN (cumCost)

        #Check if it's the goal
        if (problem.isGoalState(currState)):
            return actions

        #If not the goal, expand the current node and add the possibilities to the search tree
        for successor in problem.getSuccessors(currState):
            #parse successor
            state = successor[0]
            action = actions + [successor[1]]
            cost = cumCost + successor[2]

            if (state not in visited):
                fringe.push([state, action, cost], cost + heuristic(state, problem))
                visited.add(state)
                priority[state] = cost
            elif (priority[state] > cost + heuristic(state, problem)):
                fringe.update([state, action, cost], cost + heuristic(state, problem))
                priority[state] = cost + heuristic(state, problem)

    #Failure, no possible goal
    return None

# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
