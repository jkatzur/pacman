# search.py
# ---------
# Licensing Information: Please do not distribute or publish solutions to this
# project. You are free to use and extend these projects for educational
# purposes. The Pacman AI projects were developed at UC Berkeley, primarily by
# John DeNero (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# For more info, see http://inst.eecs.berkeley.edu/~cs188/sp09/pacman.html

"""
In search.py, you will implement generic search algorithms which are called
by Pacman agents (in searchAgents.py).
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
     Returns the start state for the search problem
     """
     util.raiseNotDefined()

  def isGoalState(self, state):
     """
       state: Search state

     Returns True if and only if the state is a valid goal state
     """
     util.raiseNotDefined()

  def getSuccessors(self, state):
     """
       state: Search state

     For a given state, this should return a list of triples,
     (successor, action, stepCost), where 'successor' is a
     successor to the current state, 'action' is the action
     required to get there, and 'stepCost' is the incremental
     cost of expanding to that successor
     """
     util.raiseNotDefined()

  def getCostOfActions(self, actions):
     """
      actions: A list of actions to take

     This method returns the total cost of a particular sequence of actions.  The sequence must
     be composed of legal moves
     """
     util.raiseNotDefined()


def tinyMazeSearch(problem):
  """
  Returns a sequence of moves that solves tinyMaze.  For any other
  maze, the sequence of moves will be incorrect, so only use this for tinyMaze
  """
  from game import Directions
  s = Directions.SOUTH
  w = Directions.WEST
  return  [s,s,w,s,w,w,s,w]

class Node:
    def __init__(self, state, action, cost, parent):
        self.state = state
        self.action = action
        self.cost = cost
        self.parent = parent


def path(curr_node):
    path = []
    print curr_node.state
    while curr_node.action:
        #print "Current state is: " + str(curr_node.state) + " and, Action is: " + str(curr_node.action)
        path.append(curr_node.action)
        curr_node = curr_node.parent
    path.reverse()
    return path

def depthFirstSearch(problem):
  """
  Search the deepest nodes in the search tree first
  [2nd Edition: p 75, 3rd Edition: p 87]

  Your search algorithm needs to return a list of actions that reaches
  the goal.  Make sure to implement a graph search algorithm
  [2nd Edition: Fig. 3.18, 3rd Edition: Fig 3.7].

  To get started, you might want to try some of these simple commands to
  understand the search problem that is being passed in:

  print "Start:", problem.getStartState()
  print "Is the start a goal?", problem.isGoalState(problem.getStartState())
  print "Start's successors:", problem.getSuccessors(problem.getStartState())
  """

  frontier = util.Queue()
  explored = []
  starting_node = Node(problem.getStartState(),False,0,False)
  frontier.push(starting_node)
  while not frontier.isEmpty():
    proc_node = frontier.pop()
    #print "Proc node is: " + str(proc_node.state)
    if problem.isGoalState(proc_node.state):
        #print "AT THE END"
        return path(proc_node)
    explored.append(proc_node.state)
    for next_loc in problem.getSuccessors(proc_node.state):
        #print "Next Loc is: " + str(next_loc[0])
        if next_loc[0] not in explored:
            frontier.push(Node(next_loc[0],next_loc[1],next_loc[2],proc_node))

def breadthFirstSearch(problem):
  """
  Search the shallowest nodes in the search tree first.
  [2nd Edition: p 73, 3rd Edition: p 82]
  """
  frontier = util.PriorityQueue()
  depth = util.Counter()
  explored = []
  starting_node = Node(problem.getStartState(),False,0,False)
  frontier.push(starting_node,depth[starting_node])
  while not frontier.isEmpty():
      proc_node = frontier.pop()
      #print "Proc node is: " + str(proc_node.state)
      if problem.isGoalState(proc_node.state):
          #print "AT THE END"
          return path(proc_node)
      explored.append(proc_node.state)
      for next_loc in problem.getSuccessors(proc_node.state):
          #print "Next Loc is: " + str(next_loc[0])
          if next_loc[0] not in explored:
              frontier.push(Node(next_loc[0],next_loc[1],next_loc[2],proc_node),depth[next_loc[0]]+1)


def uniformCostSearch(problem):
  "Search the node of least total cost first. "
  "*** YOUR CODE HERE ***"
  util.raiseNotDefined()

def nullHeuristic(state, problem=None):
  """
  A heuristic function estimates the cost from the current state to the nearest
  goal in the provided SearchProblem.  This heuristic is trivial.
  """
  return 0

def aStarSearch(problem, heuristic=nullHeuristic):
  "Search the node that has the lowest combined cost and heuristic first."
  "*** YOUR CODE HERE ***"
  util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
