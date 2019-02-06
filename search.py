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

def depthFirstSearch(problem):
  """
  Search the deepest nodes in the search tree first [p 74].
  
  Your search algorithm needs to return a list of actions that reaches
  the goal.  Make sure to implement a graph search algorithm [Fig. 3.18].
  
  To get started, you might want to try some of these simple commands to
  understand the search problem that is being passed in:
  
  print "Start:", problem.getStartState()
  print "Is the start a goal?", problem.isGoalState(problem.getStartState())
  print "Start's successors:", problem.getSuccessors(problem.getStartState())
  """
  "*** YOUR CODE HERE ***"
  stack = util.Stack()
  visited_nodes =[]
  paths = []
  current_node=[]
  # add start state to Q
  parent_node = problem.getStartState()
  stack.push((parent_node, paths))
  
  while stack:
      current_node, paths = stack.pop()
        #print("currstate?",problem.isGoalState(cur_state))
      child_nodes = []
      direction ={} 
      if current_node in visited_nodes:
          continue

      elif problem.isGoalState(current_node):
          return paths

      else:
        visited_nodes = visited_nodes  + [current_node]

      
      for element in problem.getSuccessors(current_node):
        child_nodes.append(element[0])
        direction[element[0]] = element[1]
          

      for node in child_nodes:
          if node not in visited_nodes:
              stack.push((node, paths + [direction[node]]))

  return paths
  util.raiseNotDefined()
  


def breadthFirstSearch(problem):
  "Search the shallowest nodes in the search tree first. [p 74]"
  "*** YOUR CODE HERE ***"
  queue = util.Queue()
  visited_nodes =[]
  paths = []
  current_node=[]
  # add start state to Q
  parent_node = problem.getStartState()
  queue.push((parent_node, paths))
  
  while queue:
      current_node, paths = queue.pop()
        #print("currstate?",problem.isGoalState(cur_state))
      child_nodes = []
      direction ={} 
      if current_node in visited_nodes:
          continue

      elif problem.isGoalState(current_node):
          return paths

      else:
        visited_nodes = visited_nodes  + [current_node]

      
      for element in problem.getSuccessors(current_node):
        child_nodes.append(element[0])
        direction[element[0]] = element[1]
          

      for node in child_nodes:
          if node not in visited_nodes:
              queue.push((node, paths + [direction[node]]))

  return paths
  util.raiseNotDefined()
      
def uniformCostSearch(problem):
  "Search the node of least total cost first."
  '''
  this uniformCostSearch algorithm is the same as A* search algorithm,
  but in this project, A* search algorithm need cost estimation
  '''
  "*** YOUR CODE HERE ***"
  priorityQ = util.PriorityQueue()
  visited_nodes =[]
  paths = []
  current_node=[]
  cost=0
  # add start state to Q
  parent_node = problem.getStartState()
  priorityQ.push((parent_node,paths),cost)
  
  while priorityQ:
      current_node, paths = priorityQ.pop()
        #print("currstate?",problem.isGoalState(cur_state))
      child_nodes = []
      direction ={}
      if current_node in visited_nodes:
          continue

      elif problem.isGoalState(current_node):
          return paths

      else:
        visited_nodes = visited_nodes  + [current_node]

      
      for element in problem.getSuccessors(current_node):
        child_nodes.append(element[0])
        direction[element[0]] = element[1]         
      for node in child_nodes:
          if node not in visited_nodes:
              priorityQ.push((node, paths + [direction[node]]),problem.getCostOfActions(paths +[direction[node]]))
  return paths
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
  priorityQ = util.PriorityQueue()
  visited_nodes =[]
  paths = []
  current_node=[]
  # add start state to Q
  parent_node = problem.getStartState()
  priorityQ.push((parent_node,paths),nullHeuristic(parent_node, problem))
  
  while priorityQ:
      current_node, paths = priorityQ.pop()
        #print("currstate?",problem.isGoalState(cur_state))
      child_nodes = []
      direction ={}
      if current_node in visited_nodes:
          continue

      elif problem.isGoalState(current_node):
          return paths

      else:
        visited_nodes = visited_nodes  + [current_node]

      
      for element in problem.getSuccessors(current_node):
        child_nodes.append(element[0])
        direction[element[0]] = element[1]         
      for node in child_nodes:
          if node not in visited_nodes:
              priorityQ.push((node, paths + [direction[node]]),problem.getCostOfActions(paths +[direction[node]]) + heuristic(node, problem))
  return paths
  util.raiseNotDefined()
  
# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
