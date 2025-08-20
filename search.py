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
from game import Directions
from typing import List

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


def genericSearch(problem, fronteira, strategy)-> List[Directions]:
    best_costs = {}
    nos_visited = set()
    initial_state = (problem.getStartState(), [], 0)
    strategy(fronteira, initial_state, 0, problem)
    best_costs[problem.getStartState()] = 0

    while not fronteira.isEmpty():
        # (state, cost, path) = fronteira.pop()
        state, path, cost = fronteira.pop()

        # Verificar se ainda é o melhor caminho (para buscas de custo uniforme)
        if hasattr(strategy, 'recheck') and strategy.recheck:
            if cost != best_costs.get(state, float('inf')):
                continue
        
        if problem.isGoalState(state):
            return path
        
            
            # Para buscas de custo uniforme, verificar se é melhor
        if hasattr(strategy, 'recheck') and strategy.recheck:
            for next_state, action, step_cost in problem.getSuccessors(state):
                new_cost = cost + step_cost
                new_path = path + [action]
                new_state = (next_state, new_path, new_cost)

                if next_state not in best_costs or new_cost < best_costs[next_state]:
                    best_costs[next_state] = new_cost
                    strategy(fronteira, new_state, new_cost, problem)
        else:
            # Para DFS/BFS, simplesmente adicionar se não visitado
            if not state in nos_visited:
                nos_visited.add(state)
                for filhoNo, filhoAction, filhoCusto in problem.getSuccessors(state):
                    newCusto = cost + filhoCusto
                    newCaminho = path + [filhoAction]
                    newState = (filhoNo, newCaminho, newCusto)
                    strategy(fronteira, newState, newCusto, problem)

    return []
    

def tinyMazeSearch(problem: SearchProblem) -> List[Directions]:
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem: SearchProblem) -> List[Directions]:
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

    nosAtivos = util.Stack()

    def estragegiaProfundidade(frontier, state, cost, problem):
        frontier.push(state)

    estragegiaProfundidade.recheck = False

    return genericSearch(problem, nosAtivos, estragegiaProfundidade)

def breadthFirstSearch(problem: SearchProblem) -> List[Directions]:
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    nosAtivos = util.Queue()

    def estrategiaLargura(frontier, state, cost, problem):
        frontier.push(state)

    estrategiaLargura.recheck = False

    return genericSearch(problem, nosAtivos, estrategiaLargura)

def uniformCostSearch(problem: SearchProblem) -> List[Directions]:
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    nosAtivos = util.PriorityQueue()

    def estrategiaUCS(frontier, state, cost, problem):
        frontier.push(state, cost)

    estrategiaUCS.recheck = True

    return genericSearch(problem, nosAtivos, estrategiaUCS)

def nullHeuristic(state, problem=None) -> float:
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem: SearchProblem, heuristic=nullHeuristic) -> List[Directions]:
    """Search the node that has the lowest combined cost and heuristic first."""
    nosAtivos = util.PriorityQueue()

    def estrategiaA(frontier, state, cost, problem):
        state_pos, path, actual_cost = state
        priority = actual_cost + heuristic(state_pos, problem)
        frontier.push(state, priority)

    estrategiaA.recheck = True

    return genericSearch(problem, nosAtivos, estrategiaA)

# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
