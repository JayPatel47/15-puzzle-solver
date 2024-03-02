#
# Jay Patel
#
# A program which performs iterative deepening A-star search using two
# heuristic approaches: Manhattan distance and Misplaced tiles.
# The algorithm is performed on a board to solve 15 puzzle.
#

import time
import psutil
import os
import sys
import math
import tracemalloc
from queue import PriorityQueue

#
# Node class to store the state in list, pointer to parent node, list of child nodes,
# character move (up, right, left, or down), and a non-functional variable depth.
#
class Node:

  def __init__(self, state, parent, move, depth, path_cost):   
    self.state = state
    self.parent = parent
    self.move = move
    self.depth = depth
    self.path_cost = path_cost

  def __lt__(self, other):
    return (self.path_cost < other.path_cost)

class Search:

  # This function backtracks from current node to reach initial configuration.
  # The list of actions would constitute a solution path
  def find_path(self, node):
    path = []
    
    while node.parent is not None:
      path.append(node.move)
      node = node.parent

    path.reverse()
    return path

  # calculates the number of tiles that are not as same as the goal state.
  def misplaced_tiles(self, node):
    goal_state = ['1', '2', '3', '4', '5', '6', '7', '8', '9', '10', '11', '12', '13', '14', '15', '0']
    misplaced = 0
    for i in range(15):
      if node.state[i] != goal_state[i]:
        misplaced += 1

    return misplaced

  #calculates the distance between two tiles on a node.
  def manhatten_distance(self, node):
    distance = 0
    for i in range(len(node.state)):
      if node.state[i] != 0:
        row_diff = abs((int(node.state[i])-1) // 4 - i // 4)
        col_diff = abs((int(node.state[i])-1) % 4 - i % 4)
        distance += row_diff + col_diff

    return distance
    
    
  #
  # get_children takes a node with a particular board state and
  # returns a list of child nodes with four different states of the board.
  #
  def get_children(self, node, heuristic):
    # get the possible moves (up, down, left, right)
    zero_index = node.state.index('0')
    row, col = divmod(zero_index, 4)
    moves = []

    if row > 0:
      up = node.state[:]
      up[zero_index], up[zero_index-4] = up[zero_index-4], up[zero_index]
      moves.append(Node(up, node, 'U', node.depth+1, node.depth + heuristic(node)))
    if row < 3:
      down = node.state[:]
      down[zero_index], down[zero_index+4] = down[zero_index+4], down[zero_index]
      moves.append(Node(down, node, 'D', node.depth+1, node.depth + heuristic(node)))
    if col > 0:
      left = node.state[:]
      left[zero_index], left[zero_index-1] = left[zero_index-1], left[zero_index]
      moves.append(Node(left, node, 'L', node.depth+1, node.depth + heuristic(node)))
    if col < 3:
      right = node.state[:]
      right[zero_index], right[zero_index+1] = right[zero_index+1], right[zero_index]
      moves.append(Node(right, node, 'R', node.depth+1, node.depth + heuristic(node)))

    return moves

  # function performs the iterative deepening A* search algorithm
  # Returns solution node, path cost, and nodes expanded.
  def idbs(self, root_node, heuristic, threshold):
    nodes_expanded = 0 # initialize the variable to store the number of nodes expanded
    frontier = PriorityQueue() # priority queue based on the path cost.
    frontier.put((heuristic(root_node), root_node)) 
    reached = {}
    min_cost = float('inf')

    while frontier:
      _, node = frontier.get()

      if node.path_cost > threshold:
        min_cost = min(min_cost, node.path_cost)

      if (self.goal_test(node.state)):
        return node, node.path_cost, nodes_expanded

      children = self.get_children(node, heuristic)
      for child in children:
        if child not in reached or child.path_cost < reached[child.state]:
          nodes_expanded += 1
          reached["".join(child.state)] = child
          frontier.put((child.path_cost, child))

    return "cutoff", min_cost

  # calls the idbfs function as long as a goal state is reached.
  def run_idbfs(self, root_node, heuristic):
    root_node.path_cost = heuristic(root_node)
    threshold = root_node.path_cost
    while True:
      result, threshold, nodes_expanded = self.idbs(root_node, heuristic, threshold)

      if result != "cutoff":
            return result, nodes_expanded


  def goal_test(self, cur_tiles):
    return cur_tiles == ['1', '2', '3', '4', '5', '6', '7', '8', '9', '10', '11', '12', '13', '14', '15', '0']  

  def solve(self, input, heuristic = "manhattan"): # Format : "1 0 2 4 5 7 3 8 9 6 11 12 13 10 14 15"

    initial_list = input.split(" ")

    root = Node(initial_list, None, None, 0, 0)
    start_time = time.time()
    tracemalloc.start()
    if heuristic == "manhattan":
      solution_node, expanded_nodes = self.run_idbfs(root, self.manhatten_distance)
    if heuristic == "misplaced tiles":
        solution_node, expanded_nodes = self.run_idbfs(root, self.misplaced_tiles)
    
    path = self.find_path(solution_node)
    end_time = time.time()
    current, peak = tracemalloc.get_traced_memory()
    print("Moves: " + " ".join(path))
    print("Number of expanded Nodes: " + str(expanded_nodes))
    print("Time Taken: " + str(end_time - start_time))
    print("Max Memory (Bytes): " + format(peak))
    return "".join(path)

if __name__ == '__main__':
  agent = Search()
  initial_state = "1 0 2 4 5 7 3 8 9 6 11 12 13 10 14 15"
  path = agent.solve(initial_state)
