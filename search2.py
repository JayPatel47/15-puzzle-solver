#
# Jay Patel
#
# A program which performs iterative deepening depth 
# first search to find the solution to any given board
# position for 15 puzzle 
#

import time
import psutil
import os
import sys
import math

#
# Node class to store the state in list, pointer to parent node, list of child nodes,
# character move (up, right, left, or down), and a non-functional variable depth.
#
class Node:

  def __init__(self, state, parent, move, depth=0):   
    self.state = state
    self.parent = parent
    self.move = move
    self.depth = depth

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

  #
  # get_children takes a node with a particular board state and
  # returns a list of child nodes with four different states of the board.
  #
  def get_children(self, node):
        # get the possible moves (up, down, left, right)
        zero_index = node.state.index('0')
        row, col = divmod(zero_index, 4)
        moves = []

        if row > 0:
            up = node.state[:]
            up[zero_index], up[zero_index-4] = up[zero_index-4], up[zero_index]
            moves.append(Node(up, node, 'U', node.depth + 1))
        if row < 3:
            down = node.state[:]
            down[zero_index], down[zero_index+4] = down[zero_index+4], down[zero_index]
            moves.append(Node(down, node, 'D', node.depth + 1))
        if col > 0:
            left = node.state[:]
            left[zero_index], left[zero_index-1] = left[zero_index-1], left[zero_index]
            moves.append(Node(left, node, 'L', node.depth + 1))
        if col < 3:
            right = node.state[:]
            right[zero_index], right[zero_index+1] = right[zero_index+1], right[zero_index]
            moves.append(Node(right, node, 'R', node.depth + 1))

        return moves

  # This checks for cycle in the graph and helps the algorithm to avoid traversing in a loop.
  def is_cycle(self, node):
    ancestor = node.parent

    while ancestor:
      if ancestor.state == node.state:
          return True
      ancestor = ancestor.parent

    return False

  # function performs the depth first search algorithm
  # Returns a goal state.
  def dls(self, root_node, depth):
    max_memory = 0
    nodes_expanded = 0 # initialize the variable to store the number of nodes expanded
    frontier = [] # initialize stack with the root node.
    frontier.append(root_node)

    result = "failure"

    while frontier:
      max_memory = max(max_memory, sys.getsizeof(frontier))
      node = frontier.pop()

      if (self.goal_test(node.state)):
        return node, nodes_expanded, max_memory

      if node.depth > depth:
        result = "cutoff"
      elif not self.is_cycle(node):
        children = self.get_children(node)
        for child in children:
          nodes_expanded += 1
          frontier.append(child)
    
    return result, nodes_expanded, max_memory
        
  #
  # run_iddfs used iterative deepening depth-first search algorithm to expand all nodes
  # until a goal state is reached.
  #
  def run_iddfs(self, root_node):

    # runs forever until finds a goal state
   for depth in range(sys.maxsize):
      result, nodes_expanded, mem_usage = self.dls(root_node, depth)
      
      if result != "cutoff" and result != "failure":
          return self.find_path(result), nodes_expanded, mem_usage
      


  def goal_test(self, cur_tiles):
    return cur_tiles == ['1', '2', '3', '4', '5', '6', '7', '8', '9', '10', '11', '12', '13', '14', '15', '0']  

  def solve(self, input): # Format : "1 0 2 4 5 7 3 8 9 6 11 12 13 10 14 15"

    initial_list = input.split(" ")
    root = Node(initial_list, None, None, 0)
    start_time = time.time()
    path, expanded_nodes, memory_consumed = self.run_iddfs(root)
    end_time = time.time()
    print("Moves: " + " ".join(path))
    print("Number of expanded Nodes: " + str(expanded_nodes))
    print("Time Taken: " + str(end_time - start_time))
    print("Max Memory (Bytes): " + str(memory_consumed))
    return "".join(path)

if __name__ == '__main__':
  agent = Search()
  #initial_state = "1 0 2 4 5 7 3 8 9 6 11 12 13 10 14 15"
  #path = agent.solve(initial_state)
