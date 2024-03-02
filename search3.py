#
# Jay Patel
#
# Program to solve the 15 puzzle using the breadth first
# search algorithm.
#

import queue
import time
import resource

#
# Node class to store the state in list, pointer to parent node, list of child nodes,
# character move (up, right, left, or down), and a non-functional variable depth.
#
class Node:

  def __init__(self, state, parent=None, children=None, move=None, depth=0):   
    self.state = state
    self.parent = parent
    self.children = children or []
    self.move = move
    self.depth = depth

class Search:

  def goal_test(self, cur_tiles):
    return cur_tiles == ['1', '2', '3', '4', '5', '6', '7', '8', '9', '10', '11', '12', '13', '14', '15', '0']

  def solve(self, input): # Format : "1 0 2 4 5 7 3 8 9 6 11 12 13 10 14 15"

    initial_list = input.split(" ")

    start_time = time.perf_counter() # to calculate time taken to run the bfs function
    result = self.run_bfs(initial_list)
    end_time = time.perf_counter()

    solution_moves = self.get_path(result[0]) # get_path concatenates the moves all nodes

    mem_usage = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss # to calculate memory usage

    print("Moves: " + " ".join(solution_moves))
    print("Number of expanded Nodes:", result[1])
    print("Time Taken:", '%.3f' % (end_time - start_time))
    print("Max Memory (Bytes):", mem_usage)

    return "".join(solution_moves) # Get the list of moves to solve the puzzle. Format is "RDLDDRR"

  #
  # run_bfs uses a breadth first search algorithm to expand all nodes
  # until a goal state is reached.
  #
  def run_bfs(self, initial_list):
    
    nodes_expanded = 0 # initialize the variable to store the number of nodes expanded
    root = Node(initial_list, None, None, None, 0) # initialize the root node.
    q = queue.Queue()
    q.put(root)
    visited = set()

    while q:
      node = q.get()
      visited.add(node)
      
      if self.goal_test(node.state):
        return (node, nodes_expanded) # returns a tuple the goal state node and
                                      # number of nodes expanded.

      self.get_children(node) # get_children makes a list of child nodes
      
      for neighbor in node.children:
        if neighbor not in visited:
          nodes_expanded += 1
          visited.add(neighbor)
          q.put(neighbor)

  #
  # get_children takes a node with a particular board state and
  # returns a list of child nodes with four different states of the board.
  #
  def get_children(self, node):

    zero_index = node.state.index('0') #variable to store index of zero on the board

    # up state
    if zero_index not in range(0, 4): # condition to check if zero is not in the first row.
      up_state = node.state[:]
      up_state[zero_index], up_state[zero_index - 4] =  up_state[zero_index - 4],  up_state[zero_index]
      node.children.append(Node(up_state, node, None, 'U', node.depth + 1))

    # down state
    if zero_index not in range(12, 16): # condition to check if zero is not in the last row.
      down_state = node.state[:]
      down_state[zero_index], down_state[zero_index + 4] =  down_state[zero_index + 4],  down_state[zero_index]
      node.children.append(Node(down_state, node, None, 'D', node.depth + 1))

    # left state
    if zero_index not in [0, 4, 8, 12]: # condition to check if zero is not in the first column.
      left_state = node.state[:]
      left_state[zero_index], left_state[zero_index - 1] =  left_state[zero_index - 1],  left_state[zero_index]
      node.children.append(Node(left_state, node, None, 'L', node.depth + 1))

    # right state
    if zero_index not in [3, 7, 11, 15]: # condition to check if zero is not in the last column.
      right_state = node.state[:]
      right_state[zero_index], right_state[zero_index + 1] =  right_state[zero_index + 1],  right_state[zero_index]
      node.children.append(Node(right_state, node, None, 'R', node.depth + 1))

  #
  # get_path takes a node with the goal state and concatenates all
  # the moves of the nodes from the goal state node up the root node
  # using parent pointers. Returns a list of moves. (U, D, L, R)
  #
  def get_path(self, node):

    path = []

    while node.parent is not None:
      path.insert(0, node.move)
      node = node.parent

    return path

if __name__ == '__main__':
  agent = Search()
  #initial_state = "1 0 2 4 5 7 3 8 9 6 11 12 13 10 14 15"
  #path = agent.solve(initial_state)
