import math


class Node():   #creating class Node for creating nodes in the search tree of A* algorithm
    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):  #Using dunder method for finding node objects with same positions 
        return self.position == other.position


def astar(maze, start, end):  # Function for A* algorithm
    
    if maze[end[0]][end[1]] == 1:  #If end position is an obstacle, then return no path
        return None
    # Create start and end node
    start_node = Node(None, start)
    start_node.g = start_node.h = start_node.f = 0
    end_node = Node(None, end)
    end_node.g = end_node.h = end_node.f = 0

    # Initialize both open and closed list
    open_list = []
    closed_list = []

    open_list.append(start_node)  # Add the start node

    while len(open_list) > 0:  # Loop until open list becomes empty

        current_node = open_list[0]  
        current_index = 0
        for index, item in enumerate(open_list):  # Get the current node
            if item.f < current_node.f:
                current_node = item
                current_index = index

        open_list.pop(current_index)  # Remove the current node from open list and add it to closed list
        closed_list.append(current_node)

        if current_node == end_node:  # Return the path if we reach the end node
            path = [] 
            current = current_node
            while current is not None:
                path.append(current.position)
                current = current.parent  # Traversing back in the search tree
            return path[::-1]    # Return reversed path

        # Generate child nodes
        children = []
        for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0)]:  # Adjacent directions: up, down, left, right

            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])  # Get child node position

            if node_position[0] > (len(maze) - 1) or node_position[0] < 0 or node_position[1] > (len(maze[len(maze)-1]) -1) or node_position[1] < 0:  # Condition to check if node within range
                continue

            if maze[node_position[0]][node_position[1]] != 1:  # Check for obstacles
                continue

            new_node = Node(current_node, node_position) # Create new node

            children.append(new_node) # Append child nodes

        for child in children:  # Loop through child nodes

            for closed_child in closed_list:  # Child is on the closed list
                if child == closed_child:
                    continue

            # Create the f, g, and h values
            child.g = current_node.g + 1  # Assuming cost in each of the four directions is one
            child.h = math.sqrt(((child.position[0] - end_node.position[0]) ** 2) + ((child.position[1] - end_node.position[1]) ** 2)) # Taking euclidean distance as the heuristic
            child.f = child.g + child.h

            # Child is already in the open list
            for open_node in open_list:
                if child == open_node and child.g > open_node.g:
                    continue

            # Add the child to the open list
            open_list.append(child)

def sic(paths):  # Function to calculate sum of individual costs
    sum = 0
    for path in paths:
        sum = sum + len(path)
    return sum - len(paths)

def find_conflict(paths): # Finding first conflict and returning the tuple (i, j, (x,y)). Here i, j are agents
    for i in range(len(paths)-1):
        for j in range(i+1, len(paths)):
            for k in range(min(len(paths[i]), len(paths[j]))):
                if(paths[i][k] == paths[j][k]):
                    print(f"Conflict found: ({i+1}, {j+1}, {paths[i][k]})")
                    return (i+1, j+1, paths[i][k])
    return None


class NodeInCT: # Defining class node to create nodes in the constraint tree
    def __init__(self, constraints, solution, total_cost, maze):
        self.maze = maze
        self.constraints = constraints
        self.solution = solution
        self.total_cost = total_cost
        self.left = None  # Defining left node for the binary tree
        self.right = None # Defining right node for the binary tree

def cbs_mapf(paths, start_list, goal_list, maze): # Function for mapf using cbs
    open_list = []
    closed_list = []
    maze_copies = [] # Initializing an empty list which contains n copies of agents, n is number of agents
    for i in range(len(paths)): 
        maze_copy = [row[:] for row in maze] 
        maze_copies.append(maze_copy)
    solution = paths.copy()
    constraints = []
    total_cost = sic(paths)
    node = NodeInCT(constraints, solution, total_cost, maze) # Constructing root node of the binary search tree
    open_list.append(node) 
    while len(open_list) > 0: # Loop until open list becomes empty
        current_node = open_list[0]
        for _, item in enumerate(open_list): # Finding the node with the lowest sum of individual cost of paths
            if sic(item.solution) < sic(current_node.solution):
                current_node = item
        open_list.remove(current_node)
        closed_list.append(current_node)
        
        print("Solution in the current node:",current_node.solution)
        conflict = find_conflict(current_node.solution) # Finding first conflict in the current node's solution 
        if conflict == None: 
            print("Final paths are")
            return current_node.solution
        
        constraint1 = (conflict[0], conflict[2]) # Creating two constraints to be passed to each node of the binary tree
        constraint2 = (conflict[1], conflict[2])

        solution1 = current_node.solution.copy()  # Create a copy of the current solution
        solution2 = current_node.solution.copy()  # Create a copy of the current solution

        if constraint1[1] != goal_list[constraint1[0]-1] and constraint2[1] == goal_list[constraint2[0]-1]:
            maze_copies[conflict[0]-1][conflict[2][0]][conflict[2][1]] = 1 # For the i th agent, change its own copy of maze such that obstacle is created at the conflicting position
            solution1[conflict[0] - 1] = astar(maze_copies[conflict[0]-1], start_list[conflict[0] - 1], goal_list[conflict[0] - 1]) # Find new path for i th agent considering conflict as an obstacle
            node1 = NodeInCT(constraint1, solution1, sic(solution1), maze_copies[conflict[0]-1])  # Create left node
            current_node.left = node1
            open_list.append(node1)

        elif constraint1[1] == goal_list[constraint1[0]-1] and constraint2[1] != goal_list[constraint2[0]-1]:
            maze_copies[conflict[1]-1][conflict[2][0]][conflict[2][1]] = 1 # For the j th agent, change its own copy of maze such that obstacle is created at the conflicting position
            solution2[conflict[1] - 1] = astar(maze_copies[conflict[1]-1], start_list[conflict[1] - 1], goal_list[conflict[1] - 1]) # Find new path for j th agent considering conflict as an obstacle
            node2 = NodeInCT(constraint2, solution2, sic(solution2), maze_copies[conflict[1]-1])  # Create right node
            current_node.right = node2
            open_list.append(node2)

        else:
            solution1 = current_node.solution.copy()  # Create a copy of the current solution
            maze_copies[conflict[0]-1][conflict[2][0]][conflict[2][1]] = 1 # For the i th agent, change its own copy of maze such that obstacle is created at the conflicting position
            solution1[conflict[0] - 1] = astar(maze_copies[conflict[0]-1], start_list[conflict[0] - 1], goal_list[conflict[0] - 1]) # Find new path for i th agent considering conflict as an obstacle
        
            solution2 = current_node.solution.copy()  # Create a copy of the current solution
            maze_copies[conflict[1]-1][conflict[2][0]][conflict[2][1]] = 1 # For the j th agent, change its own copy of maze such that obstacle is created at the conflicting position
            solution2[conflict[1] - 1] = astar(maze_copies[conflict[1]-1], start_list[conflict[1] - 1], goal_list[conflict[1] - 1]) # Find new path for j th agent considering conflict as an obstacle

            node1 = NodeInCT(constraint1, solution1, sic(solution1), maze_copies[conflict[0]-1])  # Create left node
            node2 = NodeInCT(constraint2, solution2, sic(solution2), maze_copies[conflict[1]-1])  # Create right node
            current_node.left = node1  
            current_node.right = node2  
            open_list.append(node1)  # Append left and right nodes to open list
            open_list.append(node2) 

    return solution

        
def main():

    maze = [[1, 1, 1, 1, 1, 1, 0, 1, 1, 1], # Custom grid/maze with 0 as walkable cells and 1 as obstacles
            [1, 1, 1, 0, 1, 1, 1, 0, 1, 1],
            [1, 1, 1, 0, 1, 1, 0, 1, 0, 1],
            [0, 0, 1, 0, 1, 0, 0, 0, 0, 1],
            [1, 1, 1, 0, 1, 1, 1, 0, 1, 0],
            [1, 0, 1, 1, 1, 1, 0, 1, 0, 0],
            [1, 0, 0, 0, 0, 1, 0, 0, 0, 1],
            [1, 0, 1, 1, 1, 1, 0, 1, 1, 1],
            [1, 1, 1, 0, 0, 0, 1, 0, 0, 1]]

    # agents = int(input("Enter number of agents: "))
    agents = 3
    start_list = []
    goal_list = []
    paths = []

    # for agent in range(agents):
    #     start_x = int(input("Enter x coordinate for start position of agent: "))
    #     start_y = int(input("Enter y coordinate for start position of agent: "))
    #     start_list.append((start_x, start_y))
    #     goal_x = int(input("Enter x coordinate for goal position of agent: "))
    #     goal_y = int(input("Enter y coordinate for goal position of agent: "))
    #     goal_list.append((goal_x, goal_y))
        
    start_list.append((0, 0))
    start_list.append((0, 2))
    start_list.append((2, 0))
    # start_list.append((1, 4))
    # start_list.append((3, 3))
    # start_list.append((3, 8))
    goal_list.append((2, 2))
    goal_list.append((2, 0))
    goal_list.append((2, 3))
    # goal_list.append((6, 8))
    # goal_list.append((6, 1))
    # goal_list.append((6, 5))

    for i in range(agents):
        path = astar(maze, start_list[i], goal_list[i])
        paths.append(path)

    for path in paths:
        if path is None:
            print("Solution is not feasible")
            return
    
    print("Initial paths are: ")
    for j in range(agents):
        print(paths[j])

    mapf = cbs_mapf(paths, start_list, goal_list, maze)
    
    for i in range(agents):
        print(f"Path for agent {i+1}: ",mapf[i])

if __name__ == '__main__':
    main()