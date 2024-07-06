import math

class Node():
    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position


def astar(maze, start, end):
    

    # Create start and end node
    start_node = Node(None, start)
    start_node.g = start_node.h = start_node.f = 0
    end_node = Node(None, end)
    end_node.g = end_node.h = end_node.f = 0

    # Initialize both open and closed list
    open_list = []
    closed_list = []

    # Add the start node
    open_list.append(start_node)

    # Loop until you find the end
    while len(open_list) > 0:

        # Get the current node
        current_node = open_list[0]
        current_index = 0
        for index, item in enumerate(open_list):
            if item.f < current_node.f:
                current_node = item
                current_index = index

        # Pop current off open list, add to closed list
        open_list.pop(current_index)
        closed_list.append(current_node)

        # Found the goal
        if current_node == end_node:
            path = []
            current = current_node
            while current is not None:
                path.append(current.position)
                current = current.parent
            return path[::-1] # Return reversed path

        # Generate children
        children = []
        for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0)]: # Adjacent squares

            # Get node position
            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

            # Make sure within range
            if node_position[0] > (len(maze) - 1) or node_position[0] < 0 or node_position[1] > (len(maze[len(maze)-1]) -1) or node_position[1] < 0:
                continue

            # Make sure walkable terrain
            if maze[node_position[0]][node_position[1]] != 0:
                continue

            # Create new node
            new_node = Node(current_node, node_position)

            # Append
            children.append(new_node)

        # Loop through children
        for child in children:

            # Child is on the closed list
            for closed_child in closed_list:
                if child == closed_child:
                    continue

            # Create the f, g, and h values
            child.g = current_node.g + 1
            child.h = ((child.position[0] - end_node.position[0]) ** 2) + ((child.position[1] - end_node.position[1]) ** 2)
            child.f = child.g + child.h

            # Child is already in the open list
            for open_node in open_list:
                if child == open_node and child.g > open_node.g:
                    continue

            # Add the child to the open list
            open_list.append(child)

def sic(paths):
    sum = 0
    for path in paths:
        sum = sum + len(path)
    return sum - len(paths)

def find_conflict(paths):
    conflicts = []
    for i in range(len(paths)-1):
        for j in range(i+1, len(paths)):
            for k in range(min(len(paths[i]), len(paths[j]))):
                if(paths[i][k] == paths[j][k]):
                    conflicts.append((i+1, j+1, paths[i][k]))
    return conflicts


class NodeInCT:
    def __init__(self, constraints, solution, total_cost):
        self.constraints = constraints
        self.solution = solution
        self.total_cost = total_cost

def cbs_mapf(paths):
    open_list = []
    closed_list = []
    solution = paths
    #conflicts = find_conflict(paths)
    constraints = []
    total_cost = sic(paths)
    node = NodeInCT(constraints, solution, total_cost)
    open_list.append(node)
    while len(open_list) > 0:
        current_node = open_list[0]
        for _, item in enumerate(open_list):
            if (sic(item.solution) < sic(current_node.solution)):
                current_node = item

        open_list.remove(current_node)
        closed_list.append(current_node)

        if find_conflict(current_node.solution) == None:
            return current_node.solution
        conflicts = find_conflict(current_node.solution)
        

        
def main():

    maze = [[0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [1, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 1, 1, 1, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 1, 0, 1, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0, 0, 0, 0, 0]]

    agents = input("Enter number of agents: ")
    start_list = []
    goal_list = []
    paths = []

    for agent in range(agents):
        start_x = int(input("Enter x coordinate for start position: "))
        start_y = int(input("Enter y coordinate for start position: "))
        start_list.append((start_x, start_y))
        goal_x = int(input("Enter x coordinate for goal position: "))
        goal_y = int(input("Enter y coordinate for goal position: "))
        goal_list.append((goal_x, goal_y))
        
    for i in range(agents):
        path = astar(maze, start_list(i), goal_list(i))
        paths.append(path)


if __name__ == '__main__':
    main()

