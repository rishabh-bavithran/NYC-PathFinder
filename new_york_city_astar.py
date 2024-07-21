import math
import numpy as np
import matplotlib.pyplot as plt

# start = np.array([615,378]) #shortest
# goal = np.array([632,369])

start = np.array([889,312])
goal = np.array([615,378])

grid = np.load('new_york.npy')

explored_path = np.zeros([len(grid), len(grid[0])], dtype=float)
#EASY VISUALIZATION PURPOSES
explored_path -= 2000
for i in range(1,8): 
    for j in range(1,8): 
        explored_path[start[0]+ i, start[1] + j] = 1
        explored_path[goal[0]+ i,goal[1]+ j] = 1

best_path = np.zeros([len(grid), len(grid[0])], dtype=int)


class AStarNYC:
    def __init__(self, start, goal, grid, explored_path):
        self.explored = {}
        self.not_explored = {}
        self.current_pos = start
        self.current_pos_str = str(start)
        self.current_pos_dist = 0
        self.goal_str = str(goal)
        self.not_explored[str(start)] = 0
        self.grid = grid
        self.explored_path = explored_path
        self.path_threshold = 0.2

    def get_neighbours(self):
        potential_moves = self.potential_neighbours(self.current_pos)
        
        for move in potential_moves: 
            if not self.validate_neighbour(move): 
                continue
        
        #STORING EACH NEIGHBOURS MOVES FROM START AND HEURISTIC FOR CHOOSING BEST NEIGHBOUR LATER
            if str(move) not in self.explored and str(move) not in self.not_explored: 
                self.not_explored[str(move)] = self.current_pos_dist  + 1 + self.heuristic(move)
        self.explored[str(self.current_pos)] = 0 
        return True


    def choose_best_neighbour(self):
        #CHOOSING BEST NEIGHBOUR WHICH HAS THE LEAST WEIGHT (HEART OF THE A*, BFS, DFS(CHOOSING MAX WEIGHT [REVERSE=TRUE]))
        sorted_moves = sorted(self.not_explored, key=self.not_explored.get, reverse=False)
        self.current_pos_str = sorted_moves[0]
        self.current_pos = self.str_to_np(self.current_pos_str)
        
        #REMOVAL OF HEURISTIC DISTANCE REQUIRED, ONLY NUMBER OF MOVES FROM START REQUIRED, HEURISTIC ONLY REQUIRED FOR SORTING
        self.current_pos_dist = self.not_explored.pop(self.current_pos_str) - self.heuristic(self.current_pos)
        self.explored_path[self.current_pos[0], self.current_pos[1]] = round(self.current_pos_dist, 1)
        return True

    def heuristic(self, move):

        #EUCLEDIAN DISTANCE FROM GOAL
        x_dist = abs(goal[0] - move[0])
        y_dist = abs(goal[1] - move[1])
        dist = math.sqrt(x_dist**2 + y_dist**2)
        answer = dist
        return round(answer, 1)
    
    def goal_check(self):
        if self.goal_str in self.not_explored:
            self.current_pos = self.str_to_np(self.goal_str)
            self.current_pos_dist = self.not_explored.pop(self.goal_str)
            self.explored_path[self.current_pos[0], self.current_pos[1]] = self.current_pos_dist
            return True
        return False

    def potential_neighbours(self, current_pos):
        up = np.array([-1, 0])
        down = np.array([1, 0])
        left = np.array([0, -1])
        right = np.array([0, 1])

        #ALL 8 SURROUNDING PIXELS
        potential_moves = [current_pos + up, current_pos + down, current_pos + left, current_pos + right, 
                           current_pos + up+right, current_pos + up+left, current_pos + down+right, current_pos + down+left]
        return potential_moves

    def validate_neighbour(self, move):
        # OVERALL BOUNDARY CHECK
        if (move[0] < 0) or (move[0] >= len(grid)):
            return False
        if (move[1] < 0) or (move[1] >= len(grid[0])):
            return False
        # OBSTACLE CHECK 
        if self.grid[move[0], move[1], 0] < self.path_threshold:
            return False
        return True

    def str_to_np(self, string):
        string = string.replace('[', '')
        string = string.replace(']', '')
        string = string.split()
        array = [int(string[0]), int(string[1])]
        return np.array(array)

astar = AStarNYC(start, goal, grid, explored_path)
fig, ax = plt.subplots()
ax.clear()
ax.imshow(grid, cmap='Greys')
ax.imshow(explored_path, cmap='hot', alpha=0.75) 
plt.pause(2) 

while True:
    astar.get_neighbours()
    if astar.goal_check():
        break
    astar.choose_best_neighbour()
    ax.clear()
    ax.imshow(grid, cmap='Greys')
    ax.imshow(explored_path, cmap='hot', alpha=0.75) 
    plt.tight_layout()
    plt.pause(0.01)  


print('')
print('EXPLORED PIXEL COUNT : ' + str(len(explored_path[explored_path > 0])))


current_pos = goal
moves_to_goal = 0

#FINDING BEST PATH TO START FROM GOAL 
while True:
    best_path[current_pos[0], current_pos[1]] = 1
    potential_color_path  = astar.potential_neighbours(current_pos) 
    
    #VISUALIZATION PURPOSES
    for move in potential_color_path: 
        best_path[move[0], move[1]] = 1

    moves_from_start = round(explored_path[current_pos[0], current_pos[1]], 1)
    if moves_from_start == 1:
        break
    potential_moves = astar.potential_neighbours(current_pos)
    for move in potential_moves:
        if not astar.validate_neighbour(move):
            continue
        move_tracker = explored_path[move[0], move[1]]
        if move_tracker == (moves_from_start - 1):
            moves_to_goal += 1
            current_pos = move
            break

print('MOVES TO GOAL: ' + str(moves_to_goal))

ax.clear()
ax.imshow(grid, cmap='Greys')
ax.imshow(best_path, cmap='hot', alpha=0.75) 
plt.tight_layout()
plt.draw()
plt.pause(0.01)  
plt.show()
