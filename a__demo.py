import pygame
import math
from queue import PriorityQueue

# =============================================================================
# UI COLOURS AND INITIAL SETTINGS
# =============================================================================
# Defining RGB values for different node states
WHITE = (255, 255, 255)   # Unvisited node
BLACK = (0, 0, 0)         # Barrier (Wall)
GREEN = (0, 255, 0)       # Start point
RED = (255, 0, 0)         # End point
ORANGE = (255, 165, 0)    # Final path color
GREY = (128, 128, 128)    # Grid line color
TURQUOISE = (64, 224, 208)# Nodes currently in the open set
BLUE = (173, 216, 230)    # Visited nodes

# Window setup
WIDTH = 800
WIN = pygame.display.set_mode((WIDTH, WIDTH))
pygame.display.set_caption("A* Pathfinding (Diagonal Movement)")

# =============================================================================
# THE SPOT CLASS: Represents each individual square on the grid
# =============================================================================
class Spot:
    def __init__(self, row, col, width, total_rows):
        self.row = row                  # Row index
        self.col = col                  # Column index
        self.x = row * width            # Pixel X
        self.y = col * width            # Pixel Y
        self.color = WHITE              # Initial state
        self.neighbors = []             # List for adjacent nodes
        self.width = width              # Square size
        self.total_rows = total_rows    # Grid size

    def get_pos(self):
        return self.row, self.col       # Coordinates helper

    def is_barrier(self):
        return self.color == BLACK      # Check if wall

    def draw(self, win):
        # Draw the square on screen
        pygame.draw.rect(win, self.color, (self.x, self.y, self.width, self.width))

    def update_neighbors(self, grid):
        self.neighbors = []
        # STRAIGHT MOVEMENTS (Cost: 1)
        if self.row < self.total_rows - 1 and not grid[self.row + 1][self.col].is_barrier(): # DOWN
            self.neighbors.append((grid[self.row + 1][self.col], 1))
        if self.row > 0 and not grid[self.row - 1][self.col].is_barrier(): # UP
            self.neighbors.append((grid[self.row - 1][self.col], 1))
        if self.col < self.total_rows - 1 and not grid[self.row][self.col + 1].is_barrier(): # RIGHT
            self.neighbors.append((grid[self.row][self.col + 1], 1))
        if self.col > 0 and not grid[self.row][self.col - 1].is_barrier(): # LEFT
            self.neighbors.append((grid[self.row][self.col - 1], 1))

        # DIAGONAL MOVEMENTS (Cost: sqrt(2) approx 1.4)
        # Check Down-Right
        if self.row < self.total_rows - 1 and self.col < self.total_rows - 1:
            if not grid[self.row + 1][self.col + 1].is_barrier():
                self.neighbors.append((grid[self.row + 1][self.col + 1], 1.4))
        # Check Down-Left
        if self.row < self.total_rows - 1 and self.col > 0:
            if not grid[self.row + 1][self.col - 1].is_barrier():
                self.neighbors.append((grid[self.row + 1][self.col - 1], 1.4))
        # Check Up-Right
        if self.row > 0 and self.col < self.total_rows - 1:
            if not grid[self.row - 1][self.col + 1].is_barrier():
                self.neighbors.append((grid[self.row - 1][self.col + 1], 1.4))
        # Check Up-Left
        if self.row > 0 and self.col > 0:
            if not grid[self.row - 1][self.col - 1].is_barrier():
                self.neighbors.append((grid[self.row - 1][self.col - 1], 1.4))

# =============================================================================
# UTILITY FUNCTIONS
# =============================================================================
def h(p1, p2):
    # Euclidean distance is better for diagonal movement
    x1, y1 = p1
    x2, y2 = p2
    return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)

def reconstruct_path(came_from, current, draw):
    # Path highlighting loop
    while current in came_from:
        current = came_from[current]
        if current.color != GREEN:
            current.color = ORANGE
        draw()

def make_grid(rows, width):
    # Create the 2D grid structure
    grid = []
    gap = width // rows
    for i in range(rows):
        grid.append([])
        for j in range(rows):
            spot = Spot(i, j, gap, rows)
            grid[i].append(spot)
    return grid

def draw_grid(win, rows, width):
    # Draw the grid wireframe
    gap = width // rows
    for i in range(rows):
        pygame.draw.line(win, GREY, (0, i * gap), (width, i * gap))
        for j in range(rows):
            pygame.draw.line(win, GREY, (j * gap, 0), (j * gap, width))

def draw(win, grid, rows, width):
    # Clear and redraw everything
    win.fill(WHITE)
    for row in grid:
        for spot in row:
            spot.draw(win)
    draw_grid(win, rows, width)
    pygame.display.update()

def get_clicked_pos(pos, rows, width):
    # Maps screen pixels to grid index
    gap = width // rows
    y, x = pos
    return y // gap, x // gap

# =============================================================================
# THE A* ALGORITHM (Modified for weights)
# =============================================================================
def algorithm(draw, grid, start, end):
    count = 0
    open_set = PriorityQueue()
    open_set.put((0, count, start))
    came_from = {}
    g_score = {spot: float("inf") for row in grid for spot in row}
    g_score[start] = 0
    f_score = {spot: float("inf") for row in grid for spot in row}
    f_score[start] = h(start.get_pos(), end.get_pos())
    open_set_hash = {start}

    while not open_set.empty():
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()

        current = open_set.get()[2] # Extract the Spot object
        open_set_hash.remove(current)

        if current == end:
            reconstruct_path(came_from, end, draw)
            end.color = RED
            return True

        for neighbor_data in current.neighbors:
            neighbor, weight = neighbor_data # Unpack spot and its movement cost
            temp_g_score = g_score[current] + weight # Use variable weight (1 or 1.4)

            if temp_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = temp_g_score
                f_score[neighbor] = temp_g_score + h(neighbor.get_pos(), end.get_pos())
                if neighbor not in open_set_hash:
                    count += 1
                    open_set.put((f_score[neighbor], count, neighbor))
                    open_set_hash.add(neighbor)
                    neighbor.color = TURQUOISE

        draw()
        if current != start:
            current.color = BLUE
    return False

# =============================================================================
# MAIN EXECUTION LOOP
# =============================================================================
def main(win, width):
    ROWS = 50
    grid = make_grid(ROWS, width)
    start, end, run = None, None, True

    while run:
        draw(win, grid, ROWS, width)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False
            
            if pygame.mouse.get_pressed()[0]: # LEFT CLICK
                pos = pygame.mouse.get_pos()
                row, col = get_clicked_pos(pos, ROWS, width)
                spot = grid[row][col]
                if not start and spot != end:
                    start = spot
                    start.color = GREEN
                elif not end and spot != start:
                    end = spot
                    end.color = RED
                elif spot != end and spot != start:
                    spot.color = BLACK

            elif pygame.mouse.get_pressed()[2]: # RIGHT CLICK
                pos = pygame.mouse.get_pos()
                row, col = get_clicked_pos(pos, ROWS, width)
                spot = grid[row][col]
                spot.color = WHITE
                if spot == start: start = None
                elif spot == end: end = None

            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE and start and end:
                    for row in grid:
                        for spot in row:
                            spot.update_neighbors(grid)
                    algorithm(lambda: draw(win, grid, ROWS, width), grid, start, end)
                
                if event.key == pygame.K_c: # CLEAR
                    start, end = None, None
                    grid = make_grid(ROWS, width)

    pygame.quit()

if __name__ == "__main__":
    main(WIN, WIDTH)
