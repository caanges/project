from load_maps import load_map
import numpy as np
import math
import heapq
import cv2
import json

blockef_f_t = False
global click

class Cell:
    def __init__(self):
        self.parent_i = 0.0 #row index
        self.parent_j = 0.0 #column index
        self.f = float('inf') #(g + h)
        self.g = float('inf') 
        self.h = 0.0

def pad_obstacles(map_img, padding_radius):
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * padding_radius + 1, 2 * padding_radius + 1))
    padded_map = cv2.dilate(map_img, kernel, iterations=1)
    return padded_map

def is_valid(start, goal, rows, cols):
    row = start[0]
    col = start[1]
    row_goal = goal[0]
    col_goal = goal[1]
    if row < 0 or row >= rows or col < 0 or col >= cols:
        return 0
    if row_goal < 0 or row_goal > rows or col_goal < 0 or col_goal > cols:
        return 0
    return 1

def is_unblocked(map_img, row, col):
    if map_img[row][col] == 0:
        return 1
    else:
        return 0
    
def calculate_h_value(row, col, goal):
    return ((row - goal[0]) ** 2 + (col - goal[1]) ** 2) ** 0.5

def is_destination(row, col, goal):
    if row == goal[0] and col == goal[1]:
        return 1
    else:
        return 0

def trace_path(cell_details, goal):
    path=[]
    row = goal[0]
    col = goal[1]

    while not (cell_details[row][col].parent_i == row and cell_details[row][col].parent_j == col):
        path.append((row, col))
        temp_row = cell_details[row][col].parent_i
        temp_col = cell_details[row][col].parent_j
        row = temp_row
        col = temp_col

    path.append((row, col))
    path.reverse()

    for i in path:
        print("->", i, end=" ")
    print()

    return path

def a_star_search_setup(grid, start, goal, rows, cols):
    print(f"start search {start[0]}")
    print(f"start search y {start[1]}")

    if not is_valid(start, goal, rows, cols):
        print("goal or start is not valid")
        return
    
    if not is_unblocked(grid, start[0], start[1]):
        print("There is a obstacle")
        return
    
    if is_destination(start[0], start[1], goal):
        return
    
    closed_list = [[False for _ in range(cols)] for _ in range(rows)]
    cell_details = [[Cell() for _ in range(cols)] for _ in range(rows)]
    
    i = start[0]
    j = start[1]
    cell_details[i][j].f = 0.0
    cell_details[i][j].g = 0.0
    cell_details[i][j].h = 0.0
    cell_details[i][j].parent_i = i
    cell_details[i][j].parent_j = j

    open_list = []
    heapq.heappush(open_list, (0.0, i, j))

    # Initialize the flag for whether destination is found
    found_dest = False

    while len(open_list) > 0:
        p = heapq.heappop(open_list)

        i = p[1]
        j = p[2]
        closed_list[i][j] = True
        
        directions = [(0, 1), (0, -1), (1, 0), (-1, 0),
                    (1, 1), (1, -1), (-1, 1), (-1, -1)]

        for dir in directions:
            new_i = i + dir[0]
            new_j = j + dir[1]

            if not is_valid((new_i, new_j), goal, rows, cols):
                continue
            if not is_unblocked(grid, new_i, new_j):
                continue
            if closed_list[new_i][new_j]:
                continue

            # Prevent diagonal corner cutting
            if abs(dir[0]) == 1 and abs(dir[1]) == 1:
                if not (is_unblocked(grid, i + dir[0], j) and is_unblocked(grid, i, j + dir[1])):
                    continue

            if is_destination(new_i, new_j, goal):
                cell_details[new_i][new_j].parent_i = i
                cell_details[new_i][new_j].parent_j = j
                print("destination is found")

                path = trace_path(cell_details, goal)
                found_dest = True
                return path
            else:
                g_new = cell_details[i][j].g + 1.0
                h_new = calculate_h_value(new_i, new_j, goal)
                f_new = g_new + h_new

                if cell_details[new_i][new_j].f == float('inf') or cell_details[new_i][new_j].f > f_new:
                    heapq.heappush(open_list, (f_new, new_i, new_j))
                    cell_details[new_i][new_j].f = f_new
                    cell_details[new_i][new_j].g = g_new
                    cell_details[new_i][new_j].h = h_new
                    cell_details[new_i][new_j].parent_i = i
                    cell_details[new_i][new_j].parent_j = j

    if not found_dest:
        print("failed to find destination")
        return None
        
def print_Path(map_img, path):
    color_map = cv2.cvtColor(map_img.astype(np.uint8), cv2.COLOR_GRAY2BGR)

    for i in range(1, len(path)):
        pt1 = (path[i - 1][1], path[i - 1][0])
        pt2 = (path[i][1], path[i][0])
        cv2.line(color_map, pt1, pt2, color=(0,0,255), thickness = 1)

    if path:
        cv2.circle(color_map, (path[-1][1], path[-1][0]), 2, (255, 0, 0), -1)
        cv2.circle(color_map, (path[0][1], path[0][0]), 2, (0, 255, 0), -1) 

    scale_factor = 6  
    resized_img = cv2.resize(color_map, None, fx=scale_factor, fy=scale_factor, interpolation=cv2.INTER_NEAREST)

    cv2.imshow("Path with Lines", resized_img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def mouse_clicks(event, x, y, flags, params):
    #pick goal first then start
    if event == cv2.EVENT_LBUTTONDOWN:
        row = y // params['scale']
        col = x // params['scale']
        click_list = params['click']
        if len(click_list) < 2:
            click_list.append((row, col))
            print(f"click {len(click_list)}: ({row}, {col})")


def starting_search():
    map_img, obstacles, free_space, origin, res = load_map("maps/mapfinal.yaml")

    click = []
    padding_radius = 6  # tune this number based on robot size in pixels
    map_img = pad_obstacles(map_img, padding_radius)


    resize_factor = 6
    start_img = cv2.resize(map_img, None, fx=resize_factor, fy=resize_factor, interpolation=cv2.INTER_NEAREST)
    start_img = cv2.cvtColor(start_img.astype(np.uint8), cv2.COLOR_GRAY2BGR)

    cv2.imshow("Click to set start and goal", start_img)
    
    cv2.setMouseCallback("Click to set start and goal", mouse_clicks, {'scale': resize_factor, 'click': click})

    print("enter start and end points")

    while len(click) < 2:
        cv2.waitKey(1)

    cv2.destroyAllWindows()

    rows = len(map_img)
    cols = len(map_img[0])

    #x_goal = 30
    #y_goal = 40


    #x_start = list(click[0])
    #y_start = list(click[1])
        
    print(rows)
    print(cols)

    #print(map_img[x_goal][y_goal])
    #print(map_img[x_start][y_start])

    start = click[0]
    goal = click[1]

    path = a_star_search_setup(map_img, start, goal, rows, cols)

    if path:

        flipped_path = list(reversed(path))

        path_data = [{
            #"y": round(origin[0] + float(p[1]) * res, 2), 
            #"x": round(origin[1] + float(p[0]) * res, 2)
            
            #"y": round(origin[0] + (float(p[1]) * res), 2), 
            #"x": round(origin[1] + (float(p[0]) * res), 2)
            
            "y": round(origin[0] + (float(p[1]) * res), 2), 
            "x": round(origin[1] + ((map_img.shape[0] - 1 - float(p[0])) * res), 2)

            } for p in flipped_path
        ]  # swap (row,col) -> (x,y)
        with open("path_coordinates.json", "w") as f:
            json.dump(path_data, f, indent=4)
        
        print("Path written to path_coordinates.json")
        print_Path(map_img, path)
    else:
        print("no path found")