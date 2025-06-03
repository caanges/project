import numpy as np
import cv2
from load_maps import load_map

blocked_f_t = False
SIZE = 5


def draw_map(grid, Path=None, visited=None, start=None, end=None):
    rows,cols = len(grid), len(grid[0])
    img = np.zeros((rows * SIZE, cols * SIZE, 3), dtype=np.uint8)

