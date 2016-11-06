# Stores an occupancy grid and a list of food positions in cm.
# Gives information about where obstacles and food are.
# Displays arena with robot and food marked.
# Catches key presses and updates food list.

import sys
import numpy as np
import cv2
import occ_grid

class Arena:
    def __init__(self, occ_grid, x_sz=139.5, y_sz=75.5):
        self.grid = occ_grid
        self.x_sz = x_sz
        self.y_sz = y_sz
        ht, wd = self.grid.shape[0:2]
        # scale
        self.x_sc = self.x_sz / float(wd)
        self.y_sc = self.y_sz / float(ht)
        # display scale
        self.scale = 16
        self.food = []

    BLUE = (255, 0, 0)
    RED = (0, 0, 255)
    GREEN = (0, 255, 0)

    def cm_to_img(self, coord):
        return (int(coord[0] / self.x_sc * self.scale),
                int((self.y_sz - coord[1]) / self.y_sc * self.scale))

    def show(self, pos=None, scale=16):
        ht, wd = self.grid.shape[0:2]
        img = np.zeros([scale * ht, scale * wd, 3], np.uint8)
        for y in xrange(ht):
            for x in xrange(wd):
                img[y*scale:(y+1)*scale, x*scale:(x+1)*scale, :] = self.grid[y, x]
        for food_pos in self.food:
            coords = self.cm_to_img(food_pos)
            cv2.circle(img, coords, scale / 2, self.RED, -1)
        if pos:
            coord = self.cm_to_img(pos)
            cv2.circle(img, coord, scale * 3 / 2, self.GREEN, -1)
        cv2.imshow('Arena', img)
        cv2.waitKey(300)

def main(argv):
    img = cv2.imread(argv[0])
    # (56, 30) corresponds roughly to 2.5 x 2.5cm blocks
    og = occ_grid.make_occ_grid(img, 56, 30, thresh=.5)
    arena = Arena(og)
    arena.food.append((100, 10))
    arena.food.append((75, 50))
    arena.show((67, 15))

if __name__ == '__main__':
    main(sys.argv[1:])
