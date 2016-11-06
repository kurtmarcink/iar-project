# Stores an occupancy grid and a list of food positions in cm.
# Gives information about where obstacles and food are.
# Displays arena with robot and food marked.
# Catches key presses and updates food list.

import numpy as np
import cv2
import occ_grid


class Arena:
    def __init__(self, occ_grid, x_sz=139.5, y_sz=75.5):
        self.grid = occ_grid

        self.robot_x = 67
        self.robot_y = 15
        self.robot_angle = 0

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

    def cm_to_grid(self, coord):
        return (int(coord[0] / self.x_sc),
                int((coord[1]) / self.y_sc))

    def get_landmarks_in_grid(self):
        landmarks = []
        for i in range(0, len(self.grid)):
            for j in range(0, len(self.grid[i])):
                if self.grid[i][j] == 0:
                    # from (y, x) to (x, y)
                    landmarks.append((j, i))

        for i in range(0, len(self.grid[0])):
            landmarks.append((i, -1))

        for i in range(0, len(self.grid)):
            landmarks.append((len(self.grid[0]) - 1, i))

        return landmarks

    def get_robot_in_grid(self):
        return self.cm_to_grid((self.robot_x, self.robot_y))

    def show(self, scale=16):
        ht, wd = self.grid.shape[0:2]
        img = np.zeros([scale * ht, scale * wd, 3], np.uint8)
        for y in xrange(ht):
            for x in xrange(wd):
                img[y*scale:(y+1)*scale, x*scale:(x+1)*scale, :] = self.grid[ht-y-1, x]

        for food_pos in self.food:
            coords = self.cm_to_img(food_pos)
            cv2.circle(img, coords, scale / 2, self.RED, -1)

        coord = self.cm_to_img((self.robot_x, self.robot_y))

        cv2.circle(img, coord, scale * 3 / 2, self.GREEN, -1)

        cv2.imshow('Arena', img)
        cv2.waitKey(0)


def build_arena(img):
    img = cv2.imread(img)

    # (56, 30) corresponds roughly to 2.5 x 2.5cm blocks
    og = occ_grid.make_occ_grid(img, 56, 30, thresh=.5)
    arena = Arena(og[::-1])

    # arena.show()
    return arena

if __name__ == '__main__':
    build_arena('arena_16_small.bmp')
