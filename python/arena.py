# Stores an occupancy grid and a list of food positions in cm.
# Gives information about where obstacles and food are.
# Displays arena with robot and food marked.
# Catches key presses and updates food list.

import cv2
import numpy as np

import occ_grid


class Arena:
    def __init__(self, occ_grid, x_sz=139.5, y_sz=75.5):
        self.grid = occ_grid

        self.robot_x = 67
        self.robot_y = 15
        self.robot_angle = 90

        self.home_x = 67
        self.home_y = 15

        self.x_sz = x_sz
        self.y_sz = y_sz
        ht, wd = self.grid.shape[0:2]

        # scale
        self.x_sc = self.x_sz / float(wd)
        self.y_sc = self.y_sz / float(ht)

        self.pf_robot_x = 67
        self.pf_robot_y = 15

        # display scale
        self.scale = 16
        self.particles = []
        self.food = []

        # life_size = occ_grid.make_occ_grid('arena_16_small.bmp', 140, 76, thresh=.5)
        # self.life_size_grid = life_size[::-1]

    BLUE = (255, 0, 0)
    RED = (0, 0, 255)
    GREEN = (0, 255, 0)
    PURPLE = (57, 18, 76)

    def cm_to_img(self, coord, scale=None):
        if not scale:
            scale = self.scale
        ht, wd = self.grid.shape[0:2]
        return (int(coord[0] / self.x_sc * scale),
                ht * scale - int(coord[1] / self.y_sc * scale))

    def cm_to_grid(self, coord):
        return (int(coord[0] / self.x_sc),
                int((coord[1]) / self.y_sc))

    def distance_to_collision(self, x, y, angle):

        rad = angle * np.pi / 180

        for i in range(1, 1000):
            col_x = int(np.round(x + i * np.cos(rad)))
            col_y = int(np.round(y + i * np.sin(rad)))

            if col_x >= len(self.grid) or col_y >= len(self.grid[0]) or col_x < 0 or col_y < 0:
                return i

            if self.grid[col_x][col_y] == 0:
                return i

        raise Exception("no collision detected: x:{} y:{} angle:{}")

    def get_robot_in_grid(self):
        return self.cm_to_grid((self.robot_x, self.robot_y))

    def show(self, scale=5, wait_time=200):
        ht, wd = self.grid.shape[0:2]
        img = np.zeros([scale * ht, scale * wd, 3], np.uint8)
        for y in xrange(ht):
            for x in xrange(wd):
                img[y*scale:(y+1)*scale, x*scale:(x+1)*scale, :] = self.grid[ht-y-1, x]

        for food_pos in self.food:
            coords = self.cm_to_img(food_pos, scale=5)
            cv2.circle(img, coords, scale, self.RED, -1)

        for particle_pos in self.particles:
            coords = self.cm_to_img(particle_pos, scale=3)
            cv2.circle(img, coords, scale, self.RED, -1)

        coord = self.cm_to_img((self.robot_x, self.robot_y))
        pf_coord = self.cm_to_img((self.pf_robot_x, self.pf_robot_y))

        cv2.circle(img, coord, scale * 3 / 2, self.GREEN, -1)
        cv2.circle(img, pf_coord, scale * 3 / 2, self.PURPLE, -1)

        cv2.imshow('Arena', img)
        cv2.waitKey(1)

    def add_angle(self, angle):
        self.robot_angle = (self.robot_angle + angle) % 360

    def add_straight(self, cm):
        self.robot_x += cm * np.cos(self.robot_angle * np.pi / 180)
        self.robot_y += cm * np.sin(self.robot_angle * np.pi / 180)

    def mark_food(self):
        # Avoid duplicate food locations
        for food_loc in self.food:
            if np.sqrt((food_loc[0] - self.robot_x) *
                       (food_loc[0] - self.robot_x) +
                       (food_loc[1] - self.robot_y) *
                       (food_loc[1] - self.robot_y)) < 20:
                self.food.remove(food_loc)
        self.food.append((self.robot_x, self.robot_y))


def build_arena(img):
    img = cv2.imread(img)

    # (56, 30) corresponds roughly to 2.5 x 2.5cm blocks
    og = occ_grid.make_occ_grid(img, 140, 76, thresh=.5)
    arena = Arena(og[::-1])

    # arena.particles.append((5, 5))

    # arena.show()
    return arena

if __name__ == '__main__':
    build_arena('arena_16_small.bmp')
