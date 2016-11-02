#!/usr/bin/python

import sys
import numpy as np
import cv2

def make_occ_grid(img, n_x, n_y, thresh=.95):
    img /= 255
    ht, wd, dp = img.shape
    x_sz = int(wd / n_x)
    y_sz = int(ht / n_y)
    occ_grid = np.zeros([n_y, n_x])
    for x in xrange(n_x):
        for y in xrange(n_y):
            block = img[y*y_sz:(y+1)*y_sz, x*x_sz:(x+1)*x_sz, 0]
            lightness = block.sum() / float(block.size)
            if lightness > thresh:
                occ_grid[y, x] = 255
    return occ_grid

def show_occ_grid(occ_grid, scale):
    ht, wd = occ_grid.shape[0:2]
    img = np.zeros([scale * ht, scale * wd], np.uint8)
    for y in xrange(ht):
        for x in xrange(wd):
            img[y*scale:(y+1)*scale, x*scale:(x+1)*scale] = occ_grid[y, x]
    cv2.imshow('Occupancy grid', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def main(argv):
    img = cv2.imread(argv[0])
    # (56, 30) corresponds roughly to 2.5 x 2.5cm blocks
    occ_grid = make_occ_grid(img, 56, 30, thresh=.5)
    # Zoom the occupancy grid so it is visible
    show_occ_grid(occ_grid, 16)

if __name__ == '__main__':
    main(sys.argv[1:])
