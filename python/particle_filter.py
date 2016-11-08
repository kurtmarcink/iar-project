# import matplotlib.pyplot as plt
import numpy as np
import scipy
import scipy.stats

from filterpy.monte_carlo import systematic_resample
from numpy.linalg import norm
from numpy.random import randn
from numpy.random import seed
from util import degrees_to_rad


# TODO: landmark measurements, integration with robot commands
class ParticleFilter:
    def __init__(self, particle_count, arena):
        seed(2)

        self.particle_count = particle_count
        self.arena = arena
        self.life_size_grid = arena.grid

        self.particles = self.create_particles()
        self.weights = np.zeros(self.particle_count)

        self.arena.particles = self.particles

    def predict(self, particles, u, std, dt=1.):
        """ move according to control input u (heading change, velocity)
        with noise Q (std heading change, std velocity)`"""

        N = len(particles)
        # update heading
        particles[:, 2] += u[0] + (randn(N) * std[0])
        particles[:, 2] %= 360

        # move in the (noisy) commanded direction
        dist = (u[1] * dt) + (randn(N) * std[1])
        particles[:, 0] += np.cos(particles[:, 2]) * dist
        particles[:, 1] += np.sin(particles[:, 2]) * dist

        return particles

    def update(self, distance, R):
        self.weights.fill(1.)

        distances = np.empty([self.particle_count])

        # TODO: add in readings of other sensor values!!

        for i, p in enumerate(self.particles):
            coord_in_cm = self.arena.cm_to_grid((p[0], p[1]))

            particle_dist = self.arena.distance_to_collision(coord_in_cm[0], coord_in_cm[1], p[2])

            if particle_dist > 5:
                particle_dist = 5.5

            distances[i] = particle_dist

        self.weights *= scipy.stats.norm(distances, R).pdf(distance)

        self.weights += 1.e-300  # avoid round-off to zero
        self.weights /= sum(self.weights)  # normalize

    # def update(self, particles, weights, z, R, landmarks):
    #     weights.fill(1.)
    #     for i, landmark in enumerate(landmarks):
    #         distance = np.linalg.norm(particles[:, 0:2] - landmark, axis=1)
    #         weights *= scipy.stats.norm(distance, R).pdf(z[i])
    #
    #     weights += 1.e-300  # avoid round-off to zero
    #     weights /= sum(weights)  # normalize

    def estimate(self, particles, weights):
        """returns mean and variance of the weighted particles"""

        pos = particles[:, 0:2]
        mean = np.average(pos, weights=weights, axis=0)
        var = np.average((pos - mean) ** 2, weights=weights, axis=0)
        return mean, var

    def neff(self, weights):
        return 1. / np.sum(np.square(weights))

    def resample_from_index(self, particles, weights, indexes):
        particles[:] = particles[indexes]
        weights[:] = weights[indexes]
        weights /= np.sum(weights)

    def create_particles(self):
        particles = np.empty((self.particle_count, 3))

        particles[:, 0] = 67
        particles[:, 1] = 15
        particles[:,  2] = np.random.random_integers(0, 359, self.particle_count)
        return particles

    def go(self, movement, angle, sensor_distance, sensor_std_err=.5):

        # TODO
        # print "SENSOR DISTANCE: " + str(sensor_distance)

        self.particles = self.predict(self.particles, u=(angle, movement), std=(.2, .05))

        self.arena.particles = self.particles

        self.update(sensor_distance, sensor_std_err)

        if self.neff(self.weights) < self.particle_count / 2:
            indexes = systematic_resample(self.weights)
            self.resample_from_index(self.particles, self.weights, indexes)

        mu, var = self.estimate(self.particles, self.weights)
        return mu

