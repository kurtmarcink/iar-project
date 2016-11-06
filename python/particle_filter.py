import numpy as np
import matplotlib.pyplot as plt
import numpy.random
import scipy
from filterpy.monte_carlo import systematic_resample
from numpy.linalg import norm
from numpy.random import randn
import scipy.stats
from numpy.random import seed


# TODO: landmark measurements, integration with robot commands
class ParticleFilter:
    def __init__(self, initial_pose, landmarks, particle_count):
        seed(2)

        self.landmarks = landmarks
        self.initial_pose = initial_pose
        self.particle_count = particle_count

        self.particles = self.create_particles(mean=self.initial_pose, std=(5, 5, np.pi / 4))
        self.weights = np.zeros(self.particle_count)

    def predict(self, particles, u, std, dt=1.):
        """ move according to control input u (heading change, velocity)
        with noise Q (std heading change, std velocity)`"""

        N = len(particles)
        # update heading
        particles[:, 2] += u[0] + (numpy.random.randn(N) * std[0])
        particles[:, 2] %= 2 * np.pi

        # move in the (noisy) commanded direction
        dist = (u[1] * dt) + (numpy.random.randn(N) * std[1])
        particles[:, 0] += np.cos(particles[:, 2]) * dist
        particles[:, 1] += np.sin(particles[:, 2]) * dist

    def update(self, particles, weights, z, R, landmarks):
        weights.fill(1.)
        for i, landmark in enumerate(landmarks):
            distance = np.linalg.norm(particles[:, 0:2] - landmark, axis=1)
            weights *= scipy.stats.norm(distance, R).pdf(z[i])

        weights += 1.e-300  # avoid round-off to zero
        weights /= sum(weights)  # normalize

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

    def create_particles(self, mean, std):
        particles = np.empty((self.particle_count, 3))
        particles[:, 0] = mean[0] + (randn(self.particle_count) * std[0])
        particles[:, 1] = mean[1] + (randn(self.particle_count) * std[1])
        particles[:, 2] = mean[2] + (randn(self.particle_count) * std[2])
        particles[:, 2] %= 2 * np.pi

        return particles

    def run_pf1(self, iters=18, sensor_std_err=.1, plot_particles=False, xlim=(0, 20), ylim=(0, 20)):

        plt.figure()

        if plot_particles:
            alpha = .20

            if self.particle_count > 5000:
                alpha *= np.sqrt(5000) / np.sqrt(self.particle_count)

            plt.scatter(self.particles[:, 0], self.particles[:, 1], alpha=alpha, color='g')

        xs = []
        robot_pos = np.array([0., 0.])
        for x in range(iters):
            robot_pos += (1, 1)

            # distance from robot to each landmark
            # TODO
            zs = (norm(self.landmarks - robot_pos, axis=1) + (randn(len(self.landmarks)) * sensor_std_err))

            # move diagonally forward to (x+1, x+1)
            self.predict(self.particles, u=(0.00, 1.414), std=(.2, .05))

            # incorporate measurements
            self.update(self.particles, self.weights, z=zs, R=sensor_std_err, landmarks=self.landmarks)

            # resample if too few effective particles
            if self.neff(self.weights) < self.particle_count / 2:
                indexes = systematic_resample(self.weights)
                self.resample_from_index(self.particles, self.weights, indexes)

            mu, var = self.estimate(self.particles, self.weights)
            xs.append(mu)

            if plot_particles:
                plt.scatter(self.particles[:, 0], self.particles[:, 1], color='k', marker=',', s=1)
            p1 = plt.scatter(robot_pos[0], robot_pos[1], marker='+', color='k', s=180, lw=3)
            p2 = plt.scatter(mu[0], mu[1], marker='s', color='r')

        xs = np.array(xs)
        # plt.plot(xs[:, 0], xs[:, 1])
        plt.legend([p1, p2], ['Actual', 'PF'], loc=4, numpoints=1)
        plt.xlim(*xlim)
        plt.ylim(*ylim)
        print('final position error, variance:\n\t', mu, var)
        plt.show()

# run_pf1(N=5000, plot_particles=True, ylim=(-20, 20), self.initial_pose=(1, 1, 0))

