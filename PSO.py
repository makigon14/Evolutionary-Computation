# -*- coding: utf-8 -*-
"""
Created on Mon Oct 15 00:45:25 2018

@author: makigondesk
"""

import random
import numpy as np
import math
import copy

# v_i(t+1) = wv_i(t) + c1(g-x_i) * rand(0,1) + c2(p_i-x_i) * rand(0,1)
# x_i(t+1) = x_i(t) + v_i(t+1)
w = 0.729844  # Inertia weight to prevent velocities becoming too large
c1 = 1.496180  # Scaling co-efficient on the social component
c2 = 1.496180  # Scaling co-efficient on the cognitive component
dimension = 2  # Size of the problem
iterations = 3000
swarmSize = 20


class Particle:

    def __init__(self):
        self.velocity = []
        self.pos = []
        self.pBest = []
        for i in range(dimension):
            self.pos.append(random.uniform(-5.12, 5.12))
            self.velocity.append(0.1 * random.random())
            self.pBest.append(self.pos[i])
        return

    def updatePositions(self):
        for i in range(dimension):
            self.pos[i] = self.pos[i] + self.velocity[i]
        return

    def updateVelocities(self, gBest):  # g:globalbest p:personalbest
        for i in range(dimension):
            r1 = random.random()
            r2 = random.random()
            social = c1 * r1 * (gBest[i] - self.pos[i])
            cognitive = c2 * r2 * (self.pBest[i] - self.pos[i])
            self.velocity[i] = (w * self.velocity[i]) + social + cognitive
        return

    # def satisfyConstraints(self):
    # do not use this time
    # return


# This class contains the particle swarm optimization algorithm
class ParticleSwarmOptimizer:
    # solution=[]
    # swarm=[]
    def __init__(self):
        self.solution = []
        self.swarm = []
        for h in range(swarmSize):
            particle = Particle()
            self.swarm.append(particle)

        #return

    def optimize(self):
        gBest = self.swarm[0].pBest
        sol = []
        sol.append(gBest)
        xrealgBest = copy.deepcopy(sol[0])
        yrealgBest = self.f2(sol[0])
        for i in range(iterations):

            # Get the global best particle
            for j in range(swarmSize):
                p = self.swarm[j].pBest
                if self.f2(p) < self.f2(gBest):
                    gBest = copy.deepcopy(p)
                    #gBest = p
            sol.append(gBest)
            if i == 0:
                sol[0] = gBest
                xrealgBest = sol[0]
                yrealgBest = self.f2(sol[0])
            for t in range(i, i + 1):
                if self.f2(sol[t]) < yrealgBest:
                    xrealgBest = copy.deepcopy(sol[t])
                    yrealgBest = self.f2(sol[t])
                    print(xrealgBest)
                print("sdfh" + str(xrealgBest))
            print("ui"+str(xrealgBest))
            print("df" + str(gBest))

            # Update position of each paricle
            for k in range(swarmSize):
                self.swarm[k].updateVelocities(gBest)
                self.swarm[k].updatePositions()
                # self.swarm[k].satisfyConstraints()
            # Update the personal best positions
                #print(yrealgBest, xrealgBest)

            for l in range(swarmSize):
                q = self.swarm[l].pBest
                if self.f2(self.swarm[l].pos) < self.f2(q):
                    self.swarm[l].pBest = self.swarm[l].pos
            print(yrealgBest, xrealgBest)
            # print(gBest)
        # return solution


    def f(self, solution):  # Sphere function

        np_sol = np.array(solution)
        np_sol = np_sol ** 2

        return np.sum(np_sol)

    def f2(self, solution):  # Rastrigin function
        y = 0
        # for m in solution:
        # if m > 5.12 or m < -5.12:
        # m = 10000
        # return m

        for i in range(dimension):
            y = y + solution[i] ** 2 - 10 * math.cos(2 * math.pi * solution[i])

        return 10 * dimension + y


def main():
    pso = ParticleSwarmOptimizer()
    pso.optimize()
    # print(pso.f2(gBest))


if __name__ == '__main__':
    main()