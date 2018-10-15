# -*- coding: utf-8 -*-
"""
Created on Mon Oct 15 00:45:25 2018

@author: makigondesk
"""

# Portfolio optimization using particle swarm optimization article - PSO bare bones code
 
import random
import numpy as np
 
w = 0.729844 # Inertia weight to prevent velocities becoming too large
c1 = 1.496180 # Scaling co-efficient on the social component
c2 = 1.496180 # Scaling co-efficient on the cognitive component
dimension = 3 # Size of the problem
iterations = 1000
swarmSize = 30
 
# This class contains the code of the Particles in the swarm
class Particle:

 
    def __init__(self):
        self.velocity=[]
        self.pos=[]
        self.pBest=[]
        for i in range(dimension):
            self.pos.append(30*random.random())
            self.velocity.append(0.01 * random.random())
            self.pBest.append(self.pos[i])
        return
 
    def updatePositions(self):
        for i in range(dimension):
            self.pos[i] = self.pos[i] + self.velocity[i]   
        return
 
    def updateVelocities(self, gBest):
        for i in range(dimension):
            r1 = random.random()
            r2 = random.random()
            social = c1 * r1 * (gBest[i] - self.pos[i])
            cognitive = c2 * r2 * (self.pBest[i] - self.pos[i])
            self.velocity[i] = (w * self.velocity[i]) + social + cognitive
        return
 
    def satisfyConstraints(self):
        #This is where constraints are satisfied
        return
 
# This class contains the particle swarm optimization algorithm
class ParticleSwarmOptimizer:
    solution=[]
    swarm=[]
    def __init__(self):
        #self.solution = []
        #self.swarm = []
        for h in range(swarmSize):
            particle = Particle()
            self.swarm.append(particle)

        return
 
    def optimize(self):
        gBest = self.swarm[0].pBest
        for i in range(iterations):
            
           
            #Get the global best particle
            for j in range(swarmSize):
                pBest = self.swarm[j].pBest
                if self.f(pBest) <self.f(gBest):
                    gBest = pBest  
            solution = gBest
            #Update position of each paricle
            for k in range(swarmSize):
                self.swarm[k].updateVelocities(gBest)
                self.swarm[k].updatePositions()
                self.swarm[k].satisfyConstraints()
            #Update the personal best positions
            for l in range(swarmSize):
                pBest = self.swarm[l].pBest
                if self.f(self.swarm[l].pos) <self.f(pBest):
                    self.swarm[l].pBest = self.swarm[l].pos
            print(self.f(gBest))
            #print(gBest)
        return solution
 
    def f(self, solution):
        #This is where the metaheuristic is defined
        
        np_sol=np.array(solution)
        np_sol=np_sol**2
        
        return np.sum(np_sol)  
 
def main():
    pso = ParticleSwarmOptimizer()
    pso.optimize()
 
if  __name__ =='__main__':
    main()