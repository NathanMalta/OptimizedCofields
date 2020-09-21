import random
import math
import matplotlib.pyplot as plt
import copy

from Simulator import Simulator

class simulatedAnnealing():
	def __init__(self):
		maxStartPos = 0
		self.dimSize = 18
		self.initTemp = 10000
		self.temp = self.initTemp
		self.iter = 0
		self.time = 300 #start at 200 to avoid complete random walk phase
		self.stepSize = 0.2
		self.currentScore = 1E100
		self.currentPos = []
		for _ in range(self.dimSize):
			self.currentPos.append(random.uniform(-maxStartPos, maxStartPos))


	def update(self):
		potentialNewPos = self.getNeighbor()
		newScore = self.getScore(potentialNewPos, self.iter)
		deltaE = newScore - self.currentScore
		if deltaE <= 0:
			self.currentPos = potentialNewPos
			self.currentScore = newScore
		else:
			probability = math.exp(-deltaE / self.temp * 0.1)
			if random.uniform(0, 1) < probability:
				self.currentPos = potentialNewPos
				self.currentScore = newScore


		#decay temperature as a function of time
		self.temp = self.initTemp * 0.98 ** self.time #exponential temp function
		# self.temp = self.initTemp/(1 + math.log(self.time + 1.1)) #logerithmic temp function
		# self.temp = self.initTemp - 1E1 * self.time
		self.time += 0.1
		self.iter += 1

	def getNeighbor(self):
		increasing = random.choice([True, False])
		dimToChange = random.choice(range(0, self.dimSize))
		newPos = copy.deepcopy(self.currentPos)
		if increasing:
			newPos[dimToChange] += self.stepSize
		else:
			newPos[dimToChange] -= self.stepSize
		return newPos


	def getScore(self, pos, it):
		'''Determines how close a particle is to the optima - lower is better
		'''
		uavPoly = pos[:len(pos)//2]
		targetPoly = pos[len(pos)//2:]
		sim = Simulator(uavPoly, targetPoly, f"test{it}")
		score = sim.runTest()
		return score



if __name__ == '__main__':
	simAnn = simulatedAnnealing()
	while  simAnn.time < 1000:
		simAnn.update()
		print(simAnn.time, simAnn.temp, simAnn.currentScore)
		print(simAnn.currentPos)
