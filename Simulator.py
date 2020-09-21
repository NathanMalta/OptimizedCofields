from enum import Enum
import math
import matplotlib.pyplot as plt
import time
import pickle
import random
import copy

import Constants

class Simulator():
	def __init__(self, uavPolyRaw, targetPolyRaw, testName):
		#load simulation constants from constants file
		self.numUAVs = Constants.NUM_UAVS
		self.numTargets = Constants.NUM_TARGETS

		self.uavSpeed = Constants.UAV_SPEED
		self.targetSpeed = Constants.TARGET_SPEED
		
		self.uavPoly = Polynomial(uavPolyRaw)
		self.targetPoly = Polynomial(targetPolyRaw)

		self.testName = testName

		self.maxTestTime = Constants.MAX_TEST_TIME
		self.simDt = Constants.SIM_DT
		self.simSideLen = Constants.SIM_SIDE_LEN

		self.commsDistance = Constants.COMM_DISTANCE
		self.uavDetectionDist = Constants.UAV_DETECTION_DIST
		self.targetDetectionDist = Constants.TARGET_DETECTION_DIST

		self.simStates = SimStates(self.commsDistance, self.uavDetectionDist, self.targetDetectionDist, uavPolyRaw, targetPolyRaw)

		#create dynamic simulation variables
		self.time = 0
		self.uavs = []
		self.targets = []

		for i in range(self.numUAVs):
			self.uavs.append(UAV(i, copy.deepcopy(Constants.UAV_LAYDOWN[i]), self.uavSpeed))


		for i in range(self.numTargets):
			self.targets.append(Target(self.numUAVs + i, copy.deepcopy(Constants.TARGET_LAYDOWN[i]), self.targetSpeed))

	def runTest(self):
		infoScores = []
		startTime = time.time()
		while self.time < self.maxTestTime:
			#run the test for the desired duration
			self.time += self.simDt 
			self.updateWorldModels() #update the world model information for all targets and uavs
			self.updateHeadings() #update the headings of UAVs (via DCF) and targets (via random walk)
			self.updatePositions() #update the positions of the uavs and targets based on their constant velocity and calculated headings\
			score = self.getInfoScore(self.time)
			infoScores.append(score) #record info score
			self.simStates.append(self.uavs, self.targets, self.time, score) #record state of simulation

		testTime = time.time() - startTime 
		print(f"test time: {testTime} simRatio: {50/testTime}")

		self.saveResults() #save the information surrounding the test so it can be replayed later

		#determine average info score over the test
		totalScore = 0
		for score in infoScores:
			totalScore += score
		avgScore = totalScore / len(infoScores)
		return avgScore

	def updateWorldModels(self):
		'''See what each UAV can sense/communicate with to update its worldModel
		'''
		#add uavs and targets within detection range into the world model
		for ownship in self.uavs:
			for detection in self.uavs:
				if ownship.uavID == detection.uavID:
					continue
				if ownship.getDistance(detection) <= self.uavDetectionDist:
					ownship.worldModel.addToWorldModel(WorldObject(detection.uavID, detection.pos, self.time, WorldObjectType.UAV))

			for detection in self.targets:
				if ownship.getDistance(detection) <= self.targetDetectionDist:
					ownship.worldModel.addToWorldModel(WorldObject(detection.targetID, detection.pos, self.time, WorldObjectType.TARGET))

		#add current uav positions to path point
		for ownship in self.uavs:
			ownship.worldModel.addPathPoint(PathPoint(ownship.pos, self.time))
			ownship.worldModel.removeOldPathPoints(self.time)

		for ownship in self.uavs:
			for commRobot in self.uavs:
				if ownship.uavID == commRobot.uavID:
					continue
				if ownship.getDistance(commRobot) <= self.commsDistance:
					ownship.worldModel.combineModel(commRobot.worldModel)

	def updateHeadings(self):
		'''apply DCF with the given functions to produce an output heading for the UAVs
		'''
		for uav in self.uavs:
			uav.updateDCFPlanning(self.uavPoly, self.targetPoly, self.time)
		for target in self.targets:
			target.updateRandomWalk()

	def updatePositions(self):
		'''Use a simple physics model to update the positions of the UAVs based on their headings and constant velocites 
		'''
		for uav in self.uavs:
			uav.updatePos(self.simDt)
		for target in self.targets:
			target.updatePos(self.simDt)

	def getInfoScore(self, currentTime):
		'''Produce an info score based on the average time since detection for all objects in all worldmodels
		   Gives an idea of how well the algorithm is working
		'''
		timeTotal = 0
		for uav in self.uavs:
			timeTotal += uav.potentialLocationsScore
		return timeTotal / self.numUAVs

	def saveResults(self):
		with open(self.testName + ".simLog",'wb') as saveFile:
			pickle.dump(self.simStates, saveFile)

class UAV():
	def __init__(self, uavID, pos, vel):
		self.uavID = uavID
		self.pos = pos
		self.vel = vel
		self.heading = 0
		self.worldModel = WorldModel(uavID)
		self.potentialLocationsScore = 0

	def getDistance(self, uavOrTarget):
		'''Returns the distance from this uav to the uav or target in question
		'''
		dx = uavOrTarget.pos[0]-self.pos[0]
		dy = uavOrTarget.pos[1]-self.pos[1]
		return math.sqrt(dx**2 + dy**2)

	def updateDCFPlanning(self, uavPoly, targetPoly, simTime):
		xForce = 0
		yForce = 0
		self.potentialLocationsScore = 0
		for uavID, uav in self.worldModel.uavs.items():
			possiblePts = self.worldModel.decomposeObject(uavID, simTime) #decompose the uav into a list of points where the UAV could exist
			self.potentialLocationsScore += len(possiblePts)
			uavXForce = 0
			uavYForce = 0
			for pt in possiblePts:
				timeDiff = simTime - uav.detectionTime

				dx = self.pos[0] - pt[0]
				dy = self.pos[1] - pt[1]
				dist = math.hypot(dx, dy)

				forceMag = uavPoly.evaluate(dist, timeDiff)

				xUnit = dx / dist
				yUnit = dy / dist

				uavXForce += forceMag * xUnit
				uavYForce += forceMag * yUnit
			if len(possiblePts) > 0:
				xForce += (uavXForce / len(possiblePts))
				yForce += (uavYForce / len(possiblePts))

		for targetID, target in self.worldModel.targets.items():
			possiblePts = self.worldModel.decomposeObject(targetID, simTime) #decompose the uav into a list of points where the UAV could exist
			self.potentialLocationsScore += len(possiblePts)
			targXForce = 0
			targYForce = 0
			for pt in possiblePts:
				timeDiff = simTime - target.detectionTime

				dx = pt[0] - self.pos[0]
				dy = pt[1] - self.pos[1]
				dist = math.hypot(dx, dy)

				forceMag = targetPoly.evaluate(dist, timeDiff)

				xUnit = dx / dist
				yUnit = dy / dist

				targXForce += forceMag * xUnit
				targYForce += forceMag * yUnit
			if len(possiblePts) > 0:
				xForce += (targXForce / len(possiblePts))
				yForce += (targYForce / len(possiblePts))

		self.heading = math.atan2(yForce, xForce)

	def updatePos(self, simDt):
		self.pos[0] += math.cos(self.heading) * simDt * self.vel
		self.pos[1] += math.sin(self.heading) * simDt * self.vel

class Target():
	def __init__(self, targetID, pos, vel):
		self.targetID = targetID
		self.pos = pos
		self.vel = vel
		self.heading = 0
		self.targetPoint = self.pos
		self.targetPoint = random.choice(self.getPossibleTargetPoints())
		self.targetPointTol = 1

	def getDistance(self, uavOrTarget):
		'''Returns the distance from this uav to the uav or target in question
		'''
		dx = uavOrTarget.pos[0]-self.pos[0]
		dy = uavOrTarget.pos[1]-self.pos[1]
		return math.sqrt(dx**2 + dy**2)

	def getPossibleTargetPoints(self):
		'''gets possible points one block away from current position of the target
		'''
		possiblePts = []
		if self.targetPoint[0] + 2 <= 20:
			possiblePts.append([self.targetPoint[0]+2, self.targetPoint[1]])
		if self.targetPoint[1] + 2 <= 20:
			possiblePts.append([self.targetPoint[0], self.targetPoint[1] + 2])
		if self.targetPoint[0] - 2 >= -20:
			possiblePts.append([self.targetPoint[0]-2, self.targetPoint[1]])
		if self.targetPoint[1] - 2 >= -20:
			possiblePts.append([self.targetPoint[0], self.targetPoint[1]-2])

		if len(possiblePts) <= 0:
			possiblePts.append([0,0])

		return possiblePts

	def updateRandomWalk(self):
		''' Calculates the heading required to get to the target position.  Calculates a new target position if the old one is within a tolerance
		'''
		distToTarget = math.hypot(self.targetPoint[0]-self.pos[0], self.targetPoint[1]-self.pos[1])
		if distToTarget <= self.targetPointTol:
			self.targetPoint = random.choice(self.getPossibleTargetPoints())
		self.heading = math.atan2(self.targetPoint[1] - self.pos[1], self.targetPoint[0] - self.pos[0])

	def updatePos(self, simDt):
		# self.pos[0] += math.cos(self.heading) * simDt * self.vel
		# self.pos[1] += math.sin(self.heading) * simDt * self.vel
		pass

class WorldModel():
	def __init__(self, ownshipID):
		self.numUAVs = Constants.NUM_UAVS
		self.numTargets = Constants.NUM_TARGETS
		self.ownshipID = ownshipID

		self.uavSpeed = Constants.UAV_SPEED
		self.targetSpeed = Constants.TARGET_SPEED


		#produce list of UAVs and Targets representing no information
		self.uavs = {}
		self.targets = {}

		for i in range(self.numUAVs):
			if i == ownshipID:
				continue #don't add ownship to the world model
			self.uavs[i] = WorldObject(i, [0,0], -100, WorldObjectType.UAV)

		for i in range(self.numTargets):
			self.targets[self.numUAVs + i] = WorldObject(self.numUAVs + i, [0,0], -100, WorldObjectType.TARGET)

		#make an empty array of path points to hold negative info
		self.pathPoints = []

	def addToWorldModel(self, detectedObject):
		'''Updates the world model to include the detected object
		'''
		if detectedObject.objID == self.ownshipID:
			return

		if detectedObject.objType == WorldObjectType.UAV:
			#the detected object is a uav
			if self.uavs[detectedObject.objID].detectionTime < detectedObject.detectionTime:
				#the observation is newer than the one currently in the world model
				self.uavs[detectedObject.objID] = detectedObject
		else:
			#the detected object is a target
			if self.targets[detectedObject.objID].detectionTime < detectedObject.detectionTime:
				#the observation is newer than the one currently in the world model
				self.targets[detectedObject.objID] = detectedObject

	def addPathPoint(self, newPoint):
		'''Add a path point (previous UAV location where any detections would be known) to the model
		'''
		#check if there is a close path point already in the model
		for i in range(len(self.pathPoints)):
			point = self.pathPoints[i]
			if newPoint.getDistance(point) < 1:
				#the points are close enough to be considered in the same spot
				if point.timeRecorded < newPoint.timeRecorded:
					#a similar older point is already in the list, replace the older point
					self.pathPoints[i] = newPoint
				#otherwise, a newer point is already in the list - do nothing
				return #in either case, do not continue

		#the list does not contain any similar points - add the new path point
		self.pathPoints.append(newPoint)

	def combineModel(self, otherModel):
		'''incorperates all information from another world model into this model - used when UAVs share world models
		'''
		for _, uav in otherModel.uavs.items():
			self.addToWorldModel(uav)

		for _, target in otherModel.targets.items():
			self.addToWorldModel(target)

		for pathPoint in otherModel.pathPoints:
			self.addPathPoint(pathPoint)

	def getAvgDetectionTime(self):
		'''Gets the average detection time of all world objects - useful as a metric of how good the model is
		'''
		timeTotal = 0
		for _, uav in self.uavs.items():
			timeTotal += uav.detectionTime

		for _, target in self.targets.items():
			timeTotal += target.detectionTime

		return timeTotal / (self.numUAVs + self.numTargets-1)

	def removeOldPathPoints(self, simTime):
		i = 0
		while i < len(self.pathPoints):
			if self.pathPoints[i].getRadius(simTime) < 1E-5:
				self.pathPoints.pop(i)
			else:
				i += 1

	def decomposeObject(self, objID, simTime):
		'''Reduces the object with given ID into a list of points representing possible locations for that object, using positive and negative information from the world model
		'''
		if objID == self.ownshipID:
			print("CANNOT DECOMPOSE OWNSHIP!")
			return []

		#grab the world object for the given ID from the world model
		wObj = None
		if objID < self.numUAVs:
			#object is a uav
			wObj = self.uavs[objID]
		elif self.numUAVs <= objID < (self.numUAVs + self.numTargets):
			wObj = self.targets[objID]
		else:
			print("INVAID OBJECT ID FOR DECOMPOSITION")
			return []

		#decompose the object into a list of possible points

		#first decompose positive information into a series of points
		centerPos = wObj.pos
		rad = wObj.getRadius(simTime)
		maxCoord = Constants.SIM_SIDE_LEN

		#if the radius is very small, return a single point
		if rad < 0.5:
			return [centerPos]

		#produce max coordinate set
		maxX = min(centerPos[0] + rad, maxCoord)
		minX = max(centerPos[0] - rad, -maxCoord)
		maxY = min(centerPos[1] + rad, maxCoord)
		minY = max(centerPos[1] - rad, -maxCoord)

		possiblePts = []

		x = minX
		while x <= maxX:
			y = minY
			while y <= maxY:
				#check if the point is in the circle
				dx = centerPos[0] - x
				dy = centerPos[1] - y
				if dx**2 + dy**2 <= rad**2:
					possiblePts.append([x, y])
				y += 1
			x += 1

		#remove negative information from point array
		i = 0

		while i < len(possiblePts):
			didFind = False
			positivePt = possiblePts[i]
			for negativePt in self.pathPoints:
				rad = negativePt.getRadius(simTime)
				dx = positivePt[0] - negativePt.pos[0]
				dy = positivePt[1] - negativePt.pos[1]
				if dx**2 + dy**2 <= rad**2:
					possiblePts.pop(i)
					didFind = True
					break
			if not didFind:
				i += 1

		return possiblePts
		

class WorldObject():
	def __init__(self, objID, pos, detectionTime, objType):
		self.pos = pos
		self.objID = objID
		self.detectionTime = detectionTime
		self.objType = objType

	def getRadius(self, time):
		'''Returns the radius of postive information at this point at a given time
		'''
		growthSpeed = 0
		if self.objType == WorldObjectType.UAV:
			growthSpeed = Constants.UAV_SPEED
		elif self.objType == WorldObjectType.TARGET:
			growthSpeed = Constants.TARGET_SPEED

		dt = time - self.detectionTime
		return growthSpeed * dt


class WorldObjectType(Enum):
	UAV = 0
	TARGET = 1


class PathPoint():
	def __init__(self, pos, timeRecorded):
		self.pos = pos
		self.timeRecorded = timeRecorded

	def getDistance(self, point):
		'''Returns the distance from this uav to the uav or target in question
		'''
		dx = point.pos[0]-self.pos[0]
		dy = point.pos[1]-self.pos[1]
		return math.sqrt(dx**2 + dy**2)

	def getRadius(self, time):
		'''Returns the radius of negative information at this point at a given time
		'''
		decaySpeed = max(Constants.UAV_SPEED, Constants.TARGET_SPEED)
		startRad = min(Constants.UAV_DETECTION_DIST, Constants.TARGET_DETECTION_DIST)
		dt = time - self.timeRecorded
		rad = startRad - decaySpeed * dt
		if rad < 0:
			rad = 0
		return rad

class Polynomial():
	def __init__(self, rawPoly):
		assert(len(rawPoly) % 2 == 1) #the length of the polygon should be odd in order to construct a 2D polynomial
		self.rawPoly = rawPoly
		self.polyOrder = (len(self.rawPoly) + 1)//2 #the magnitude of the max power of the polynomial

	def evaluate(self, distance, infoAge):
		retVal = 0
		if distance == 0:
			distance = 1E-5
		if infoAge == 0:
			infoAge = 1E-5
		for i in range(len(self.rawPoly)):
			if i == 0:
				retVal += self.rawPoly[i] #first value is a constant
			else:
				xPow = i - 2
				yPow = self.polyOrder - i + 2
				retVal += self.rawPoly[i] * (distance ** xPow) * (infoAge ** yPow)
		return retVal

	def __str__(self):
		retVal = ""
		for i in range(len(self.rawPoly)):
			if i == 0:
				retVal += str(round(self.rawPoly[i],4)) #first value is a constant
			else:
				xPow = i - 2
				yPow = self.polyOrder - i + 2
				if xPow == 0:
					retVal += f" + {round(self.rawPoly[i],4)}*i^{yPow}"
				elif yPow == 0:
					retVal += f" + {round(self.rawPoly[i],4)}*d^{xPow}"
				else:
					retVal += f" + {round(self.rawPoly[i],4)}*d^{xPow}*i^{yPow}"
		return retVal

class SimStates():
	def __init__(self, commsDistance, uavDetectionDist, targetDetectionDist, uavPolyRaw, targetPolyRaw):
		self.commsDistance = commsDistance
		self.uavDetectionDist = uavDetectionDist
		self.targetDetectionDist = targetDetectionDist
		self.states = []
		self.uavPolyRaw = uavPolyRaw
		self.targetPolyRaw = targetPolyRaw

	def append(self, uavs, targets, time, infoScore):
		self.states.append(SimState(copy.deepcopy(uavs), copy.deepcopy(targets), time, infoScore))

class SimState():
	def __init__(self, uavs, targets, time, infoScore):
		self.uavs = uavs
		self.targets = targets
		self.time = time
		self.infoScore = infoScore