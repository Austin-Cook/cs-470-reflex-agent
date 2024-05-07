import sys
import os
import time
import math
import numpy

# *****************************************************************************************************************
# 
# 	TODO: Write the function computeTrajectory
#  			robotPos: an array that contains the position (x-coordinate, y-coordinate, orientation)  of the robot in the world
#						Note: orientation is given in degrees
# 	 		goalPos: an array that contains the position (x-coordinate, y-coordinate) of the robot's goal
# 	 		distanceSensors: an array that contains the distance reading (how far to an object) on each of the robot sensors,
# 	 						 which are positioned in a ring around the robot at even distances. 
# 	 						 Examples (for orientation):
# 	 						   distanceSensors[0] gives the distance reading directly to the robot's right
# 							   distanceSensors[4] gives the distance reading direclty in front of the robot
# 							   distanceSensors[8] gives the distance reading direclty to the robot's left
# 							   distanceSensors[12] gives the distance reading direclty behind the robot
# 	 		This function should return a normalized array that specifies the absolute (in world coordinates) 
# 				direction the robot should go in the world as a vector.
# 	 		For example: if you want the robot to go straight to the right, trajectory[0] = 1, trajectory[1] = 0
# 	 		             if you want the robot to go straight up in the world, trajectory[0] = 0, trajectory[1] = 1
#  
#  ****************************************************************************************************************
def computeTrajectory(robotPos, goalPos, distanceSensors):
	goal_radius = 10 # NOTE - the radius must be consistent with that in robotViewer.java ln 1093 (remember to recompile)
	obj_radius = 1
	alpha = 5 # how big a step to take to goal
	beta = 100 # how big a step to take away from obstacle
	goal_spread = 75
	obj_spread = 5
	goal_x = goalPos[0]
	goal_y = goalPos[1]
	robot_x = robotPos[0]
	robot_y = robotPos[1]
	robot_orientation = robotPos[2]
	
	dist = math.sqrt(((goal_x - robot_x) ** 2) + ((goal_y - robot_y) ** 2))
	theta = math.atan((goal_y - robot_y) / (goal_x - robot_x)) 
 	# attractive forces
	# visual representation:
	# outside the spread  |  spread  |  radius  |goal|
	delta_x = 0 # value if inside radius
	delta_y = 0 # value if inside radius
	if goal_radius <= dist and dist <= goal_spread + goal_radius:
		# inside spead
		delta_x = alpha * (dist - goal_radius) * math.cos(theta)
		delta_y = alpha * (dist - goal_radius) * math.sin(theta)
	elif dist > goal_spread + goal_radius: 
		# outside the spread
		delta_x = alpha * goal_spread * math.cos(theta)
		delta_y = alpha * goal_spread * math.sin(theta)
  
	# repulsive forces
	repulsive_x = 0
	repulsive_y = 0
	for i in range(16):
		obj_dist = distanceSensors[i]
		obj_theta = ((((2 * math.pi) / 16) * i) + ((3 * math.pi) / 2) + math.radians(robot_orientation)) % (2 * math.pi)
		print("Sensor ", i, ": theta: ", math.degrees(obj_theta))
		if obj_dist < obj_radius:
			# within object radius - jump away
			repulsive_x += -(numpy.sign(math.cos(obj_theta))) * 10000
			repulsive_y += -(numpy.sign(math.sin(obj_theta))) * 10000
		elif obj_radius <= obj_dist and obj_dist <= obj_spread + obj_radius:
			repulsive_x += -(beta * (obj_spread + obj_radius - obj_dist) * math.cos(obj_theta))
			repulsive_y += -(beta * (obj_spread + obj_radius - obj_dist) * math.sin(obj_theta))
		# if distance to object is greater than obj spread, no repulsive forces detected

	trajectory = []
	trajectory.append(delta_x + repulsive_x)
	trajectory.append(delta_y + repulsive_y)

	# normalize the trajectory vector so that it is a unit vector
	trajectory = normalize(trajectory)
  
	return trajectory


# normalizes the vector
def normalize(v):
	# normalize the vector v
	mag = math.sqrt(v[0]*v[0] + v[1]*v[1])
	if (mag != 0):
		v[0] = v[0] / mag
		v[1] = v[1] / mag

	return v


def endSimulation():
	f = open("../MundoVerdadero/State/sim.txt", "r")

	if f.closed:
		print("sim.txt not found")
		quit()

	msg = f.readline()

	f.close()

	# print(msg[0:3])

	if msg[0:3] == "off":
		return True

	return False


def writeTrajectory(destination, speed, v):
	fnombre = "../Robot/output/" + destination + ".tmp"; 
	f = open(fnombre, "w")

	f.write(str(speed) + "\n")
	f.write(str(v[0]) + "\n")
	f.write(str(v[1]) + "\n")

	f.close()

	# mandato = "mv ../Robot/output/" + destination + ".tmp ../Robot/output/" + destination + ".txt";
	# os.system(mandato)
	fromFile = "../Robot/output/" + destination + ".tmp"
	toFile = "../Robot/output/" + destination + ".txt"
	os.replace(fromFile, toFile)

	return


def readPerception(source):
	fnombre = "../Robot/output/" + source + ".txt"
	try:
		f = open(fnombre, "r")

		if f.closed:
			print("percept.txt not found")
			# quit()
		else:
			robotPos = []
			for i in range(0,3):
				robotPos.append(float(f.readline()))

			chargerPos = []
			for i in range(0,2):
				chargerPos.append(float(f.readline()))

			distanceSensors = []
			for i in range(0,16):
				distanceSensors.append(float(f.readline()))

			f.close()
	except IOError:
		# print("didn't find the file percept.txt")
		return [-1, -1, -1], [-1, -1], [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]

	return robotPos, chargerPos, distanceSensors


# Start of the program
# python3 agentPython.py [pfields/false] [navigate/false] 
if __name__ == "__main__":
	print("agentPython " + sys.argv[0] + " " + sys.argv[1] + " " + sys.argv[2])

	if len(sys.argv) < 3:
		print("Not enough parameters")
		quit()

	# do only if computing potential fields is turned on with the first argument
	if sys.argv[1] == "pfields":
		print("pfields")
		f = open("../Robot/output/globalpercepts.txt", "r")

		if f.closed:
			print("globalpercepts not found")
		else:
			fo = open("../Robot/output/globaltrajectory.tmp", "w")

			if fo.closed:
				print("globaltrajectory not opened")
			else:
				while True:
					robotPos = []
					strng = f.readline()
					if (len(strng) < 2):
						break

					words = strng.split(" ")
					
					robotPos.append(float(words[0]))
					robotPos.append(float(words[1]))
					robotPos.append(float(words[2]))
				
					chargerPos = []
					chargerPos.append(float(words[3]))
					chargerPos.append(float(words[4]))

					distanceSensors = []
					for i in range(0,16):
						distanceSensors.append(float(words[i+5]))

					trajectory = computeTrajectory(robotPos, chargerPos, distanceSensors)

					fo.write(str(robotPos[0]) + " " + str(robotPos[1]) + " " + str(robotPos[2]) + " 1.0 " + str(trajectory[0]) + " " + str(trajectory[1]) + "\n")
				
				fo.close()

				# os.system("mv ../Robot/output/globaltrajectory.tmp ../Robot/output/globaltrajectory.txt")
				os.replace("../Robot/output/globaltrajectory.tmp", "../Robot/output/globaltrajectory.txt")

			f.close()
	
	# do only if navigate is turned on with the second argument
	if sys.argv[2] == "navigate":
		print("navigating")
		quit = False

		while (not quit):
			# read in the percepts
			robotPos, chargerPos, distanceSensors = readPerception("percepts")

			# compute trajectory
			if robotPos[0] != -1:
				trajectory = computeTrajectory(robotPos, chargerPos, distanceSensors)

				# write trajectory
				writeTrajectory("trajectory", 1.0, trajectory)

			quit = endSimulation()

			time.sleep(.5)
