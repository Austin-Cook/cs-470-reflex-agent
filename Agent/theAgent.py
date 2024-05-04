# TODO: You should write the function computeTrajectory

import sys
import os
import time
import math

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
	# TODO: tells the robot the robot which direction to go (default specified here: robot goes up)

	trajectory = []
	trajectory.append(0.0)
	trajectory.append(1.0)

	# End TODO

	# call this function before returning (to normalize the trajectory vector so that it is a unit vector)
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


