## Compile and Run
1.  Compile the code the java code in the folder robotViewer by typing the command:: `javac *.java`
2.  Run the robot simulator: `java robotViewer`
    - This will launch a GUI through which you can evaluate your algorithms on the robot. The interface allows you to do the following:
        - Load a scenario. There are four, which you can load by pressing `1`, `2`, `3`, or `4` on the keyboard when the GUI window is active
        - Turn the “potential fields” display on and off: Press `p`
        - Turn the simulator on (to activate the robot): Press `r`.  You'll then need to run the python script below to actually get the robot to move.
        - Stop the simulator (turn it off): Press `q`
        - Manual move the robot: `Left click` on the world to place the robot in a specific place in the world.
        - Change the robot's goal: `Right click` in the world to move the goal location of the robot. By default, it is set to be on the charger whenever you load a new scenario.
4.  Press `r` to start the simulator. Then run your potential fields algorithm on the robot by running theAgent.py python program in the folder called Agent. To do this, type: `python3 theAgent.py false navigate`
    - The program will only run if the simulator is turned on (in the GUI). To give yourself an idea of how the robot will navigate the world, you can update the potential fields display in the GUI by running this same program in a different mode: `python3 theAgent.py pfields false`

## Implementing your Algorithm in the Code
You should modify/complete the function “computeTrajectory” in theAgent.py in the Agent folder. The function takes as input the sensor readings from the robot (robot’s position, goal position, and distance sensors). Your algorithm should compute a directional unit vector (stored in the array trajectory), which specifies the direction the robot should go given the robot’s percepts (sensor readings).