# 16.633- The Autonomous Drone Class
This class uses Tello drones to autonomously navigate around a track. The track composes of hoops with april tags that the drones must fly through. This project details a progression of assignments that lead up to the final competition. The assignments can be found in the neet_ass folder as html files. The course covers concepts like optical flow, pinhole camera model, image transformations, and PID controls. 

Issue Log:
1. Could not download April Tags plugins:
- Solution: Downgrade python to version 3.7 or 3.6 using a virtual environment 
2. Flow point drifting:
- Solution 1: Make the Flow point be closer to the top of the screen because when the drone flys forwards you are pointing downwards
- Solution 2: Correct your x, y position using the tags before using the flow point 
- Solution 3: Make sure the surroundings are not one plain surface but have distinguishable things like lines or stark different colors to place flow points on 
3. failing to grab video stream: 
