# StateFeedbackControl
Waypoint following controller design for vehicles using feedback controller

•  Vehicle models
•  Waypoint following controller design for vehicles
•  Controller design with state feedback control

▪ Implement the path following controller to drive the simulated vehicle along the given trajectory.
▪ The path following controller produced by this gain matrix performs a PD-control. It uses a PD-controller to correct along-track error for longitude control. The lateral control is also a PD-controller for cross-track error because δθ is related to the derivative of δy.


## Implementing the Controller
Reference States:  [xref , yref , θref , vref ]
 
Current States: [xB, yB, θB, vB]
Errors
![image](https://user-images.githubusercontent.com/64373075/177687374-03c3f684-1732-4673-a127-0c0bdfd4c01e.png)

Control Law:
![image](https://user-images.githubusercontent.com/64373075/177687545-fe516a1d-1b42-4cf6-b3a2-bbe16699eae8.png)


## Steps to run the Project
1. cd catkin_ws/
2. source devel/setup.bash
3. cd src/mp2/src
4. python3 main.py
