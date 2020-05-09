# ENPM661_Project5

Instructions for running the code:

Unzip the submitted folder => "Output"
We recommend you save the folder on desktop and to run the program using the terminal in linux.


For Dijkstra based path planning algorithm: 
Two .py files are required to run the Dijkstra Algorithm based program. (frontend_dij.py and backend_dij.py).
Run the file named frontend_dij.py from dijkstra_trial to execute the code.

Once you run the program, you will see a OpenCV terminal showing output stepwise. 
You will have to press 'Q'/'Esc' key to start the next step after the execution of preious step

Python files needed:
frontend_dij.py
backend_dij.py

Result:
The blue region is the explored region. 
The green region is the unexplored region. 
The red region is the path from the start node to goal node.
The black region is the Obstacle space with clearance.

If there is a solution you can see the path in the pop up

Default Values:(X,Y)

Start Point = (10,10)
Given Contamination point 1: (475, 100)- 30%
Given Contamination point 2: (450, 680)- 50%
Given Contamination point 3: (200, 500)- 20%



For A-Star based path planning algorithm: 

Two .py files are required to run the A-star Algorithm based program. (frontend_astar.py and backend_astar.py).
Run the file named frontend_astar.py from astar_trial to execute the code.

Once you run the program, you will see a OpenCV terminal showing output stepwise. 
You will have to press 'Q'/'Esc' key to start the next step after the execution of preious step.


Py files needed:
frontend_astar.py
backend_astar.py


Result:
The blue region is the explored region. 
The black region is the unexplored region. 
The red region is the path from the start node to goal node.
The green region is the Obstacle space with clearance.

If there is a solution you can see the path in the pop up

Default Values:(X,Y, Orientation)

Start Point = (10,10,60)
Given Contamination point 1: (475, 100, 60)- 30%
Given Contamination point 2: (450, 690, 30)- 40%
Given Contamination point 3: (150, 500, 90)- 20%



Software Required:
To run the .py files, use Python 3. Standard Python 3 libraries like numpy, heapq, math and OpenCV are used.
