from backend_dij import *
import numpy as np
import math
import sys
'''
print("The start and end coordinates should lie between 200 x 300 area.")
startRow = int(input("Enter the x-coordinate for start node: "))
startCol = int(input("Enter the y-coordinate for start node: "))
goalRow = int(input("Enter the x-coordinate for goal node: "))
goalCol = int(input("Enter the y-coordinate for goal node: "))
radius = int(input("Enter the radius for the robot : "))
clearance = int(input("Enter the clearance for the robot : "))
'''


startRow = 10
startCol = 10
goal1Row = 475
goal1Col = 100
goal1per = 30
goal2Row = 450
goal2Col = 680
goal2per = 50
goal3Row = 200
goal3Col = 500
goal3per = 20

radius = 1
clearance = 1

# take start and goal node as input
start = (startRow, startCol)
e = (goal1Row, goal1Col)
f = (goal2Row, goal2Col)
g = (goal3Row, goal3Col)


print("\n Start point:", start)
print("\n Contamination points (x co-ordinate, y co-rodinate, contamination percent) Before Prioritizing based on Contamination percentage")
print((goal1Row, goal1Col, goal1per))
print((goal2Row, goal2Col, goal2per))
print((goal3Row, goal3Col, goal3per))


#Calculate Centroid
x = int((goal1Row + goal2Row + goal3Row ) / 3)
y = int((goal1Col + goal2Col + goal3Col ) / 3)
goal = (x,y)


A = {goal1per: e, goal2per: f, goal3per: g}
l=list(A.items()) 
l.sort()
l.reverse()

print("\n The Priority Queue based on Contamination Percent", l)

goal1 = (l[0][1])
print("\n contamination Point 1: ",goal1)
goal2 = (l[1][1])
print("\n contamination Point 2: ",goal2)
goal3 = (l[2][1])
print("\n contamination Point 3: ",goal3)


print("\n Centroid for all the contamination point is: ",goal)



step1 = Step(start, goal, None, clearance, radius)

step2 = Step(goal, goal1, None, clearance, radius)

step3 = Step(goal1, goal2, None, clearance, radius)

step4 = Step(goal2, goal3, None, clearance, radius)




print("\n Step 1: To Calculate the path to Centroid")
print("\n .")
print("\n .")
print("\n .")

if(step1.IsValid(start[0], start[1])) and (step1.IsValid(goal[0], goal[1])) and (step1.obstacle(start[0],start[1]) == False) and (step1.obstacle(goal[0], goal[1]) == False):
    (explored, backstates, path_step1) = step1.Dij()
    step1.animate(explored, backstates, "C:/Users/13017/Desktop/Output/proj5/path/step1.avi")
    print("\nOptimal path found. Distance is " + str(path_step1))
    if(path_step1 == float('inf')):
        print("\nNo optimal path found.")
    else:
        pass
else:
    print("The entered nodes are outside the map or in the obstacle ")
    
print("\n ")
print("\n ")
print("\n ")

print("\n Step 2: To Calculate the path to Contamination point 1")
print("\n .")
print("\n .")
print("\n .")

if(step2.IsValid(goal[0], goal[1])) and (step2.IsValid(goal1[0], goal1[1])) and (step2.obstacle(goal[0],goal[1]) == False) and (step2.obstacle(goal1[0], goal1[1]) == False):
    (explored, backstates, path_step2) = step2.Dij()
    step2.animate(explored, backstates, "C:/Users/13017/Desktop/Output/proj5/path/step2.avi")
    print("\nOptimal path found. Distance is " + str(path_step2))
    if(path_step2 == float('inf')):
        print("\nNo optimal path found.")
    else:
        pass
else:
    print("The entered nodes are outside the map or in the obstacle ")

print("\n ")
print("\n ")
print("\n ")

print("\n Step 3: To Calculate the path to Contamination point 2")
print("\n .")
print("\n .")
print("\n .")

if(step3.IsValid(goal1[0], goal1[1])) and (step3.IsValid(goal2[0], goal2[1])) and (step3.obstacle(goal1[0],goal1[1]) == False) and (step3.obstacle(goal2[0], goal2[1]) == False):
    (explored, backstates, path_step3) = step3.Dij()
    step3.animate(explored, backstates, "C:/Users/13017/Desktop/Output/proj5/path/step3.avi")
    print("\nOptimal path found. Distance is " + str(path_step3))
    if(path_step3 == float('inf')):
        print("\nNo optimal path found.")
    else:
        pass
else:
    print("The entered nodes are outside the map or in the obstacle ")

print("\n ")
print("\n ")
print("\n ")

print("\n Step 4: To Calculate the path to Contamination point 3")
print("\n .")
print("\n .")
print("\n .")

if(step4.IsValid(goal2[0], goal2[1])) and (step4.IsValid(goal3[0], goal3[1])) and (step4.obstacle(goal2[0],goal2[1]) == False) and (step4.obstacle(goal3[0], goal3[1]) == False):
    (explored, backstates, path_step4) = step4.Dij()
    step4.animate(explored, backstates, "C:/Users/13017/Desktop/Output/proj5/path/step4.avi")
    print("\nOptimal path found. Distance is " + str(path_step4))
    if(path_step4 == float('inf')):
        print("\nNo optimal path found.")
    else:
        pass
else:
    print("The entered nodes are outside the map or in the obstacle ")
print("\n .")
print("\n .")
print("\n .")



print("\n Path Planning Completed")