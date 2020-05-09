
# header files
from backend_astar import *


#Taking inputs from user
startRow = 10
startCol = 10
startOrientation = 60
goalOrientation = 30
goal1Row = 475
goal1Col = 100
goal1Orientation = 60
goal1per = 30
goal2Row = 450
goal2Col = 690
goal2Orientation = 30
goal2per = 40
goal3Row = 150
goal3Col = 500
goal3Orientation = 90
goal3per = 20
radius = 1
clearance = 1
rpm1 = 30
rpm2 = 60






radius = 5 #Radius of bot in cm
wheelRadius = 1 #Wheel radius in meters 
length = 5 #Length of bot
dt = 0.5 #Time intravel



# take start and goal node as input
start = (startRow, startCol)
e1 = (goal1Row, goal1Col)

f1 = (goal2Row, goal2Col)

g1 = (goal3Row, goal3Col)

print("\n Start point:", start)

print("\n Orientation of start point:", startOrientation)

print("\n RPM 1:", rpm1)

print("\n RPM 2:", rpm2)


print("\n Contamination points (x co-ordinate, y co-rodinate, orientation, contamination percent) Before Prioritizing based on Contamination percentage")
print((goal1Row, goal1Col, goal1Orientation, goal1per))
print((goal2Row, goal2Col, goal2Orientation, goal2per))
print((goal3Row, goal3Col, goal3Orientation, goal3per))


#Calculate Centroid
x = int((goal1Row + goal2Row +goal3Row ) / 3)
y = int((goal1Col + goal2Col +goal3Col ) / 3)
goal = (x,y)
goalnew = (x,y,goalOrientation)
#print(goal)
#print(goalnew)


A = {goal1per: e1,  goal2per: f1,  goal3per: g1}
l=list(A.items()) 
l.sort()
l.reverse()

goal1 = (l[0][1])

goal2 = (l[1][1])

goal3 = (l[2][1])


print("\n The Priority Queue based on Contamination Percent", l)

goal1 = (l[0][1])
print("\n contamination Point 1: ",goal1)
goal2 = (l[1][1])
print("\n contamination Point 2: ",goal2)
goal3 = (l[2][1])
print("\n contamination Point 3: ",goal3)


print("\n (X,Y) Co-ordinates for the centroid for all the contamination point is: ",goal)

print("\n Orientation for centroid is default at: ",goalOrientation)

print("\n Centroid for all the contamination point is: ",goalnew)



step1 = Step(start, goal, startOrientation, clearance, rpm1, rpm2, radius, wheelRadius ,length, dt)

step2 = Step(goal, goal1, goalOrientation, clearance, rpm1, rpm2, radius, wheelRadius ,length, dt)

step3 = Step(goal1, goal2, goal1Orientation, clearance, rpm1, rpm2, radius, wheelRadius ,length, dt)

step4 = Step(goal2, goal3, goal2Orientation, clearance, rpm1, rpm2, radius, wheelRadius ,length, dt)

print("\n .")

print("\n Step 1: To Calculate the path to Centroid")

print("\n .")

ans1 = step1.IsValid(start[0], start[1]) and step1.IsValid(goal[0], goal[1])
if(ans1==1):
    ans2 = step1.IsObstacle(start[0],start[1]) and step1.IsObstacle(goal[0],goal[1])
    if(ans2==1):
        (explored_states, backtrack_states) = step1.Astar()
        step1.animate(explored_states, backtrack_states, "C:/Users/13017/Desktop/Output/proj5/path_1/step1.avi")
    else:
        pass
else:
    print("The entered initial node or final node have error is outside the map ")
    
print("\n .")

print("\n Step 2: To Calculate the path to Contamination point 1")

print("\n .")

ans3 = step2.IsValid(goal[0], goal[1]) and step2.IsValid(goal1[0], goal1[1])
if(ans3==1):
    ans4 = step2.IsObstacle(goal[0],goal[1]) and step2.IsObstacle(goal1[0],goal1[1])
    if(ans4==1):
        (explored_states, backtrack_states) = step2.Astar()
        step2.animate(explored_states, backtrack_states, "C:/Users/13017/Desktop/Output/proj5/path_1/step2.avi")
    else:
        pass
else:
    print("The entered initial node or final node have error is outside the map ")
    
print("\n .")

print("\n Step 3: To Calculate the path to Contamination point 2")

print("\n .")

ans5 = step3.IsValid(goal1[0], goal1[1]) and step3.IsValid(goal2[0], goal2[1])
if(ans5==1):
    ans6 = step3.IsObstacle(goal1[0],goal1[1]) and step3.IsObstacle(goal2[0],goal2[1])
    if(ans6==1):
        (explored_states, backtrack_states) = step3.Astar()
        step3.animate(explored_states, backtrack_states, "C:/Users/13017/Desktop/Output/proj5/path_1/step3.avi")
    else:
        pass
else:
    print("The entered initial node or final node have error is outside the map ")
    
print("\n .")

print("\n Step 4: To Calculate the path to Contamination point 3")

print("\n .")

ans7 = step4.IsValid(goal2[0], goal2[1]) and step4.IsValid(goal3[0], goal3[1])
if(ans7==1):
    ans8 = step4.IsObstacle(goal2[0],goal2[1]) and step4.IsObstacle(goal3[0],goal3[1])
    if(ans8==1):
        (explored_states, backtrack_states) = step4.Astar()
        step4.animate(explored_states, backtrack_states, "C:/Users/13017/Desktop/Output/proj5/path_1/step4.avi")
    else:
        pass
else:
    print("The entered initial node or final node have error is outside the map ")

print("\n .")
print("\n .")
print("\n .")



print("\n Path Planning Completed")