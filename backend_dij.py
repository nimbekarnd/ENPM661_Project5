import numpy as np
import math
from collections import defaultdict
from heapq import *
import cv2




class Step(object):

	def __init__(self, start, goal, position , clearance , radius ):
		self.start = start
		self.goal = goal
		self.position = position
		self.clearance = clearance
		self.radius = radius
		self.cost = 0
		self.y = 0
		self.x = 0
		self.numRows = 500
		self.numCols = 750

	def __eq__(self, other):
		return self.position == other.position

	def IsValid(self, currRow, currCol):
		sum_rc = self.clearance + self.radius
		return (currRow >= (1 + sum_rc) and currRow <= (500 - sum_rc) and currCol >= (1 + sum_rc) and currCol <= (750 - sum_rc))


	def obstacle(self, row, col):
		sum_rc = self.clearance + self.radius
		
		#circle
		circle1 = (row - 175)**2 + (col - 50)**2 - (15+sum_rc)**2 <= 0
		circle2 = (row - (175))**2 + (col - (700))**2 - (15+sum_rc)**2 <= 0
	       

	    #square
		square1 = row <= 425 + sum_rc and row >= 400 - sum_rc and col <= 150 + sum_rc and col >= 100 - sum_rc
		square2 = row <= 499 + sum_rc and row >= 400 - sum_rc and col <= 175 + sum_rc and col >= 150 - sum_rc
		square3 = row <= 499 + sum_rc and row >= 400 - sum_rc and col <= 600 + sum_rc and col >= 575 - sum_rc
		square4 = row <= 425 + sum_rc and row >= 400 - sum_rc and col <= 650 + sum_rc and col >= 600 - sum_rc
		square5 = row <= 40 + sum_rc and row >= 0 - sum_rc and col <= 575 + sum_rc and col >= 175 - sum_rc
		square6 = row <= 300 + sum_rc and row >= 200 - sum_rc and col <= 200 + sum_rc and col >= 0 - sum_rc
		square7 = row <= 300 + sum_rc and row >= 200 - sum_rc and col <= 750 + sum_rc and col >= 550 - sum_rc
		square8 = row <= 325 + sum_rc and row >= 100 - sum_rc and col <= 325 + sum_rc and col >= 300 - sum_rc
		square9 = row <= 325 + sum_rc and row >= 100 - sum_rc and col <= 450 + sum_rc and col >= 425 - sum_rc
		square10= row <= 125 + sum_rc and row >= 100 - sum_rc and col <= 300 + sum_rc and col >= 0 - sum_rc
		square11= row <= 125 + sum_rc and row >= 100 - sum_rc and col <= 750 + sum_rc and col >= 450 - sum_rc



		if(circle1 or circle2 or square1 or square2 or square3 or square4 or square5 or square6 or square7 or square8 or square9 or square10 or square11):
			return True
		return False
        
		

	def ActionU(self, currRow, currCol):
		if(self.IsValid(currRow - 1, currCol) and self.obstacle(currRow - 1, currCol) == False):
			self.x = 0
			self.y = 1
			self.cost = 1
			return True
		else:
			return False

	def ActionUR(self, currRow, currCol):
		if(self.IsValid(currRow+1, currCol+1) and self.obstacle(currRow +1, currCol + 1) == False):
			self.x = 1
			self.y = 1
			self.cost = np.sqrt(2)
			return True
		else:
			return False

	def ActionR(self, currRow, currCol):
		if(self.IsValid(currRow+1, currCol) and self.obstacle(currRow +1, currCol) == False):
			self.x = 1 
			self.y = 0
			self.cost = 1
			return True
		else:
			return False

	def ActionDR(self, currRow, currCol):
		if(self.IsValid(currRow+1, currCol-1) and self.obstacle(currRow + 1, currCol - 1) == False):
			self.x = 1 
			self.y = -1
			self.cost = np.sqrt(2)
			return True
		else:
			return False
	def ActionD(self, currRow, currCol):
		if(self.IsValid(currRow, currCol - 1) and self.obstacle(currRow, currCol - 1) == False):
			self.x = 0 
			self.y = -1
			self.cost = 1
			return True
		else:
			return False

	def ActionDL(self, currRow, currCol):
		if(self.IsValid(currRow - 1, currCol - 1) and self.obstacle(currRow - 1, currCol - 1) == False):
			self.x = -1 
			self.y = -1
			self.cost = np.sqrt(2)
			return True
		else:
			return False

	def ActionL(self, currRow, currCol):
		if(self.IsValid(currRow - 1, currCol) and self.obstacle(currRow - 1, currCol) == False):
			self.x = -1 
			self.y = 0
			self.cost = 1
			return True
		return False

	def ActionUL(self, currRow, currCol):
		if(self.IsValid(currRow - 1, currCol+1) and self.obstacle(currRow - 1, currCol + 1) == False):
			self.x = -1 
			self.y = 1
			self.cost = np.sqrt(2)
			return True
		else:
			return False

	def checkgoal(self,currRow, currCol):
		if(currRow== self.goal[0] and currCol == self.goal[1]):
			
			return True
		else:
			return False


	def Dij(self):
		print('Computing...')
		costMap = {}
		visited_nodes = {}
		path = {}

		for row in np.arange(1, self.numRows + 1, 1):
			for col in np.arange(1, self.numCols + 1, 1): 
				costMap[(row, col)] = float('inf')
				visited_nodes[(row, col)] = False
				path[(row, col)] = -1

		explored = []
		queue = []

		heappush(queue, (0, self.start))
		costMap[self.start] = 0

		while(len(queue)) > 0:
			heapify(queue)
			_, currNode = heappop(queue)
			visited_nodes[currNode] = True
			explored.append(currNode)
			

			if(self.checkgoal(currNode[0], currNode[1]) == True):
				break

			if(self.ActionU(currNode[0],currNode[1]) and visited_nodes[(currNode[0] + self.x ,currNode[1] + self.y)] == False and (costMap[(currNode[0]+ self.x, currNode[1] + self.y)] > costMap[(currNode)] + self.cost)):
				costMap[(currNode[0] + self.x, currNode[1] + self.y)] = costMap[(currNode)] + self.cost
				path[(currNode[0] + self.x, currNode[1] + self.y)] = currNode
				heappush(queue,(costMap[(currNode[0] + self.x, currNode[1] + self.y)], (currNode[0] + self.x, currNode[1] + self.y)))

			if(self.ActionUR(currNode[0],currNode[1]) and visited_nodes[(currNode[0] + self.x ,currNode[1] + self.y)] == False and (costMap[(currNode[0]+ self.x, currNode[1] + self.y)] > costMap[(currNode)] + self.cost)):
				costMap[(currNode[0] + self.x, currNode[1] + self.y)] = costMap[(currNode)] + self.cost
				path[(currNode[0] + self.x, currNode[1] + self.y)] = currNode
				heappush(queue,(costMap[(currNode[0] + self.x, currNode[1] + self.y)], (currNode[0] + self.x, currNode[1] + self.y)))

			if(self.ActionR(currNode[0],currNode[1]) and visited_nodes[(currNode[0] + self.x ,currNode[1] + self.y)] == False and (costMap[(currNode[0]+ self.x, currNode[1] + self.y)] > costMap[(currNode)] + self.cost)):
				costMap[(currNode[0] + self.x, currNode[1] + self.y)] = costMap[(currNode)] + self.cost
				path[(currNode[0] + self.x, currNode[1] + self.y)] = currNode
				heappush(queue,(costMap[(currNode[0] + self.x, currNode[1] + self.y)], (currNode[0] + self.x, currNode[1] + self.y)))

			if(self.ActionDR(currNode[0],currNode[1]) and visited_nodes[(currNode[0] + self.x ,currNode[1] + self.y)] == False and (costMap[(currNode[0]+ self.x, currNode[1] + self.y)] > costMap[(currNode)] + self.cost)):
				costMap[(currNode[0] + self.x, currNode[1] + self.y)] = costMap[(currNode)] + self.cost
				path[(currNode[0] + self.x, currNode[1] + self.y)] = currNode
				heappush(queue,(costMap[(currNode[0] + self.x, currNode[1] + self.y)], (currNode[0] + self.x, currNode[1] + self.y)))

			if(self.ActionD(currNode[0],currNode[1]) and visited_nodes[(currNode[0] + self.x ,currNode[1] + self.y)] == False and (costMap[(currNode[0]+ self.x, currNode[1] + self.y)] > costMap[(currNode)] + self.cost)):
				costMap[(currNode[0] + self.x, currNode[1] + self.y)] = costMap[(currNode)] + self.cost
				path[(currNode[0] + self.x, currNode[1] + self.y)] = currNode
				heappush(queue,(costMap[(currNode[0] + self.x, currNode[1] + self.y)], (currNode[0] + self.x, currNode[1] + self.y)))

			if(self.ActionDL(currNode[0],currNode[1]) and visited_nodes[(currNode[0] + self.x ,currNode[1] + self.y)] == False and (costMap[(currNode[0]+ self.x, currNode[1] + self.y)] > costMap[(currNode)] + self.cost)):
				costMap[(currNode[0] + self.x, currNode[1] + self.y)] = costMap[(currNode)] + self.cost
				path[(currNode[0] + self.x, currNode[1] + self.y)] = currNode
				heappush(queue,(costMap[(currNode[0] + self.x, currNode[1] + self.y)], (currNode[0] + self.x, currNode[1] + self.y)))

			if(self.ActionL(currNode[0],currNode[1]) and visited_nodes[(currNode[0] + self.x ,currNode[1] + self.y)] == False and (costMap[(currNode[0]+ self.x, currNode[1] + self.y)] > costMap[(currNode)] + self.cost)):
				costMap[(currNode[0] + self.x, currNode[1] + self.y)] = costMap[(currNode)] + self.cost
				path[(currNode[0] + self.x, currNode[1] + self.y)] = currNode
				heappush(queue,(costMap[(currNode[0] + self.x, currNode[1] + self.y)], (currNode[0] + self.x, currNode[1] + self.y)))

			if(self.ActionUL(currNode[0],currNode[1]) and visited_nodes[(currNode[0] + self.x ,currNode[1] + self.y)] == False and (costMap[(currNode[0]+ self.x, currNode[1] + self.y)] > costMap[(currNode)] + self.cost)):
				costMap[(currNode[0] + self.x, currNode[1] + self.y)] = costMap[(currNode)] + self.cost
				path[(currNode[0] + self.x, currNode[1] + self.y)] = currNode
				heappush(queue,(costMap[(currNode[0] + self.x, currNode[1] + self.y)], (currNode[0] + self.x, currNode[1] + self.y)))


		check = []
		goalx = self.goal[0]
		goaly = self.goal[1]

	

		for a in np.arange(goalx - 1, goalx + 1, 1):
			for b in np.arange(goaly - 1, goaly + 1, 1):
				check.append(costMap[a,b])



		ans = float('inf')
		for c in range(len(check)):
			if(check[c] != ans):
				print("Path Exists")

			NoPath = 1
			break
		if (NoPath == 0):
			print("Path does not exist")
			return (explored, [], costMap[goalx,goaly])

		print(costMap[goalx, goaly], "answer")
		result = (goalx, goaly)


		backstates = []
		node = result
		while(path[node] != -1):
			backstates.append(node)
			node = path[node]
		backstates.append(self.start)
		backstates = list(reversed(backstates))

		#print(backstates)
		return (explored, backstates, costMap[goalx, goaly])



	def animate(self, explored, backstates, path):
		print('Animation in progress...')
		fourcc = cv2.VideoWriter_fourcc(*'XVID')
		out = cv2.VideoWriter(str(path), fourcc, 20.0, (self.numCols, self.numRows))
		image = np.zeros((self.numRows, self.numCols, 3), dtype=np.uint8)
		count = 0
		for state in explored:
			image[int(self.numRows - state[0]), int(state[1] - 1)] = (255, 255, 0)
			if(count%75 == 0):
				out.write(image)
			count = count + 1
			#cv2.imshow('Explored area for Reaching Centroid', image)
		count = 0
		for row in range(1, self.numRows + 1):
			for col in range(1, self.numCols + 1):
				if(image[int(self.numRows - row), int(col - 1), 0] == 0 and image[int(self.numRows - row), int(col - 1), 1] == 0 and image[int(self.numRows - row), int(col - 1), 2] == 0):
					if(self.IsValid(row, col) and self.obstacle(row, col) == False):
						image[int(self.numRows - row), int(col - 1)] = (125, 198, 0)
						if(count%75 == 0):
							out.write(image)
						count = count + 1
            
		if(len(backstates) > 0):
			for state in backstates:
				image[int(self.numRows - state[0]), int(state[1] - 1)] = (0, 0, 255)
				out.write(image)
		

				cv2.imshow('Path Generated', image)

				if cv2.waitKey(1) & 0xFF ==ord('q'):
					break



		print('Step reached')        
		cv2.waitKey(0)
		cv2.destroyAllWindows()

