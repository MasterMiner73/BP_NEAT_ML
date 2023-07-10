import pygame, sys, random, math

class Track:
	def __init__(self, number_of_track_nodes: int, track_length: int, track_width: int, track_width_variance: int, dist_variance: int, track_start_pos: int):
		self.number_of_track_nodes = number_of_track_nodes
		self.track_length = track_length
		self.track_width = track_width
		self.track_width_variance = track_width_variance
		self.dist_variance = dist_variance
		self.track_start_pos = track_start_pos
		self.track_nodes = self.calculate_basic_track_nodes_line()

	def calculate_basic_track_nodes_line(self):
		self.dist_between_points = self.track_length / (int(self.number_of_track_nodes) + 1)
		list_nodes = []
		list_nodes.append([self.track_start_pos, False, ((0, 0), (0, 0))]) # (coord: tuple, angle: int, endpoints: list of tuples -> tuple of tuples) turned into tuple when done editing it
		for i in range(int(self.number_of_track_nodes)):
			length = self.dist_between_points + random.randint(0, self.dist_variance)
			new_angle =  random.randint(-75, 75)
			angle = new_angle + list_nodes[i][1]
			my_pos = (math.cos(math.radians(angle)) * length + list_nodes[i][0][0], -math.sin(math.radians(angle)) * length + list_nodes[i][0][1])
			bis_coords = self.find_bis(list_nodes[i][0], angle, list_nodes[i][1], self.track_width)
			list_nodes[i][2] = bis_coords

			if self.check_track_interesections(list_nodes, bis_coords) == False: # false if there is an intersection
				return self.calculate_basic_track_nodes_line()

			list_nodes.append([my_pos, angle, ((0, 0), (0, 0))])

		last_list_node = list_nodes[len(list_nodes) -1]
		last_list_node[2] = self.find_bis(last_list_node[0], last_list_node[1], False, self.track_width) # find the bis for the last element
		return list_nodes


	def find_bis(self, coord: tuple, angle: int, prev_angle, length: int):
		if prev_angle == False:
			prev_angle = angle
		average = (angle + 180 - prev_angle) / 2
		angle1 = angle - average
		angle2 = (angle - average) - 180
		coord_1 = (math.cos(math.radians(angle1)) * length + coord[0], -math.sin(math.radians(angle1)) * length + coord[1])
		coord_2 = (math.cos(math.radians(angle2)) * length + coord[0], -math.sin(math.radians(angle2)) * length + coord[1])
		return (coord_1, coord_2)

	def check_track_interesections(self, list_nodes: list, new_coords: list):
		prev_coords = (list_nodes[len(list_nodes) - 2][2][0], list_nodes[len(list_nodes) - 2][2][1])
		if prev_coords == False:
			return
		coords = [[0,0],[0,0]]
		coords[0][0] = new_coords[0][0] * 2 - prev_coords[0][0]
		coords[0][1] = new_coords[0][1] * 2 - prev_coords[0][1]
		coords[1][0] = new_coords[1][0] * 2 - prev_coords[1][0]
		coords[1][1] = new_coords[1][1] * 2 - prev_coords[1][1]
		edgepoints = self.calc_list_of_edgepoints(list_nodes)
		for i, edge_seg in enumerate(edgepoints):
			prev_edge_seg = []
			if i == 0:
				prev_edge_seg = edgepoints[len(edgepoints) - 1]
			else:
				prev_edge_seg = edgepoints[i - 1]
			check1 = self.calculate_hit((tuple(coords[0]), prev_coords[0]), (edge_seg, prev_edge_seg))
			check2 = self.calculate_hit((tuple(coords[1]), prev_coords[1]), (edge_seg, prev_edge_seg))
			check3 = self.calculate_hit((tuple(coords[0]), coords[1]), (edge_seg, prev_edge_seg))
			if check1 == True or check2 == True or check3 == True:
				#print("1")
				return False
		return True

	def calc_list_of_edgepoints(self, list_nodes: list):
		edge_points_1 = []
		edge_points_2 = []
		for i, track_node in enumerate(list_nodes):
			edge_points_1.append(list(track_node[2][0]))
			edge_points_2.append(list(track_node[2][1]))
		edge_points = edge_points_1 + edge_points_2[::-1]
		return edge_points

	def calculate_hit(self, new_edge_coords: tuple, edge_seg: tuple): # wall coords = ((x1, y1), (x2, y2))
		# wall coords
		x1 = new_edge_coords[0][0]
		y1 = new_edge_coords[0][1]
		x2 = new_edge_coords[1][0]
		y2 = new_edge_coords[1][1]

		# ray coords
		x3 = edge_seg[0][0]
		y3 = edge_seg[0][1]
		x4 = edge_seg[1][0]
		y4 = edge_seg[1][1]

		den = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
		if den == 0:
			return False
		t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / den
		u = ((x1 - x3) * (y3 - y2) - (y1 - y3) * (x3 - x2))/ den
		if t > 0 and t < 1 and u > 0 and u < 1:
			pt = [0, 0]
			pt[0] = x1 + t * (x2 - x1)
			pt[1] = y1 + t * (y2 - y1)
			pt = tuple(pt)
			if  math.isclose(pt[0], x3) or math.isclose(pt[0], x4) or math.isclose(pt[1], y3) or math.isclose(pt[1], y4):
				return False
			else:
				return True
		else:
			return False