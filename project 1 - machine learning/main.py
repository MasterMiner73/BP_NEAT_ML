import pygame, sys, math, random
import numpy as np
import track_class
import neat
import os
import sys

# IMPORTANT: 
# make sure you have python and pip
# make sure to install pygame and neat

# how to do this:

# in windows command prompt: 
# python --version
# pip --version
# pip install pygame
# pip install neat-python

class raycast:
	def __init__(self, start_pos: tuple, angle: float, line_len: int):
		self.line_len = line_len
		self.start_pos = start_pos
		self.angle_deg = angle 
		angle = math.radians(angle)
		self.camera = [0, 0]
		self.calculate_end_pos()

	def calculate_end_pos(self):
		self.angle = math.radians(self.angle_deg)
		end_x = self.start_pos[0] + (math.cos(self.angle) * self.line_len) # car nodes are calculated with - self.camera[0] and [1] so reverse it
		end_y = self.start_pos[1] - (math.sin(self.angle) * self.line_len)
		self.end_pos = (end_x, end_y)

	def draw_raycast_line(self):
		if self.end_pos == ():
			self.calculate_end_pos()
		pygame.draw.line(screen, light_green, (self.start_pos[0] - self.camera[0], self.start_pos[1] - self.camera[1]), (self.end_pos[0] - self.camera[0], self.end_pos[1] - self.camera[1]), 1)

	def calculate_hit(self, wall_coords: tuple): # wall coords = ((x1, y1), (x2, y2))
		self.calculate_end_pos() # resets the end pos

		# wall coords
		x1 = wall_coords[0][0]
		y1 = wall_coords[0][1]
		x2 = wall_coords[1][0]
		y2 = wall_coords[1][1]

		# ray coords
		x3 = self.start_pos[0]
		y3 = self.start_pos[1]
		x4 = self.end_pos[0]
		y4 = self.end_pos[1]

		den = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
		if den == 0:
			return False
		t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / den
		u = ((x1 - x3) * (y3 - y2) - (y1 - y3) * (x3 - x2))/ den
		if t > 0 and t < 1 and u > 0 and u < 1:
			pt = [0, 0]
			pt[0] = x1 + t * (x2 - x1)
			pt[1] = y1 + t * (y2 - y1)
			self.end_pos = tuple(pt)
			length = math.sqrt(math.pow(self.start_pos[0] - self.end_pos[0], 2) + math.pow(self.start_pos[1] - self.end_pos[1], 2))
			return (length, pt)
		else:
			return False

class Car:
	def __init__(self, position: tuple, width: int, height: int, angle: float, color: tuple, damaged_color: tuple, win_color: tuple, num_of_rays: int, angle_between_rays: int, ray_len: int, track_edge_pnts, num_of_track_nodes: int, camera):
		self.position = position
		self.width = width
		self.height = height
		self.angle = angle
		self.nodes = []
		self.angular_vel = 0
		self.speed_target = 0
		self.speed = 0
		self.car_color = color
		self.damaged_color = damaged_color
		self.win_color = win_color
		self.lock = False
		self.head = (position[0], position[1] + self.height/2)
		self.rays = []
		self.ray_len = ray_len
		self.ray_lens = []
		self.num_of_rays = num_of_rays
		self.angle_between_rays = angle_between_rays
		self.camera = camera
		self.track_edge_pnts = track_edge_pnts
		self.car_max_ang_vel = 0.7
		self.car_max_speed = 15
		self.car_acceleration_delta = 0.2
		self.num_of_track_nodes = num_of_track_nodes
		ray_angle = -(self.num_of_rays * self.angle_between_rays/2)
		for i in range(self.num_of_rays):
			ray_angle = self.angle_between_rays * i - (self.num_of_rays * self.angle_between_rays/2)
			self.rays.append(raycast(self.head, ray_angle + self.angle, self.ray_len))
		self.current_checkpoint = 0
		self.prev_pos = position
		self.update_ray_casts()

	def calc_position(self):
		self.prev_pos = self.position
		x_pos = math.cos(math.radians(self.angle)) * self.speed
		y_pos = -math.sin(math.radians(self.angle)) * self.speed
		x_pos += self.position[0]
		y_pos += self.position[1]
		return (x_pos, y_pos)

	def calc_nodes(self):
		list_nodes = []
		#delta x and y
		dx = self.width/2
		dy = self.height/2
		self.dist_to_ends = math.sqrt(math.pow(dx, 2) + math.pow(dy, 2))
		self.angle_end_node = math.degrees(math.atan(dy / dx))

		list_nodes.append(self.find_node(self.angle_end_node + self.angle - 90))
		list_nodes.append(self.find_node(180 - self.angle_end_node + self.angle - 90))
		list_nodes.append(self.find_node(self.angle_end_node + 180 + self.angle - 90)) # self.angle_end_node + 180 + self.angle - 90
		list_nodes.append(self.find_node(360 - self.angle_end_node + self.angle - 90)) # 360 - self.angle_end_node + self.angle - 90
		return list_nodes

	def find_node(self, angle: int):
		end_x = self.position[0] + (math.cos(math.radians(angle)) * self.dist_to_ends)
		end_y = self.position[1] - (math.sin(math.radians(angle)) * self.dist_to_ends)
		return (end_x - self.camera[0], end_y - self.camera[1])

	def check_for_ray_hit(self): # not for ray casts??!?!?!?!
		for i, edgepoint in enumerate(self.track_edge_pnts):
			if i == 0:
					i = len(self.track_edge_pnts)-1
			else:
				i -= 1
			prev_edgepoint = self.track_edge_pnts[i]

			for j, node in enumerate(self.nodes):
				if j == 0:
					j = len(self.nodes)-1
				else:
					j -= 1
				prev_node = self.nodes[j]
				if self.calculate_hit((node, prev_node), (edgepoint, prev_edgepoint)) == True:
					if i == self.num_of_track_nodes:
						self.car_color = self.win_color
						self.lock_car()
						return
					else:
						return True
		return False

	def calculate_hit(self, ray: tuple, edgepoint: tuple): # wall coords = ((x1, y1), (x2, y2))
		# wall coords
		x1 = edgepoint[0][0] - self.camera[0]
		y1 = edgepoint[0][1] - self.camera[1]
		x2 = edgepoint[1][0] - self.camera[0]
		y2 = edgepoint[1][1] - self.camera[1]

		# ray coords
		x3 = ray[0][0]
		y3 = ray[0][1]
		x4 = ray[1][0]
		y4 = ray[1][1]

		den = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
		if den == 0:
			return False
		t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / den
		u = ((x1 - x3) * (y3 - y2) - (y1 - y3) * (x3 - x2)) / den
		if t > 0 and t < 1 and u > 0 and u < 1:
			return True
		else:
			return False

	def update(self):
		self.calc_speed()
		self.angle += np.clip(self.angular_vel * self.speed, -self.car_max_ang_vel * 4.5, self.car_max_ang_vel * 4.5)
		self.position = self.calc_position()
		self.nodes = self.calc_nodes()
		self.head = ((self.nodes[0][0] + self.nodes[1][0])/2 + self.camera[0], (self.nodes[0][1] + self.nodes[1][1])/2 + self.camera[1])
		if self.check_for_ray_hit() == True:
			self.lock_car()
			self.car_color = self.damaged_color
		self.update_ray_casts()
		self.calculate_checkpoint()

	def calc_speed(self):
		if self.speed < self.speed_target:
			if self.speed < -self.car_max_speed:
				self.speed = -self.car_max_speed
			else:
				self.speed += self.car_acceleration_delta
		elif self.speed > self.speed_target:
			if self.speed > self.car_max_speed:
				self.speed = self.car_max_speed
			else:
				self.speed -= self.car_acceleration_delta
		if math.isclose(self.speed, 0, abs_tol = 0.0001) == True:
			self.speed = 0
	
	def render(self):
		pygame.draw.polygon(screen, self.car_color, self.nodes)

	def lock_car(self):
		self.speed_target = 0
		self.speed = 0
		self.angular_vel = 0
		self.lock = True

	def update_ray_casts(self):
		self.ray_lens = []
		ray_angle = -(self.num_of_rays * self.angle_between_rays/2)
		for i, ray in enumerate(self.rays):
			ray_angle = self.angle_between_rays* i - ((self.num_of_rays-1) * self.angle_between_rays/2)
			ray.angle_deg = ray_angle + self.angle
			ray.start_pos = self.head
			ray.calculate_end_pos()
			ray.camera = self.camera

		for i, ray in enumerate(self.rays):
			shortest_hit_ray_len = self.ray_len
			ray_new_coords = ()
			rays_pt = ()
			for i, edge_point in enumerate(self.track_edge_pnts):
				if i == self.num_of_track_nodes + 1:
					continue
				prev_index = i-1
				prev_edge_point = self.track_edge_pnts[prev_index]
				edge_seg = (edge_point, prev_edge_point)
				calculate_hit_returns = ray.calculate_hit(edge_seg) # (length, point of intersection)
				if calculate_hit_returns == False:
					continue
				length = calculate_hit_returns[0]
				pt = calculate_hit_returns[1] 
				if length < shortest_hit_ray_len:
					shortest_hit_ray_len = length
					rays_new_coords = edge_seg
					rays_pt = pt
			if rays_pt != ():
				ray.end_pos = rays_pt
			self.ray_lens.append(shortest_hit_ray_len)

	def draw_raycasts(self):
		for ray in self.rays:
			ray.draw_raycast_line()

	def calculate_checkpoint(self):
		next_checkpoint_coords = ((self.track_edge_pnts[self.current_checkpoint + 1][0] + self.camera[0], self.track_edge_pnts[self.current_checkpoint + 1][1] + self.camera[1]), (self.track_edge_pnts[len(self.track_edge_pnts) - self.current_checkpoint - 2][0] + self.camera[0], self.track_edge_pnts[len(self.track_edge_pnts) - self.current_checkpoint - 2][1] + self.camera[1]))
		current_checkpoint_coords = ((self.track_edge_pnts[self.current_checkpoint][0] + self.camera[0], self.track_edge_pnts[self.current_checkpoint][1] + self.camera[1]), (self.track_edge_pnts[len(self.track_edge_pnts) - self.current_checkpoint - 1][0] + self.camera[0], self.track_edge_pnts[len(self.track_edge_pnts) - self.current_checkpoint - 1][1] + self.camera[1]))
		pos_coords = (self.prev_pos, self.position)
		if self.calculate_hit(pos_coords, next_checkpoint_coords) == True:
			self.current_checkpoint += 1
		elif self.calculate_hit(pos_coords, current_checkpoint_coords) == True:
			self.current_checkpoint -= 1

pygame.init()

# colors
dark_grey = (15, 15, 15)
light_grey = (230, 230, 230)
blue = (60, 43, 143)
red = (143, 63, 41)
light_green = (150, 230, 150)
dark_green = (93, 199, 107)

# basic pygame variables
screen_width = 1000
screen_height = 720
clock = pygame.time.Clock()
screen = pygame.display.set_mode((screen_width, screen_height))
FPS = 30

#text variables
font = pygame.font.SysFont("Arial", 30, bold=False, italic=True)
info_font = pygame.font.SysFont("Arial", 19, bold=False, italic=True)

def eval_genomes(genomes, config):
	global FPS
	#important variables
	track_len = 3000
	num_of_track_nodes = 20
	track = track_class.Track(num_of_track_nodes, track_len, 45, 5, 5, (0, 0)) #(22, 6000, 90, 10, 10, (screen_width/2, screen_height/2))
	car_st_pos = ((track.track_nodes[0][0][0] + track.track_nodes[1][0][0])/2, (track.track_nodes[0][0][1] + track.track_nodes[1][0][1])/2)
	#car = Car(car_st_pos, 35, 60, track.track_nodes[1][1], blue, red, dark_green, 7, 30, 600, track.calc_list_of_edgepoints(track.track_nodes), num_of_track_nodes)
	#self, position: tuple, width: int, height: int, angle: float, color: tuple, damaged_color: tuple, win_color: tuple num_of_rays: int, angle_between_rays: int, ray_len: int
	time = 0
	cam_car_index = 0
	max_time = 40
	#camera = (car_st_pos[0] - screen_width/2, car_st_pos[1] - screen_height/2)
	camera = (track.track_nodes[3][0][0] - screen_width/2, track.track_nodes[3][0][1] - screen_height/2)
	cam_vel = [0, 0]
	cam_max_speed = 15

	#neural network vairables
	nets = []
	ge = []
	cars = []

	for _, g in genomes:
		net = neat.nn.FeedForwardNetwork.create(g, config)
		nets.append(net)
		cars.append(Car(car_st_pos, 17, 30, track.track_nodes[1][1], blue, red, dark_green, 7, 30, 600, track.calc_list_of_edgepoints(track.track_nodes), num_of_track_nodes, camera))
		g.fitness = 0
		ge.append(g)

	while True:
		for event in pygame.event.get():
			if event.type == pygame.QUIT:
				pygame.quit()
				sys.exit()
			
			if event.type == pygame.KEYDOWN:
				if event.key == pygame.K_LEFT:
					cam_car_index -= 1

					cam_vel[0] = -cam_max_speed

				if event.key == pygame.K_RIGHT:
					cam_car_index += 1

					cam_vel[0] = cam_max_speed

				if event.key == pygame.K_DOWN:
					cam_vel[1] = cam_max_speed
				if event.key == pygame.K_UP:
					cam_vel[1] = -cam_max_speed

				if event.key == pygame.K_a:
					FPS += 30
				if event.key == pygame.K_d:
					FPS -= 30
					if FPS <= 0:
						FPS = 30

			if event.type == pygame.KEYUP:
				if event.key == pygame.K_LEFT:
					cam_vel[0] = 0
				elif event.key == pygame.K_RIGHT:
					cam_vel[0] = 0

				if event.key == pygame.K_DOWN:
					cam_vel[1] = 0
				elif event.key == pygame.K_UP:
					cam_vel[1] = 0
			
		if len(cars) == 0 or time >= max_time:
			break


		updated_edge_points = []
		for i, coord in enumerate(track.calc_list_of_edgepoints(track.track_nodes)):
			updated_edge_points.append([coord[0], coord[1]])
			updated_edge_points[i][0] -= camera[0]
			updated_edge_points[i][1] -= camera[1]

		rem = []
		#udate the cars
		for x, car in enumerate(cars):
			# change the input to activate IF NUM OF RAYS != 7
			output = nets[x].activate((car.ray_lens[0], car.ray_lens[1], car.ray_lens[2], car.ray_lens[3], car.ray_lens[4], car.ray_lens[5], car.ray_lens[6], car.speed * 200))

			if output[0] >= 0.5:
				car.angular_vel = car.car_max_ang_vel
			elif output[1] >= 0.5: 
				car.angular_vel = -car.car_max_ang_vel
			else:
				car.angular_vel = 0
			if output[2] >= 0.5:
				car.speed_target = car.car_max_speed
			else:
				car.speed_target = car.car_max_speed / 2

			if car.lock == False:
				car.camera = camera
				car.update()
			else:
				ge[x].fitness = car.current_checkpoint
				if ge[x].fitness >= num_of_track_nodes - 1:
					ge[x].fitness += max_time - time
				cars.pop(x)
				nets.pop(x)
				ge.pop(x)

		# move objects based on camera
		if cam_car_index >= len(cars):
			cam_car_index = 0
		elif cam_car_index < 0:
			cam_car_index = len(cars) - 1
		try:
			camera = (cars[cam_car_index].position[0] - screen_width/2, cars[cam_car_index].position[1] - screen_height/2)
		except:
			pass

		#visuals
		screen.fill(dark_grey)
		pygame.draw.polygon(screen, light_grey, updated_edge_points)
		for car in cars:
			car.draw_raycasts()
			car.render()
		time_text = font.render(f'time: {round(time, 1)} seconds', True, light_grey, dark_grey)
		screen.blit(time_text, (0, 0))
		FPS_text = font.render(f'FPS: {FPS}', True, light_grey, dark_grey)
		screen.blit(FPS_text, (0, 30))
		cars_len_text = font.render(f'Cars left: {len(cars)}', True, light_grey, dark_grey)
		screen.blit(cars_len_text, (0, 60))

		cam1_text = info_font.render(" use left and right arrow keys", True, light_grey, dark_grey)
		screen.blit(cam1_text, (0, screen_height - 100))
		cam2_text = info_font.render("to change camera perspective", True, light_grey, dark_grey)
		screen.blit(cam2_text, (0, screen_height - 80))
		speed1_text = info_font.render(" use A and D to", True, light_grey, dark_grey)
		screen.blit(speed1_text, (0, screen_height - 60))
		speed2_text = info_font.render("to chane FPS (speed)", True, light_grey, dark_grey)
		screen.blit(speed2_text, (0, screen_height- 40))
		console_text = info_font.render("Generational info displayed in the console", True, light_grey, dark_grey)
		screen.blit(console_text, (0, screen_height - 20))

		pygame.display.flip()
		clock.tick(FPS)
		time += 1/30

def run(config_path):
	config = neat.config.Config(
		neat.DefaultGenome,
        neat.DefaultReproduction,
        neat.DefaultSpeciesSet,
        neat.DefaultStagnation,
        config_path
	)

	p = neat.Population(config)

	p.add_reporter(neat.StdOutReporter(True))
	stats = neat.StatisticsReporter()
	p.add_reporter(stats)

	winner = p.run(eval_genomes, 100000)

if __name__ == "__main__":
	local_dir = os.path.dirname(__file__)
	config_path = os.path.join(local_dir, "config.txt")
	run(config_path)
