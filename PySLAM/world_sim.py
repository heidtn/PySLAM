from selectors import EpollSelector
import cv2
import numpy as np
import pyglet
from pyglet import shapes
import heapq
from dataclasses import dataclass, field
from typing import Any
import time
from sklearn.manifold import trustworthiness



class World:
    def __init__(self, world_image_filename, scale=0.01):
        self.scale = scale # pixel size in meters
        self.world_grid_filename = world_image_filename
        self.world_grid = cv2.imread(world_image_filename)
        _, self.world_grid = cv2.threshold(self.world_grid, 127, 255, cv2.THRESH_BINARY)
        self.world_grid = cv2.cvtColor(self.world_grid, cv2.COLOR_BGR2GRAY)
        

    def cast_ray_ideal(self, origin, angle, space='image'):
        yslope = np.sin(angle)
        xslope = np.cos(angle)
        x = y = 0
        if space == 'image':
            x = origin[0]
            y = origin[1]
        elif space == 'world':
            x = origin[0]/self.scale
            y = self.world_grid.shape[0] - origin[1]/self.scale

        while x < self.world_grid.shape[1] and x > 0 and y < self.world_grid.shape[0] and y > 0:
            if self.world_grid[self.world_grid.shape[0] - int(y) - 1, int(x)] == 0:
                if space == 'image':
                    return np.array((x, y))
                elif space == 'world':
                    return np.array((x*self.scale, (self.world_grid.shape[0] - y)*self.scale))
            x += xslope
            y += yslope

        return None

    def create_point_grid(self, origin, count, space='image'):
        grid = np.zeros((self.world_grid.shape[0], self.world_grid.shape[1], 3))
        for i in range(count):
            angle = (np.pi*2.0*i) / count
            ray = self.cast_ray_ideal(origin, angle, space)
            if ray:
                grid[int(ray[0]), int(ray[1])] = (255, 255, 255)

        return grid

    def fill_point_grid(self, rays):
        grid = np.zeros((self.world_grid.shape[0], self.world_grid.shape[1], 3))
        for ray in rays:
            if ray is not None:
                grid[int(ray[1]), int(ray[0])] = (255, 255, 255)
        return grid

    def draw_spikes(self, grid, spikes):
        for spike in spikes:
            grid[int(spike[1]), int(spike[0])] = (255, 0, 0)

    def find_spikes(self, rays, thresh = 8, mindist=80):
        spikes = []
        for i in range(len(rays)):
            rayA = rays[(i-1)%len(rays)]
            rayB = rays[i]
            rayC = rays[(i+1)%len(rays)]
            if rayA is not None and rayB is not None and rayC is not None:
                if np.linalg.norm(rayA - rayB) < mindist and np.linalg.norm(rayC - rayB) < mindist:
                    rayAB = rayA - rayB
                    rayCB = rayC - rayB
                    if np.linalg.norm(rayAB + rayCB) > thresh:
                        spikes.append(rayB)
        return spikes
                
    def cast_ideal_rays(self, origin, count):
        # return the endpoint of N rays (None if no endpoint found)
        points = []
        for i in range(count):
            angle = (np.pi*2.0*i) / count
            points.append(self.cast_ray_ideal(origin, angle))

        return points

@dataclass
class Landmark:
    priority: int
    position: np.array
    key: int
    def __lt__(self, other):
        return self.priority > other.priority

class LandmarkDB:
    def __init__(self, threshold=25, mincount=3):
        self.landmarks: list[Landmark] = []
        self.landmark_map: dict[int, Landmark] = {}
        self.threshold = threshold
        self.mincount = mincount
        heapq.heapify(self.landmarks)
        self.cur_key = 0

    def find_matching_landmark(self, position: np.array):
        for landmark in self.landmarks:
            if np.linalg.norm(landmark.position - position) < self.threshold:
                landmark.priority += 1
                if landmark.priority > self.mincount:
                    heapq.heapify(self.landmarks)
                    return landmark.key, landmark.position
                else:
                    # TODO(HEIDT) do we want to keep checking for other nearby landmarks?
                    return None, None
        landmark = Landmark(0, position, self.cur_key)
        self.landmark_map[self.cur_key] = landmark 
        heapq.heappush(self.landmarks, landmark)
        self.cur_key += 1
        return None, None
        
    def update_position(self, key: int, position: np.array):
        self.landmark_map[key].position = position

class graphSLAM:
    def __init__(self):
        self.landmark_database = LandmarkDB()
        self.deltas = []
        self.position_landmarks = []
        self.landmark_deltas = []
        self.matching_landmarks = []
        self.last_estimated_position = np.array([0., 0.])
        self.num_observations = 0
        self.offset = np.array((0.0, 0.0))
        self.unique_landmarks: set = set([])

    def set_offset(self, offset):
        self.last_estimated_position = np.array(offset)
        if len(self.deltas) > 0:
            self.deltas[0] = offset
        else:
            self.landmark_deltas.append([])
            self.deltas.append(offset)
            self.position_landmarks.append([])
    
    def add_position(self, delta, features):
        # first add this new position to our tracking
        self.deltas.append(delta)
        # want to keep track of what landmarks this position is associated with
        self.position_landmarks.append([])
        self.landmark_deltas.append([])
        estimated_position = self.last_estimated_position + delta

        for feature in features:
            estimated_feature_location = feature + estimated_position
            key, position = self.landmark_database.find_matching_landmark(estimated_feature_location)
            # if there are existing landmarks for this position, add them to the list
            if key is not None:
                self.num_observations += 1
                self.matching_landmarks.append(key)
                self.position_landmarks[len(self.deltas) - 1].append(key)
                self.landmark_deltas[len(self.deltas) - 1].append(feature)
                self.unique_landmarks.add(key)
        # Current row form
        # X0 X1 X2 ... Y0 Y1 Y2 ... LX0 LY0 LX1 LY2 ...
        #
        # current col Form
        # X0
        # X1
        # ...
        # Y0
        # Y1
        # ...
        # X0-landmark_i0
        # Y0-landmark_i0
        # ...


        # TODO(HEIDT) transfer to a sparse matrix
        # generate the matrix and vector, multiply sizes by two as this is a 2D problem
        matsize = len(self.deltas)*2 + len(self.unique_landmarks)*2
        observation_size = self.num_observations*2 + len(self.deltas)*2
        mat = np.zeros((observation_size, matsize))
        vec = np.zeros(observation_size)

        # fill out relative constraints for X
        mat[0,0] = 1
        for i in range(len(self.deltas[:-1])):
            mat[i+1, i] = -1
            mat[i+1, i+1] = 1

        # fill out relative constraints for Y
        plen = len(self.deltas)
        mat[plen, plen] = 1
        for i, position in enumerate(self.deltas[:-1]):
            mat[i + 1 + plen, i + plen] = -1
            mat[i + 1 + plen, i + 1 + plen] = 1

        # fill out vector values
        #print("positions: ", self.positions)
        vec[0] = self.deltas[0][0]

        # doing first part of the matrix for X positions then second part for Y instead of interleaving
        for i, position in enumerate(self.deltas[1:]):
            x_diff = position[0]
            vec[i + 1] = x_diff

        vec[plen] = self.deltas[0][1]
        for i, position in enumerate(self.deltas[1:]):
            y_diff = position[1]
            vec[i + plen + 1] = y_diff

        col = len(self.deltas)*2
        landmark_keys = {}
        for landmark_id in self.unique_landmarks:
            landmark_keys[landmark_id] = col
            col += 2

        # fill out relative constraints for observations
        
        row = len(self.deltas)*2
        for j, landmark_list in enumerate(self.position_landmarks):
            if len(landmark_list) > 0:
                #print("landmark lengths: ", len(landmark_list), len(self.landmark_deltas))
                for i, landmark_id in enumerate(landmark_list):
                    # fill in for the x coordinate
                    mat[row, j] = -1
                    mat[row, landmark_keys[landmark_id]] = 1
                    vec[row] = self.landmark_deltas[j][i][0]
                    row += 1
                    # Do the same for the Y coordinate
                    mat[row, j + plen] = -1
                    mat[row, landmark_keys[landmark_id]+1] = 1
                    vec[row] = self.landmark_deltas[j][i][1]
                    row += 1

        #print("mat: ")
        #print(mat)
        #print("vec: ")
        #print(vec)
        #print("shapes")
        #print(mat.shape, vec.shape)

        res = np.linalg.lstsq(mat, vec)
        new_positions = []
        for i in range(len(self.deltas)):
            new_pos = np.array((res[0][i], res[0][i+len(self.deltas)]))
            new_positions.append(new_pos)

        # Update all the landmarks
        for key, col in landmark_keys.items():
            position = np.array([res[0][col], res[0][col+1]])
            self.landmark_database.update_position(key, position)

        # Update all the deltas
        #for i, position in enumerate(new_positions[1:]):
            #print("subtracting: ", new_positions[i], new_positions[i-1], new_positions[i] - new_positions[i-1])
            #self.deltas[i+1] = new_positions[i+1] - new_positions[i]
            
        #print("deltas: \n", self.deltas, "\n", new_deltas, "\n", new_positions)
        print(f"processed {len(self.deltas)} positions and {len(self.unique_landmarks)} unique landmarks")

        self.last_estimated_position = new_positions[-1]
        return np.array(new_positions)


world = World("floorplan.png")

window = pyglet.window.Window(world.world_grid.shape[1], world.world_grid.shape[0], "PiSLAM SIM")
image = pyglet.resource.image(world.world_grid_filename)

print(world.cast_ray_ideal((300, 300), np.pi/4))

mouse_pos = 0, 0
batch = pyglet.graphics.Batch()
slam = graphSLAM()
true_positions = []
updated_positions = []
estimated_positions = []
dedrec_positions = []

@window.event
def on_draw():
    window.clear()
    image.blit(0, 0)
    rays = world.cast_ideal_rays(mouse_pos, 360)
    raygrid = world.fill_point_grid(rays)
    spikes = world.find_spikes(rays)
    world.draw_spikes(raygrid, spikes)
    for i, position in enumerate(true_positions):
        est = (int(estimated_positions[i][0]), int(estimated_positions[i][1]))
        ded = (int(dedrec_positions[i][0]), int(dedrec_positions[i][1]))
        cv2.circle(raygrid, position, 4, (0, 255, 0), 2)
        cv2.circle(raygrid, est, 4, (255, 0, 0), 2)
        cv2.circle(raygrid, ded, 4, (0, 0, 255), 2)
        if i < len(true_positions) - 1:
            est2 = (int(estimated_positions[i+1][0]), int(estimated_positions[i+1][1]))
            ded2 = (int(dedrec_positions[i+1][0]), int(dedrec_positions[i+1][1]))
            cv2.line(raygrid, position, true_positions[i+1], (0, 255, 0), 1)
            cv2.line(raygrid, est, est2, (255, 0, 0), 1)
            cv2.line(raygrid, ded, ded2, (0, 0, 255), 1)

    cv2.imshow('frame', raygrid)
    cv2.waitKey(1)
    lines = []
    for ray in rays:
        if ray is not None:
            lines.append(shapes.Line(mouse_pos[0], mouse_pos[1], ray[0], ray[1], color = (250, 30, 30), batch = batch))
    batch.draw()

@window.event
def on_mouse_motion(x, y, dx, dy):
    global mouse_pos
    mouse_pos = x, y

@window.event
def on_mouse_press(x, y, button, modifiers):
    global true_positions, estimated_positions, dedrec_positions
    true_positions.append((x, y))
    inverted = (x, world.world_grid.shape[0] - y)
    position = np.array(inverted, dtype=np.float)

    if len(true_positions) == 1:
        estimated_positions = [np.array((x, y))]
        dedrec_positions.append(np.array((x, y)))
        slam.set_offset(np.array(inverted))
    else:
        start_time = time.time()
        rays = np.array(world.cast_ideal_rays((x, y), 100))
        rays[:, 1] = world.world_grid.shape[0] - rays[:, 1]
        spikes = np.array(world.find_spikes(rays))

        last_position = np.array(true_positions[-2])
        last_position[1] = world.world_grid.shape[0] - last_position[1]

        delta = position - last_position
        delta += np.random.rand(2)*5.0
        dedrec_positions.append(dedrec_positions[-1] + np.array((delta[0], -delta[1])))

        relative_spikes = spikes - position
        
        estimated_positions = slam.add_position(delta, relative_spikes)
        estimated_positions[:,1] = world.world_grid.shape[0] - estimated_positions[:,1]
        end_time = time.time()
        print(f"Total time to process: {end_time-start_time} seconds")

    

pyglet.app.run()


