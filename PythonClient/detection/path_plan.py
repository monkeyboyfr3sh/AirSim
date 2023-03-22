from enum import Enum, auto
import argparse
import time
import numpy as np

from planning_utils import create_grid, a_star, path_prune, heuristic, pickup_goal, \
collinear_points, path_simplify, convert_25d_3d, draw_path

TARGET_ALTITUDE = 10
SAFETY_DISTANCE = 2 

class MotionPlanning:

    def __init__(self):
        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = []
        self.in_mission = True
        self.check_state = {}

        self.global_position = (0,0,0)
        self.interactive_goal = (-122.40017151, 37.7962347, 0)
        self.temporary_scatter = None
        self.previous_location = None
        self.map_grid = None
        self.north_offset = None
        self.east_offset = None
        self.path = None

    def pick_goal(self, event):
        evt = event.mouseevent
        east = int(evt.xdata)
        north = int(evt.ydata)
        alt = self.map_grid[north, east]
        # self.interactive_goal = local_to_global(self.grid_to_local((north, east, alt)), self.global_home)
        self.interactive_goal = self.grid_to_local((north, east, alt))

        if self.temporary_scatter is not None:
            self.temporary_scatter.remove()
        fig = event.artist.figure
        self.temporary_scatter = fig.gca().scatter(east, north, marker='o', c='g')
        fig.canvas.draw()
        print("The goal location is (lat, lon, alt) {}"
                "Close figure to start simulator.".format(self.interactive_goal))

    def grid_to_local(self, grid_coord):
        lat = grid_coord[0] + self.north_offset
        lon = grid_coord[1] + self.east_offset
        return lat, lon, -grid_coord[2]

    def local_to_grid(self, position):
        north = int(position[0] - self.north_offset)
        east = int(position[1] - self.east_offset)
        alt = int(-position[2])
        return north, east, alt

    def get_map_home(self):
        with open('colliders.csv', 'r') as f:
            header_line = f.readline()
            lat_str, lon_str = header_line.split(',')
            lat = float(lat_str.strip().split(' ')[1])
            lon = float(lon_str.strip().split(' ')[1])
            print("Map home location: ({}, {})".format(lat, lon))

    def interactive_plan_path(self):
        self.get_map_home()

        print("Searching path...")
        self.target_position[2] = TARGET_ALTITUDE
        local_position = self.global_position
        print('position {0}, local position {1}'.format(    self.global_position,
                                                            local_position))
        
        # Read in obstacle map
        data = np.loadtxt('colliders.csv', delimiter=',', dtype=np.float64, skiprows=2)

        # Define a grid for a particular altitude and safety margin around obstacles
        grid, north_offset, east_offset = create_grid(data, SAFETY_DISTANCE)
        self.map_grid = grid
        self.north_offset = north_offset
        self.east_offset = east_offset

        print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))
        # starting point on the grid
        self.grid_start = self.local_to_grid(local_position)
        alt_start = int(max(TARGET_ALTITUDE, self.grid_start[2] + 1, grid[self.grid_start[0], self.grid_start[1]] + 1))
        self.grid_start = self.grid_start[0], self.grid_start[1], alt_start

        # visualize grid: interavtive goal pick up
        # self.temporary_scatter = pickup_goal(grid, self.grid_start, self.pick_goal)
        self.temporary_scatter = pickup_goal(grid, self.grid_start, self.live_path_plan)

    def live_path_plan(self, event):
        evt = event.mouseevent
        east = int(evt.xdata)
        north = int(evt.ydata)
        alt = self.map_grid[north, east]
        # self.interactive_goal = local_to_global(self.grid_to_local((north, east, alt)), self.global_home)
        self.interactive_goal = self.grid_to_local((north, east, alt))

        if self.temporary_scatter is not None:
            self.temporary_scatter.remove()
        fig = event.artist.figure
        self.temporary_scatter = fig.gca().scatter(east, north, marker='o', c='g')
        fig.canvas.draw()
        self.path_plan()

    def path_plan(self):
        goal = self.interactive_goal
        if len(goal) < 3:
            goal = (goal[0], goal[1], 0)
        goal_local = goal
        goal_grid = self.local_to_grid(goal_local)
        goal_north, goal_east, goal_alt = goal_grid
        grid_goal = (goal_north,
                    goal_east,
                    int(max(self.map_grid[goal_north, goal_east] + 1, TARGET_ALTITUDE, goal_alt + 1)))

        print('Start and Goal location', self.grid_start, grid_goal)
        print("Searching path...")
        path = a_star(self.map_grid, heuristic, self.grid_start, grid_goal, TARGET_ALTITUDE)
        path = path_prune(path, collinear_points)
        print("3D Pruned Path:", path)
        path = path_simplify(self.map_grid, path)
        print("Path found!")
        print(path)
        self.path = path
        waypoints = self.path_to_waypoints(path)
        self.waypoints = waypoints
        draw_path(self.map_grid,self.grid_start,grid_goal,path)

    def path_to_waypoints(self, path):
        # Convert path to waypoints
        waypoints = []
        for i in range(len(path)):
            p = path[i]
            p_prev = path[i - 1] if i > 0 else None
            orientation = 0
            if p_prev is not None:
                orientation = np.arctan2(p[1] - p_prev[1], p[0] - p_prev[0])
            waypoints.append([p[0] + self.north_offset, p[1] + self.east_offset, p[2], orientation])
        return waypoints
    
if __name__ == "__main__":
    mission = MotionPlanning()
    mission.interactive_plan_path()