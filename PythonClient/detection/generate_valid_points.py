import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import time
import threading
import filecmp

class MapDescription():
    def __init__(self,xmin,xmax,xstep,ymin,ymax,ystep,zmin,zmax,zstep) -> None:
        self.xmin, self.xmax, self.xstep = xmin, xmax, xstep
        self.ymin, self.ymax, self.ystep = ymin, ymax, ystep
        self.zmin, self.zmax, self.zstep = zmin, zmax, zstep

def generate_all_points(map_description : MapDescription):
    return np.array([   (x, y, z) 
                        for x in range(int(map_description.xmin),int(map_description.xmax+1),int(map_description.xstep))
                        for y in range(int(map_description.ymin),int(map_description.ymax+1),int(map_description.ystep))
                        for z in range(int(map_description.zmin),int(map_description.zmax+1),int(map_description.zstep))])

def slow_generate_valid_points(obstacle_points, all_points, save_filename = None, distance_threshold : int = 10):

    # Loop over all_points and remove any that are close to obstacle points
    for i in range(len(all_points)):
        for j in range(len(obstacle_points)):
            distance = np.linalg.norm(all_points[i] - obstacle_points[j])
            if distance < distance_threshold:
                all_points[i] = np.zeros_like(all_points[i])
                
    # Remove any zero values from all_points
    valid_points = all_points[~np.all(all_points == 0, axis=1)]

    # Save the valid points
    if(save_filename!=None):
        np.savetxt(save_filename, valid_points, delimiter=',')        

    return valid_points

def fast_generate_valid_points(obstacle_points, all_points, save_filename = None, distance_threshold: int = 10):

    # FIXME: could probably iterate through obstacle points instead somehow
    valid_points = []
    for point in all_points:
        distances = np.linalg.norm(point - obstacle_points, axis=1)
        if np.all(distances >= distance_threshold):
            valid_points.append(point)
    valid_points = np.array(valid_points)

    # Save the valid points
    if(save_filename!=None):
        np.savetxt(save_filename, valid_points, delimiter=',')        

    return valid_points

def benchmark_generation(generation_func, map_description : MapDescription,save_filename=None):
    print("loading obstacles")
    # load obstacle points
    obstacle_points = np.genfromtxt(DEFAULT_OBSTACLE_FILENAME, delimiter=',')
    generation_thread = threading.Thread(target=generation_func,args=(obstacle_points,map_description,save_filename))
    # Generate the valid list
    print("Generating the valid list of points... ")
    start_time = time.time()
    generation_thread.start()
    while generation_thread.is_alive():
        print(f"runtime: {round(time.time()-start_time,2)}s",end='\r')
    stop_time = time.time()
    time_to_generate = stop_time-start_time
    # Save the valid points
    print(f"\nSaving valid points, generated in {round(time_to_generate,2)}s")

if __name__ == "__main__":
    PLOT_X_MIN = -90
    PLOT_X_MAX = 90
    PLOT_X_STEP = 10
    PLOT_Y_MIN = -90
    PLOT_Y_MAX = 90
    PLOT_Y_STEP = 10
    PLOT_Z_MIN = -30
    PLOT_Z_MAX = 5
    PLOT_Z_STEP = 5
    DEFAULT_VALID_FILENAME = 'valid_points_plot.csv'
    DEFAULT_OBSTACLE_FILENAME = 'lidar_plot.csv'

    map = MapDescription(
        PLOT_X_MIN, PLOT_X_MAX, PLOT_X_STEP,
        PLOT_Y_MIN, PLOT_Y_MAX, PLOT_Y_STEP,
        PLOT_Z_MIN, PLOT_Z_MAX, PLOT_Z_STEP
    )
    # Generate array of all possible options
    all_points = generate_all_points(map)
    print("Benchmarking slow algorithm")
    benchmark_generation(slow_generate_valid_points, all_points, 'slow_algo_valid_points.csv')
    print("Benchmarking new algorithm")
    benchmark_generation(fast_generate_valid_points, all_points, 'fast_algo_valid_points.csv')