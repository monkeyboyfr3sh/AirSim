import airsim
from airsim.types import GeoPoint,Vector3r,CollisionInfo
from airsim.client import LidarData
import numpy

class AirSimClientManager():
    def __init__(self, airsim_path: str = None) -> None:

        # Connect to AirSim
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()        

        # Init data
        self.init_client_data()

        # Get sim start timestamp
        self.start_timestamp = self.get_gps_timestamp()

    def simPause(self, pause_set):
        self.client.simPause(pause_set)

    def sample_client_data(self):
        
        # Get velocity and coordinates
        # velocity = airsim_utils.get_gps_velocity(client)
        self.velocity = self.get_multirotor_linear_velocity()
        self.velocity_magnitude = self.velocity.get_length()
        self.timestamp = (self.get_gps_timestamp()-self.start_timestamp) / 1000000000
        self.position = self.get_multirotor_position()
        self.collision_info = self.client.simGetCollisionInfo()
        if(self.collision_info.has_collided):
            self.collision_handler(self.collision_info)

        # Add the new data
        self.x_coord.append(self.position.x_val)
        self.y_coord.append(self.position.y_val)
        self.z_coord.append(-self.position.z_val)
        
        self.v_x.append(self.velocity.x_val)
        self.v_y.append(self.velocity.y_val)
        self.v_z.append(self.velocity.z_val)
        
        self.timestamp_list.append(self.timestamp)
        self.speed_list.append(self.velocity_magnitude)

        # lidar_data = self.client.getLidarData()
        # self.lidar_data_list.append(lidar_data)
        # points = self.parse_lidarData(lidar_data)
        # # print("time_stamp: %d number_of_points: %d" % (lidar_data.time_stamp, len(points)))
        # # print( numpy.shape(points), numpy.average(points) )
        # self.lidar_points_average_list.append(numpy.average(points))
        # # print("\t\tlidar position: %s" % (pprint.pformat(lidar_data.pose.position)))
        # # print("\t\tlidar orientation: %s" % (pprint.pformat(lidar_data.pose.orientation)))

    def collision_handler(self, collision_info: CollisionInfo):
        # First pause sim
        self.simPause(True)

        timestamp = (collision_info.time_stamp-self.start_timestamp) / 1000000000

        self.collision_x_data_list.append(collision_info.position.x_val)
        self.collision_y_data_list.append(collision_info.position.y_val)
        self.collision_z_data_list.append(-collision_info.position.z_val)
        self.collision_time_data_list.append(timestamp)

        # Now resume the sim
        self.simPause(False)

    def parse_lidarData(self, data):

        # reshape array of floats to array of [X,Y,Z]
        points = numpy.array(data.point_cloud, dtype=numpy.dtype('f4'))
        points = numpy.reshape(points, (int(points.shape[0]/3), 3))
       
        return points

    def init_client_data(self):
        self.timestamp_list = []
        self.x_coord = []
        self.y_coord = []
        self.z_coord = []
        self.v_x = []
        self.v_y = []
        self.v_z = []
        self.speed_list = []
        self.lidar_data_list = []
        self.lidar_points_average_list = []
        
        self.collision_x_data_list = []
        self.collision_y_data_list = []
        self.collision_z_data_list = []
        self.collision_time_data_list = []

    def get_gps_coordinates(self) -> GeoPoint:
        gps_data = self.client.getGpsData()
        return gps_data.gnss.geo_point

    def get_gps_velocity(self) -> Vector3r:
        gps_data = self.client.getGpsData()
        return gps_data.gnss.velocity

    def get_gps_timestamp(self) -> numpy.uint64:
        gps_data = self.client.getGpsData()
        return gps_data.time_stamp

    def get_multirotor_position(self) -> Vector3r:
        kinematics = self.client.getMultirotorState().kinematics_estimated
        return kinematics.position

    def get_multirotor_linear_velocity(self) -> Vector3r:
        kinematics = self.client.getMultirotorState().kinematics_estimated
        return kinematics.linear_velocity