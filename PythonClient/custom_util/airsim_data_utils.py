import airsim
from airsim.types import GeoPoint,Vector3r
import math
from numpy import uint64

def get_gps_coordinates(client: airsim.MultirotorClient) -> GeoPoint:
    gps_data = client.getGpsData()
    return gps_data.gnss.geo_point

def get_gps_velocity(client: airsim.MultirotorClient) -> Vector3r:
    gps_data = client.getGpsData()
    return gps_data.gnss.velocity

def get_gps_timestamp(client: airsim.MultirotorClient) -> uint64:
    gps_data = client.getGpsData()
    return gps_data.time_stamp

def get_multirotor_position(client: airsim.MultirotorClient) -> Vector3r:
    kinematics = client.getMultirotorState().kinematics_estimated
    return kinematics.position

def get_multirotor_linear_velocity(client: airsim.MultirotorClient) -> Vector3r:
    kinematics = client.getMultirotorState().kinematics_estimated
    return kinematics.linear_velocity