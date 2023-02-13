import get_task_paths
import setup_path
import airsim

import sys
import time

def run_nh_circle_path(speed):

    print("Connecting to client")
    client = airsim.MultirotorClient()
    print("Confirming connection")
    client.confirmConnection()
    print("Enabling api control")
    client.enableApiControl(True)

    # Set position of drone
    pose = client.simGetVehiclePose()
    pose.position.x_val = 0
    pose.position.y_val = 0
    pose.position.z_val = 0
    client.simSetVehiclePose(pose, True)
    time.sleep(0.1)

    print("arming the drone...")
    client.armDisarm(True)

    state = client.getMultirotorState()
    if state.landed_state == airsim.LandedState.Landed:
        print("taking off...")
        client.takeoffAsync().join()
    else:
        client.hoverAsync().join()

    time.sleep(1)

    state = client.getMultirotorState()
    if state.landed_state == airsim.LandedState.Landed:
        print("take off failed...")
        sys.exit(1)

    # AirSim uses NED coordinates so negative axis is up.
    # z of -5 is 5 meters above the original launch point.
    z_base = -5
    print("make sure we are hovering at {} meters...".format(-z_base))
    client.moveToZAsync(z_base, 1).join()

    # see https://github.com/Microsoft/AirSim/wiki/moveOnPath-demo

    # this method is async and we are not waiting for the result since we are passing timeout_sec=0.

    for z in range(z_base,-20,-5):
        print(f"flying on path with height: {-z}")
        result = client.moveOnPathAsync([airsim.Vector3r(125,0,z),
                                        airsim.Vector3r(125,-130,z),
                                        airsim.Vector3r(0,-130,z),
                                        airsim.Vector3r(0,0,z)],
                                speed, 120,
                                airsim.DrivetrainType.ForwardOnly, airsim.YawMode(False,0), 20, 1).join()

    # drone will over-shoot so we bring it back to the start point before landing.
    client.moveToPositionAsync(0,0,z_base,1).join()
    print("landing...")
    client.landAsync().join()
    print("disarming...")
    client.armDisarm(False)
    client.enableApiControl(False)
    print("done.")

# if __name__ == "__main__":
#     run_nh_circle_path()