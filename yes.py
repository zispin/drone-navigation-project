import airsim
import numpy as np
import math
import time

#connect to the AirSim simulator
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

def get_orange_sphere_position():
    object_info = client.simGetObjectPose("Orangeball")
    return object_info.position

def get_drone_position():
    drone_state = client.getMultirotorState()
    return drone_state.kinematics_estimated.position

def move_drone(position, duration):
    client.moveToPositionAsync(position.x_val, position.y_val, position.z_val, 5, duration).join()

import math
import airsim

def point_camera_at_sphere(drone_pos, sphere_pos):
    dx = sphere_pos.x_val - drone_pos.x_val
    dy = sphere_pos.y_val - drone_pos.y_val
    dz = sphere_pos.z_val - drone_pos.z_val
    
    #calculate yaw (rotation around z-axis)
    yaw = math.atan2(dy, dx)
    
    #calculate distance
    distance = math.sqrt(dx**2 + dy**2 + dz**2)
    
    #check for division by zero
    if distance == 0:
        return 
    
    #calculate pitch (rotation around y-axis)
    pitch = -math.asin(dz / distance)
    
    #convert radians to degrees
    yaw_deg = math.degrees(yaw)
    pitch_deg = math.degrees(pitch)
    
    #set camera orientation
    camera_pose = airsim.Pose(airsim.Vector3r(0, 0, 0), airsim.to_quaternion(pitch_deg, 0, yaw_deg))
    client.simSetCameraPose("0", camera_pose)


def initialize_drone_position(desired_altitude):
    client.takeoffAsync().join()
    drone_pos = get_drone_position()
    move_drone(airsim.Vector3r(drone_pos.x_val, drone_pos.y_val, -desired_altitude), 5)
    print(f"Drone initialized at altitude: {desired_altitude} meters")

def circular_flight():
    desired_altitude = 30 
    radius = 45.72  #150 feet in meters, increased to provide more space
    angular_speed = 0.1  #radians per second

    #initialize drone position
    initialize_drone_position(desired_altitude)

    sphere_pos = get_orange_sphere_position()
    angle = 0

    while True:
        try:
            #calculate new position on the circle with a safety margin
            x = sphere_pos.x_val + radius * math.cos(angle)
            y = sphere_pos.y_val + radius * math.sin(angle)
            z = -desired_altitude

            
            new_position = airsim.Vector3r(x, y, z)
            move_drone(new_position, 1)

            #check for collisions
            collision_info = client.simGetCollisionInfo()
            if collision_info.has_collided:
                print("Collision detected! Resetting drone...")
                initialize_drone_position(desired_altitude)
                time.sleep(2)  
                continue  

            #point camera at the sphere this is bugged need to fix later
            drone_pos = get_drone_position()
            point_camera_at_sphere(drone_pos, sphere_pos)

            
            angle += angular_speed
            if angle > 2 * math.pi:
                angle -= 2 * math.pi

            
            print(f"Drone position: ({drone_pos.x_val:.2f}, {drone_pos.y_val:.2f}, {drone_pos.z_val:.2f})")
            print(f"Angle: {math.degrees(angle):.2f} degrees")
            print("--------------------")

            time.sleep(0.1)  #small delay to prevent overwhelming the simulator

        except KeyboardInterrupt:
            print("Stopping...")
            break

if __name__ == "__main__":
    try:
        circular_flight()
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        client.armDisarm(False)
        client.enableApiControl(False)
