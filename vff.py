from GUI import GUI
from HAL import HAL
import math
import numpy as np
# Enter sequential code!

def parse_laser_data (laser_data):
    laser = []
    i = 0
    while (i < 180):
        dist = laser_data.values[i]
        if dist > 10:
            dist = 10
        angle = math.radians(i-90) # because the front of the robot is -90 degrees
        laser += [(dist, angle)]
        i+=1
    return laser
    
def laser_data_to_repulsive_vectors(laser_data):
    repulsive_vectors = []
    for distance, angle in laser_data:
        
        
        if angle>0 and angle<0.07:
          print(distance,end="|")
        if distance==0:
          distance=0.01
        # Calculate x and y components of the vector
        x_component =  np.cos(angle) / distance  # Inversely proportional to distance
        y_component =  np.sin(angle) / distance # Inversely proportional to distance
        repulsive_vectors.append([x_component, y_component])
    print("\n#######################")
    # Sum all repulsive vectors to get the resultant vector
    resultant_vector = -1* np.sum(repulsive_vectors, axis=0)
    
    return resultant_vector/20
  
def convert_to_velocity(resultant_vector, max_linear_speed, max_angular_speed):
    # Calculate linear velocity magnitude
    linear_velocity_magnitude = np.linalg.norm(resultant_vector)
    # Normalize the resultant vector to get its direction
    direction = resultant_vector / linear_velocity_magnitude if linear_velocity_magnitude > 0 else np.array([0, 0])

    # Calculate angular velocity magnitude based on the direction angle
    angle = np.arctan2(direction[1], direction[0])
    angular_velocity_magnitude = angle * max_angular_speed / np.pi

    # Scale linear velocity within the maximum speed limit
    linear_velocity =  min(linear_velocity_magnitude, max_linear_speed)

    return linear_velocity, angular_velocity_magnitude
    
def absolute2relative (x_abs, y_abs, robotx, roboty, robott):

    # robotx, roboty are the absolute coordinates of the robot
    # robott is its absolute orientation
    # Convert to relatives
    dx = x_abs - robotx
    dy = y_abs - roboty

    # Rotate with current angle
    x_rel = dx * math.cos (-robott) - dy * math.sin (-robott)
    y_rel = dx * math.sin (-robott) + dy * math.cos (-robott)

    return x_rel , y_rel
    
def reached_waypoint(point1, point2, threshold):
  # Calculate the Euclidean distance between the two points
  distance = np.linalg.norm(np.array(point1) - np.array(point2))
  
  # Check if the distance is within the threshold
  if distance <= threshold:
      return True
  else:
      return False

def scale_vector(vector, magnitude):
    # Calculate the current magnitude of the vector
    current_magnitude = np.linalg.norm(vector)
    
    # Scale the vector to the desired magnitude
    scaled_vector = vector * (magnitude / current_magnitude)
    
    return scaled_vector

max_linear_speed = 4.0 
max_angular_speed = 4.0
    
while True:
  
    currentTarget = GUI.map.getNextTarget()
    GUI.map.targetx = currentTarget.getPose().x
    GUI.map.targety = currentTarget.getPose().y
  
    
    robotx,roboty,robott = HAL.getPose3d().x,HAL.getPose3d().y,HAL.getPose3d().yaw
    
    if reached_waypoint([robotx,roboty],[GUI.map.targetx,GUI.map.targety],2):
      currentTarget.setReached(True)
    
    
    way_x,way_y = absolute2relative(GUI.map.targetx,GUI.map.targety,robotx,roboty,robott)
    GUI.showLocalTarget([way_x,way_y])
    
    target_vector = scale_vector(np.array([way_x,way_y]),5)
    
    laser = parse_laser_data(HAL.getLaserData())
    
    repulsion_vector = laser_data_to_repulsive_vectors(laser)
    
    direction_vector = target_vector+repulsion_vector
    
    GUI.showForces(direction_vector.tolist(),repulsion_vector.tolist(),target_vector.tolist())
    
    linear,angular = convert_to_velocity(direction_vector,max_linear_speed,max_angular_speed)
  
    
    HAL.setV(linear)
    HAL.setW(angular)
    
    

