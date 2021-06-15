from Draw import Draw
from Arguments import Arguments
from detect_static_objects import Static
import numpy as np
import pandas as pd
import math
import matplotlib.pyplot as plt
from google.protobuf import text_format
from scipy.signal import savgol_filter
from derivative import  dxdt
import random
from fill_vcd_with_static_objects import Filler


def long_lat_distance(objects_of_interest,tracks,j):
    Vehicle,PorC = Vehicle_or_Pedestrian(objects_of_interest,tracks)
    v = np.array([Vehicle.states[j].velocity_x,Vehicle.states[j].velocity_y])
    #normalized vector v
    v_norm = v / (v**2).sum()**0.5
    n_norm =  np.array([Vehicle.states[j].center_x-Vehicle.states[j-1].center_x ,Vehicle.states[j].center_y -Vehicle.states[j-1].center_y])
    n_norm = n_norm / ((n_norm**2).sum())**0.5

    d12 = np.array([PorC.states[j].center_x - Vehicle.states[j].center_x,PorC.states[j].center_y - Vehicle.states[j].center_y ])
    d12_norm = d12 / (d12**2).sum()**0.5

    cosPhi = np.dot(n_norm,d12_norm)
    
    sinPhi = 1- cosPhi**2
    return ((d12**2).sum()**0.5)*cosPhi,((d12**2).sum()**0.5)*sinPhi


def Vehicle_or_Pedestrian(objects_of_interest,tracks):
    Vehicle = None
    PorC = None
    for track in tracks:
        for object_of_interest in objects_of_interest:
            if track.id == object_of_interest:
                if track.object_type == 1:
                    Vehicle = track
                else :
                    PorC  = track
    return Vehicle,PorC


def Turn(Vehicle):
    start_direction = np.array([Vehicle.states[1].center_x-Vehicle.states[0].center_x,Vehicle.states[1].center_y-Vehicle.states[0].center_y,0])
    start_direction = start_direction / (start_direction**2).sum()**0.5
    end_direction = np.array([Vehicle.states[-1].center_x-Vehicle.states[-2].center_x,Vehicle.states[-1].center_y-Vehicle.states[-2].center_y,0])
    end_direction = end_direction / (end_direction**2).sum()**0.5

    # calculate the cross product to determine new vector
    left_or_right_turn_vector = np.cross(start_direction,end_direction)
    # normalize the new vector
    normalized_left_or_right_turn_vector = left_or_right_turn_vector / (left_or_right_turn_vector**2).sum()**0.5

    # threshold the right and left turn -->
    #LEFT when |new_vector| > sin(45)
    #RIGHT when |new_vector| < -sin(45)
    if  left_or_right_turn_vector[2] > math.sin(np.pi/4): 
        return "left"
    elif left_or_right_turn_vector[2] < - math.sin(np.pi/4):
        return "right"
    elif left_or_right_turn_vector[2] <= math.sin(np.pi/4) and left_or_right_turn_vector[2] >= - math.sin(np.pi/4):
        return "Following lane"


scenario_number = 273

# parse .proto file with Arguments class
Argument = Arguments()
draw = Draw()
# pass scenario number to parse only a particular scenario
# creates a scene object with all relevant information of the scenario
# eg. vehicle positions, pedestrian positions, Cyclists positions, 
# road parts included in scenario ...
scene = Argument.return_desired_scenario(scenario_number)
# extract only objects with interesting behaviour
objects_of_interest = scene.objects_of_interest
Vehicle,PorC = Vehicle_or_Pedestrian(objects_of_interest,scene.tracks)
Map = scene.map_features

#detect static objects that the EgoVehicle interacts with
static = Static(Vehicle,Map)
static.fill_up_objects()
#instantiate the Filler class
Filler = Filler()
# Detect dynamic actions from the participants in a scenario
Filler.fill_with_moving_actions(scene,objects_of_interest)
Filler.fill_with_turning_actions(scene,objects_of_interest)
Filler.fill(static,Vehicle,scenario_number)



draw.vehicle_speed(scene,objects_of_interest,include_action=True,get_car=True)
draw.yaw_and_yaw_rate(scene,objects_of_interest,include_action=True)
draw.distance_traveled(Vehicle)
plt.show()
draw.draw_vehicle_tracks(Vehicle.states)
draw.draw_vehicle_tracks(PorC.states)
draw.draw_scenario_crosswalks(Map,static.junction["Junction_crosswalks"])
plt.show()



