import argparse
import datetime
import os
import time
import itertools
import sys
import imageio
sys.path.append(r'C:\Users\grego\MachineLearning\Keras_TensorFlow\Waymo\waymo-open-dataset-master\waymo_open_dataset')
import numpy as np
import tensorflow as tf
import struct
import cv2
import uuid
import pandas as pd
import protos.scenario_pb2 as scenario
import dataset_pb2 as open_dataset
import protos.map_pb2 as map_proto
from PIL import Image 
from utils import range_image_utils
from utils import transform_utils
import math
from matplotlib import cm
import matplotlib.animation as animation
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from google.protobuf import text_format
from mpl_toolkits.mplot3d import Axes3D
import random as random
FILENAME = "uncompressed_scenario_validation_validation.tfrecord-00052-of-00150"


def create_figure_and_axes(size_pixels):
  """Initializes a unique figure and axes for plotting."""
  fig, ax = plt.subplots(1, 1, num=uuid.uuid4())

  # Sets output image to pixel resolution.
  dpi = 100
  size_inches = size_pixels / dpi
  fig.set_size_inches([size_inches, size_inches])
  fig.set_dpi(dpi)
  fig.set_facecolor('white')
  ax.set_facecolor('white')
  ax.xaxis.label.set_color('black')
  ax.tick_params(axis='x', colors='black')
  ax.yaxis.label.set_color('black')
  ax.tick_params(axis='y', colors='black')
  fig.set_tight_layout(True)
  ax.grid(False)
  return fig, ax


def fig_canvas_image(fig):
  """Returns a [H, W, 3] uint8 np.array image from fig.canvas.tostring_rgb()."""
  # Just enough margin in the figure to display xticks and yticks.
  fig.subplots_adjust(
      left=0.08, bottom=0.08, right=0.98, top=0.98, wspace=0.0, hspace=0.0)
  fig.canvas.draw()
  data = np.frombuffer(fig.canvas.tostring_rgb(), dtype=np.uint8)
  return data.reshape(fig.canvas.get_width_height()[::-1] + (3,))


def draw_map(map_features):
    stop_sign_points = {"x":[],"y":[]}
    map_points = {"x":[],"y":[]}
    crosswalk_points = {}
    road_line_points = {"x":[],"y":[]}
    road_edge_points = {"x":[],"y":[]}
    for i,feature in enumerate(map_features):
        road_line_points = {"x":[],"y":[]}
        #print(polyline.x for polyline in feature.lane.polyline)
        #print(len(feature.lane.polyline),feature.id)
        x = [polyline.x for polyline in feature.lane.polyline]
        map_points['x']=x
        y = [polyline.y for polyline in feature.lane.polyline]
        map_points['y'] = y

        if (feature.stop_sign.position.x != 0.0 and feature.stop_sign.position.y != 0.0 ):
            stop_sign_points['x'].append(feature.stop_sign.position.x)
            stop_sign_points['y'].append(feature.stop_sign.position.y)
            
        # create a new instance of a crosswalk with x and y points 
        crosswalk_points[i] = {"x":[],"y":[]}
        for crosswalk in feature.crosswalk.polygon:
            
            crosswalk_points[i]['x'].append(crosswalk.x)
            crosswalk_points[i]['y'].append(crosswalk.y)
            # draw a filled polygon
            ax.fill(crosswalk_points[i]['x'],crosswalk_points[i]['y'],color='green',alpha=0.5)
        for road_line in feature.road_line.polyline:
            road_line_points['x'].append(road_line.x)
            road_line_points['y'].append(road_line.y)
            
        x = [road_edge.x for road_edge in feature.road_edge.polyline]
        road_edge_points['x']=x
        y = [road_edge.y for road_edge in feature.road_edge.polyline]
        road_edge_points['y']=y
        ax.plot(road_line_points['x'],road_line_points['y'],color='orange',alpha=1)
        ax.plot(map_points['x'],map_points['y'],color=np.random.rand(3,),alpha=0.8)
        ax.plot(road_edge_points['x'],road_edge_points['y'],color=np.random.rand(3,),alpha=0.7)
    ax.scatter(stop_sign_points['x'],stop_sign_points['y'],color='red',marker='x',alpha=1)


def safety_distance(v_x,v_y):
    #safety distance formula = (0.278*t*v) + v^2 / (254*f)
    # v = velocity of the vehicle
    # t = average reaction time 
    # f = traction coeffient (avg=0.7)
    # in km/h
    v = math.sqrt(v_x*v_x +v_y*v_y)*3.6666
    s = 0.278*1.5*(v) + v*v/(254*0.7)
    return s


def check_for_object_type(objects_of_interest,scene_tracks):
    object_types = []
    for object_id in objects_of_interest:
        for track in scene_tracks:
            if track.id == object_id:
                object_types.append(track.object_type)
    
    if object_types[0] != object_types[1] : 
        return True
    else:
        return False


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


def dangerous_situation(objects_of_interest,tracks,j):

    Vehicle,PorC = Vehicle_or_Pedestrian(objects_of_interest,tracks)
    v = np.array([Vehicle.states[j].velocity_x,Vehicle.states[j].velocity_y])
    #normalized vector v
    v_norm = v / (v**2).sum()**0.5

    d12 = np.array([PorC.states[j].center_x - Vehicle.states[j].center_x,PorC.states[j].center_y - Vehicle.states[j].center_y ])
    d12_norm = d12 / (d12**2).sum()**0.5

    cosPhi = np.dot(v_norm,d12_norm)
    sinPhi = 1- cosPhi**2


    if ((d12**2).sum()**0.5)*cosPhi < safety_distance(Vehicle.states[j].velocity_x,Vehicle.states[j].velocity_y) and ((d12**2).sum()**0.5)*sinPhi < Vehicle.states[j].width:
        if np.dot(v_norm,d12_norm) > 0 : 
            return True
        else:
            return False


    





colors = ['darkgrey', 'black','gold','darkmagenta','pink']
levels = [0, 1,2]
dataset = tf.data.TFRecordDataset(FILENAME, compression_type='')



for k,data in enumerate(dataset) :
    
    images = []
    scene = scenario.Scenario()
    scene.ParseFromString(bytearray(data.numpy()))
    AreThereOI = True
    objects_of_interest = scene.objects_of_interest
    
    if len(objects_of_interest) == 0 or len(objects_of_interest)> 2:
            AreThereOI = False
            continue
    
    
    if check_for_object_type(objects_of_interest,scene.tracks) == False:
       continue
    else:


        fig, ax = create_figure_and_axes(size_pixels=1000)
        draw_map(scene.map_features)

        for j,seconds in enumerate(scene.timestamps_seconds):
            #print(scene.objects_of_interest,scene.sdc_track_index)
            
            track_points = {"x":[],"y":[],'color':[]}
            
            print('{j}\'th Map drawn'.format(j=j))
            p1 = []
            p2 = []
            for object_of_interest in objects_of_interest:
                for i,tracks in enumerate(scene.tracks) : 
                    if tracks.states[j].center_x != 0.0 and tracks.states[j].center_y != 0.0 :
                            if object_of_interest == tracks.id:
                                circle3 = plt.Circle((1, 1), 0.2, color='g')
                                
                                situation = dangerous_situation(objects_of_interest,scene.tracks,j)
                                if situation:
                                    plt.text(1, 1,'DANGER',
                                    horizontalalignment='center',
                                    verticalalignment='center',
                                    c='r',
                                    transform = ax.transAxes)
                                l,w = tracks.states[j].length,tracks.states[j].width
                                rect = patches.Rectangle((tracks.states[j].center_x-(0.5*l*math.cos(tracks.states[j].heading) -0.5*w*math.sin(tracks.states[j].heading)),
                                                tracks.states[j].center_y-(0.5*l*math.sin(tracks.states[j].heading) +0.5*w*math.cos(tracks.states[j].heading))),
                                                    l,
                                                    w,
                                                    angle=math.degrees(tracks.states[j].heading),
                                                    linewidth=1,
                                                    edgecolor=colors[tracks.object_type],
                                                    facecolor='none')
                                if tracks.object_type == 1:
                                    rect1 = patches.Rectangle((tracks.states[j].center_x-(0.5*l*math.cos(tracks.states[j].heading) -0.5*w*math.sin(tracks.states[j].heading)),
                                                    tracks.states[j].center_y-(1*l*math.sin(tracks.states[j].heading) +1*w*math.cos(tracks.states[j].heading))),
                                                        safety_distance(tracks.states[j].velocity_x,tracks.states[j].velocity_y)+l,
                                                        2*w,
                                                        angle=math.degrees(tracks.states[j].heading),
                                                        linewidth=1,
                                                        edgecolor=colors[tracks.object_type],
                                                        facecolor='none')
                                    p2.append(ax.add_patch(rect1))
                                track_points['color'].append(colors[tracks.object_type])
                                track_points['x'].append(tracks.states[j].center_x)
                                track_points['y'].append(tracks.states[j].center_y)
                                df = pd.DataFrame(track_points)
                                p1.append(ax.scatter(df['x'],df['y'],c=df['color'],marker='x',alpha=1.0))
                                p2.append(ax.add_patch(rect))
                                #p2.append(ax.add_patch(rect1))
            image = fig_canvas_image(fig)
            images.append(image)
            for p1i in p1:
                p1i.remove()
            for p1i in p2:
                p1i.remove()
            #plt.close(fig)
        if AreThereOI:
            imageio.mimsave('./Scenarios_full/scenario-52/scenario{k}.gif'.format(k=k), images)
        else: 
            continue
plt.close(fig)