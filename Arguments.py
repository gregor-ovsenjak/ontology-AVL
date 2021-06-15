import sys
sys.path.append(r'C:\Users\grego\MachineLearning\Keras_TensorFlow\Waymo\waymo-open-dataset-master\waymo_open_dataset')
import numpy as np
import tensorflow as tf
import protos.scenario_pb2 as scenario


class Arguments:


    def __init__(self):
        self.FILENAME = "uncompressed_scenario_validation_validation.tfrecord-00052-of-00150"
        self.scene = None


    def return_desired_scenario(self,scenario_number):
        dataset = tf.data.TFRecordDataset(self.FILENAME, compression_type='')
        # loop through scenarios 
        for scenario_number_from_dataset,data in enumerate(dataset) :
            # choose the parsed scenario
            if scenario_number_from_dataset == scenario_number:
                # parse the scenario from tfrecord file
                scene = scenario.Scenario()
                scene.ParseFromString(bytearray(data.numpy()))
                self.scene = scene
            else:
                pass

        #return error if no scene found
        #else return scene
        if self.scene != None: 
            return self.scene
        else:
            raise ValueError("No scenario found with this number")
        

