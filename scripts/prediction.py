#!/usr/bin/python3



# ros
from scipy.ndimage.measurements import label
import rospy
import ros_numpy
import message_filters
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float32MultiArray, MultiArrayLayout, MultiArrayDimension
from SegmentationMapping.msg import ImageLabelDistribution
from pprint import pprint

# map label to color
from label2color import label_to_color, background
from NeuralNetConfigs import NeuralNetConfigs
# visualize pcl
#from helper import publish_pcl_pc2_label

import cv2, time
import numpy as np
from threading import Lock
from scipy.ndimage import rotate, zoom

import sys, os
if sys.version_info[0] < 3:
    import Queue as queue
else:
    import queue

import tensorflow.compat.v1 as tf
tf.disable_v2_behavior()
    
# debug
import pdb

# Extra
import PIL
import time



class SegmentationNode:
    def __init__(self):

        rospy.init_node('~', anonymous=True)
        
        neural_net_input_width = 640
        neural_net_input_height = 480
        #neural_net_input_width = 566
        #neural_net_input_height = 421
        
        # Load the network configurations into a tuple
        self.network_config = NeuralNetConfigs("/home/benoit/segmentation_mapping/git/SegmentationMapping/config/mobilenet_nclt/optimized_graph_mobilenet_trt32_2019-06-15.pb",\
                                               14,\
                                               "network/input/Placeholder:0",\
                                               "", \
                                               neural_net_input_width,\
                                               neural_net_input_height, \
                                               "network/output/ClassIndexPrediction:0",\
                                               "network/output/ClassDistribution:0",\
                                               True)
        print(self.network_config)
        
        # Initialize the neural network
        self.tf_init()
        
        self.counter = 0
        self.callback()

    def callback(self):        
        print("segmentation")
        now  = rospy.Time.now()

        #original_img = self.bridge.imgmsg_to_cv2(img_msg , desired_encoding="rgb8")
        original_img = cv2.imread("/home/benoit/segmentation_mapping/data/input.png", cv2.IMREAD_COLOR)
        #original_img = cv2.cvtColor(original_img, cv2.COLOR_BGR2RGB)
        labeled_img, distribution = self.infer(original_img)
        #self.publish_label_and_distribution(labeled_img, distribution, img_msg.header)
        
    def tf_init(self):
        print("open graph..")
        with tf.gfile.GFile(self.network_config.path, 'rb') as f:
            self.graph_def = tf.GraphDef()
            self.graph_def.ParseFromString(f.read())
        print("open graph complete")

        config = tf.ConfigProto()
        config.gpu_options.allow_growth = True
        self.sess = tf.Session(config=config)
        self.y, = tf.import_graph_def(self.graph_def, return_elements=[self.network_config.label_output_tensor], name='')
        self.G = tf.get_default_graph()
        self.x = self.G.get_tensor_by_name(self.network_config.image_input_tensor)
        
        self.dist = self.G.get_tensor_by_name(self.network_config.distribution_output_tensor)
        tf.global_variables_initializer().run(session=self.sess)
        print("Tf init finish")


    def publish_label_and_distribution(self, labeled_img, distribution, header, lidar_header=None):
        now  = rospy.Time.now()
        rospy.loginfo("Going to publish at time %i %i\n\n", now.secs, now.nsecs)
        
        label_msg = self.bridge.cv2_to_imgmsg(labeled_img, encoding="mono8")
        label_msg.header = header

        distribution_msg = ImageLabelDistribution()
        m_arr = Float32MultiArray()
        m_arr.layout.data_offset = 0
        m_arr.layout.dim = [MultiArrayDimension() for _ in range(3)]
        m_arr.layout.dim[0].label = "h"
        m_arr.layout.dim[0].size  = labeled_img.shape[0]
        m_arr.layout.dim[0].stride = self.network_config.num_classes * labeled_img.size
        m_arr.layout.dim[1].label = "w"
        m_arr.layout.dim[1].size  = labeled_img.shape[1]
        m_arr.layout.dim[1].stride = self.network_config.num_classes * labeled_img.shape[1]
        m_arr.layout.dim[2].label = "c"
        m_arr.layout.dim[2].size = self.network_config.num_classes
        m_arr.layout.dim[2].stride = self.network_config.num_classes
        m_arr.data = distribution.flatten().tolist()

        
        distribution_msg.header = header
        distribution_msg.distribution = m_arr

        self.labeled_img_pub.publish(label_msg)
        self.distribution_pub.publish(distribution_msg)

        #if lidar_header is not None:
        #    distribution_msg.header.stamp = lidar_header.stamp
        #    self.distribution_at_lidar_time_pub.publish(distribution_msg)

        now  = rospy.Time.now()
        rospy.loginfo("End callabck at time %i %i\n\n", now.secs, now.nsecs)

    def infer(self, rgb_img):

        num_class = self.network_config.num_classes

        #if rgb_img.shape[0] != self.network_config.input_height or \
        #   rgb_img.shape[1] != self.network_config.input_width:
        #    rgb = cv2.resize(rgb_img, \
        #                     dsize=(self.network_config.input_width, self.network_config.input_height),\
        #                     interpolation=cv2.INTER_CUBIC)
        #else:
        #    rgb = rgb_img
        
        rgb = rgb_img 
        rgb = np.expand_dims(rgb, axis=0)
        
        label_out, dist_out = self.sess.run([self.y, self.dist], feed_dict={self.x: rgb})

        now  = rospy.Time.now()

        dist_out = dist_out[0, :, :, :]
        label_out = label_out[0, :, : ].astype(np.uint8)
        rospy.loginfo("after segmentation time %i %i", now.secs, now.nsecs )
        
        
        # Generating colored output
        # start_time = time.time()
        if self.network_config.enable_colored_labeling:
            colored_label_out = np.zeros(shape=(label_out.shape[0], label_out.shape[1], 3))
            for i in range(len(colored_label_out)):
                for j in range(len(colored_label_out[i])):
                    colored_label_out[i][j] = np.array(label_to_color[label_out[i][j]]) # convert labels into color
            colored_label_out = colored_label_out.astype(np.uint8) # turn into integer
            # end_time = time.time()
            # print("TIME TAKEN: ", end_time - start_time)
            pil_image = PIL.Image.fromarray(colored_label_out, 'RGB')
            opencv_image = cv2.cvtColor(np.array(pil_image), cv2.COLOR_RGB2BGR)
            cv2.imwrite("/home/benoit/segmentation_mapping/data/output.png", opencv_image)
        

        return label_out, dist_out

    def spin(self):
        rospy.spin()

if __name__ == "__main__":
    node = SegmentationNode()
    #node.spin()
