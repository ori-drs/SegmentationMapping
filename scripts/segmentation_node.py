#!/usr/bin/python3

import cv2
import time
import numpy as np

# ros
import rospy
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from SegmentationMapping.msg import ImageLabelDistribution

# tensorflow
import tensorflow as tf

# for defining the model
import segmentation_models as sm
sm.set_framework('tf.keras')
from segmentation_models import get_preprocessing

# map label to color
from label2color import label_to_color
from NeuralNetConfigs import NeuralNetConfigs

# Extra
import PIL
# import os
# os.environ['CUDA_VISIBLE_DEVICES'] = '-1'       # disables GPU usage (uses CPU instead)


class SegmentationNode:
    """Class that performs semantic segmentation
    """
    def __init__(self):
        """Initialize an object of the class
        """
        rospy.init_node('~', anonymous=True)
        
        # Load the network configurations into a tuple
        self.network_config = NeuralNetConfigs(rospy.get_param("~neural_net_graph"),\
                                               rospy.get_param("~num_classes"),\
                                               rospy.get_param("~network_image_input_tensor"),\
                                               "", \
                                               rospy.get_param("~neural_net_input_width"),\
                                               rospy.get_param("~neural_net_input_height"), \
                                               rospy.get_param("~network_label_output_tensor"),\
                                               rospy.get_param("~network_distribution_output_tensor"),\
                                               rospy.get_param("~enable_colored_labeling", False))
        print(self.network_config)

        # Choose the desired backbone for the network
        # self.backbone = 'resnet34'
        self.backbone = 'mobilenet'
        self.preprocess_input = get_preprocessing(self.backbone)

        # Initialize the neural network
        self.tf_init()
                
        # Creating a topic to which colored label image is output
        if self.network_config.enable_colored_labeling:
            self.colored_img_pub = rospy.Publisher("/colored_label_topic",
                                                    Image, queue_size=10)
        
        self.labeled_img_pub = rospy.Publisher("~label_topic",
                                               Image, queue_size=5)
        self.distribution_pub = rospy.Publisher("~distribution_topic",
                                                ImageLabelDistribution, queue_size=5)
        self.distribution_at_lidar_time_pub = rospy.Publisher("~distribution_at_lidar_time_topic",
                                                              ImageLabelDistribution, queue_size=5)
        #self.sub = rospy.Subscriber('~color_topic', Image, self.callback)
        self.input_rgb_img_ = rospy.Subscriber('~color_topic', Image,  self.callback, queue_size=1)
        
        self.skip_img_freq = rospy.get_param('~skip_input_img_freq')
        self.counter = 0
        self.bridge = CvBridge()

    def callback(self, img_msg):
        """Call the inference script on the image when a ros image message arrives

        Args:
            img_msg (sensor_msgs.msg.Image): The ros image message for which semantic segmentation has to be performed.
        """        
        print("segmentaion call back")
        # Perform inference at chosen interval
        if self.counter % self.skip_img_freq != 0:
            self.counter += 1
            return
        else:
            self.counter += 1
        
        now  = rospy.Time.now()
        rospy.loginfo("New callback time %i %i, img_time: %i %i", now.secs, now.nsecs, img_msg.header.stamp.secs, img_msg.header.stamp.nsecs )
        original_img = self.bridge.imgmsg_to_cv2(img_msg , desired_encoding="rgb8")
        #original_img = cv2.cvtColor(original_img, cv2.COLOR_BGR2RGB)
        s_t = time.time()
        labeled_img, distribution = self.infer(original_img)
        e_t = time.time()
        print(f"Prediction time: {e_t - s_t} s")
        self.publish_label_and_distribution(labeled_img, distribution, img_msg.header)
        
    def tf_init(self):
        """Initialize the neural network that perfoms semantic segmentation
        """
        print("Initializing the neural network...")

        # Define the network
        self.model = sm.Unet(self.backbone, encoder_weights='imagenet', classes=self.network_config.num_classes, encoder_freeze=True)
        self.model.load_weights(self.network_config.path)
        
        # Predict some dummy image to load the necessary libraries
        # x = cv2.imread("/home/yemika/Mikael/Work/ODRS/SegMap/network/MobileNetSeg/data/train/images/image4914.png", cv2.IMREAD_COLOR)          # read the image as BGR
        # B, G, R = cv2.split(x)                          # split into 3 channels
        # x = cv2.merge([R, G, B])                        # convert into RGB
        x = np.zeros(shape=(self.network_config.input_height,   # 480
                     self.network_config.input_width,           # 640
                     3)).astype(np.uint8)
        x = self.preprocess_input(x)
        x = x.astype(np.float32)
        x = x[None, :]
        self.model.predict(x)
        print("Network initialization finished")

    def publish_label_and_distribution(self, labeled_img, distribution, header, lidar_header=None):
        """Publish the predicted label and the probability distribution of the classes for each pixel of the image

        Args:
            labeled_img (cv2.Mat): Image of 1 channel, each pixel containing the predicted semantic segmentation label for that pixel.
            distribution (cv2.Mat): Image of n channels, each pixel containing the probability of that pixel belonging to each of the n channels.
            header (std_msgs.msg.Header): Header of the ROS image message
            lidar_header (std_msgs.msg.Header, optional): Header of the ROS LiDAR message. Defaults to None.
        """
        now  = rospy.Time.now()
        rospy.loginfo("Going to publish at time %i %i\n\n", now.secs, now.nsecs)
        
        # Create an image message of the predicted labels
        label_msg = self.bridge.cv2_to_imgmsg(labeled_img, encoding="mono8")
        label_msg.header = header

        # Create a distribution messgae of the probabilities of each class
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
        distribution_msg = ImageLabelDistribution()
        distribution_msg.header = header
        distribution_msg.distribution = m_arr

        # Publish the images
        self.labeled_img_pub.publish(label_msg)
        self.distribution_pub.publish(distribution_msg)

        #if lidar_header is not None:
        #    distribution_msg.header.stamp = lidar_header.stamp
        #    self.distribution_at_lidar_time_pub.publish(distribution_msg)

        now  = rospy.Time.now()
        rospy.loginfo("End callback at time %i %i\n\n", now.secs, now.nsecs)

    def infer(self, rgb_img):
        """Infers semantic labels for the input color image

        Args:
            rgb_img (cv2.Mat): Input color image for which semantic labels have to be pridected.

        Returns:
            (tuple): tuple containing:
                label_out (cv2.Mat): Image of 1 channel, each pixel containing the predicted semantic segmentation label for that pixel.
                dist_out (cv2.Mat): Image of n channels, each pixel containing the probability of that pixel belonging to each of the n channels.
        """
        initial_dimensions = rgb_img.shape
        # rgb_img = rgb_img[:, :640, :]

        # Resize the input image if its size does not match the size expected by the network - tejaswid: not sure why this needs to be done
        if rgb_img.shape[0] != self.network_config.input_height or rgb_img.shape[1] != self.network_config.input_width:
           rgb = cv2.resize(rgb_img,
                            dsize=(self.network_config.input_width, self.network_config.input_height),
                            interpolation=cv2.INTER_CUBIC)
        else:
           rgb = rgb_img

        # Preprocess the input image
        rgb = self.preprocess_input(rgb)
        rgb = rgb[None, :]  # make the dimensions [1, rgb_img.shape[0], rgb_img.shape[1], rgb_img.shape[2]]

        # predict the labels and their probabilities
        dist_out = np.squeeze(self.model.predict(rgb))                # probabilites of a pixel belonging to each of the classes
        label_out = np.argmax(dist_out, axis=2)     # the class with the highest probability
        dist_out = dist_out.astype(np.float64)
        label_out = label_out.astype(np.uint8)

        # Convert back into initial size - tejaswid: if the initialial resize is not done, then this can be ignored
        dist_out = cv2.resize(dist_out, dsize=(initial_dimensions[1], initial_dimensions[0]), interpolation=cv2.INTER_NEAREST)
        label_out = cv2.resize(label_out, dsize=(initial_dimensions[1], initial_dimensions[0]), interpolation=cv2.INTER_NEAREST)
        now  = rospy.Time.now()

        rospy.loginfo("after segmentation time %i %i", now.secs, now.nsecs )
        
        # Generating colored output
        if self.network_config.enable_colored_labeling:
            colored_label_out = np.zeros(shape=(label_out.shape[0], label_out.shape[1], 3))
            for i in range(len(colored_label_out)):
                for j in range(len(colored_label_out[i])):
                    colored_label_out[i][j] = np.array(label_to_color[label_out[i][j]]) # convert labels into color
            colored_label_out = colored_label_out.astype(np.uint8) # turn into integer
            image = PIL.Image.fromarray(colored_label_out, 'RGB')   # TODO: remove dependence on PIL. Replace it with Opencv Mat for consistency.
            image_msg = Image()
            image_msg.header.stamp = rospy.Time.now()
            image_msg.height = image.height
            image_msg.width = image.width
            image_msg.encoding = 'rgb8'
            image_msg.is_bigendian = False
            image_msg.step = 3 * image.width
            image_msg.data = np.array(image).tobytes()
            self.colored_img_pub.publish(image_msg)

        return label_out, dist_out

    def spin(self):
        rospy.spin()

if __name__ == "__main__":
    node = SegmentationNode()
    node.spin()