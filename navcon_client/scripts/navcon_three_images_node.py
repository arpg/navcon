#!/usr/bin/env python

import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String
from navcon_client.srv import NavConCall, NavConCallResponse
from navcon_client.msg import NavClient3
import json
from geometry_msgs.msg import PointStamped
import numpy as np
import copy

import requests

class NavConRequest:
    def __init__(self):
        self.image_front = None
        self.image_left = None
        self.image_right = None


        # Set up the ROS subscribers
        cam_front_topic = rospy.get_param('~cam_front','image')
        cam_left_topic = rospy.get_param('~cam_left','image')
        cam_right_topic = rospy.get_param('~cam_right','image')

        self.image_front_subscriber = rospy.Subscriber(cam_front_topic, CompressedImage, self.image_front_callback)
        self.image_left_subscriber = rospy.Subscriber(cam_left_topic, CompressedImage, self.image_left_callback)
        self.image_right_subscriber = rospy.Subscriber(cam_right_topic, CompressedImage, self.image_right_callback)

        # Set up the ROS service server
        self.service_server = rospy.Service('nav_server', NavConCall, self.handle_navcon)

        # Set up the ROS publishers
        self.center_point_pub = rospy.Publisher('center_point', PointStamped, latch=True, queue_size=10)
        self.bounding_box_pub = rospy.Publisher('lang_box_image/image/compressed', CompressedImage, latch=True, queue_size=10 )
        self.concate_image_pub = rospy.Publisher('lang_concate_image/image/compressed', CompressedImage, latch=True, queue_size=10 )
        self.full_message_pub = rospy.Publisher('lang/full', NavClient3, latch=True, queue_size=10 )


        self.bridge = CvBridge()

        self.url = rospy.get_param('~url','')


    def image_front_callback(self, msg):
        if self.image_front is None:
            self.image_front_frame = msg.header.frame_id
        self.image_front = msg

    def image_left_callback(self, msg):
        if self.image_left is None:
            self.image_left_frame = msg.header.frame_id
        self.image_left = msg

    def image_right_callback(self, msg):
        if self.image_right is None:
            self.image_right_frame = msg.header.frame_id
        self.image_right = msg

    def create_request_data(self):
        self.cv_image_front = self.bridge.compressed_imgmsg_to_cv2(self.image_front, desired_encoding='rgb8')
        self.cv_image_left = self.bridge.compressed_imgmsg_to_cv2(self.image_left, desired_encoding='rgb8')
        self.cv_image_right = self.bridge.compressed_imgmsg_to_cv2(self.image_right, desired_encoding='rgb8')

        #cv_single_image = self.create_single_image(self.cv_image_left, self.cv_image_front, self.cv_image_right)
        
        #Encode images
        en_image_front = cv2.imencode('.jpg', self.cv_image_front)[1].tobytes()
        en_image_left = cv2.imencode('.jpg', self.cv_image_left)[1].tobytes()
        en_image_right = cv2.imencode('.jpg', self.cv_image_right)[1].tobytes()

        #en_single_imaage = cv2.imencode('.jpg', cv_single_image)[1].tobytes()

        # files = {'image_front': ('image_front.jpg', en_image_front, 'image/jpeg', self.image_front.header.frame_id),\
        #  'image_left': ('image_left.jpg', en_image_left, 'image/jpeg', self.image_left.header.frame_id),\
        #  'image_right': ('image_right.jpg', en_image_right, 'image/jpeg', self.image_right.header.frame_id)}

        files = {'image_front': ('image_front.jpg', en_image_front, 'image/jpeg'),\
         'image_left': ('image_left.jpg', en_image_left, 'image/jpeg'),\
         'image_right': ('image_right.jpg', en_image_right, 'image/jpeg')}


        #files = {'image': ('single_image.jpg', en_single_imaage, 'image/jpeg')}
        
        return files

    def create_single_image(self, left, front, right):
        rospy.loginfo('Making single image')
        threshold = 15
        left = copy.deepcopy(left)
        right = copy.deepcopy(right)
        front = copy.deepcopy(front)
        left_image = self.crop_image(left, threshold)
        front_image = self.crop_image(front, threshold)
        right_image = self.crop_image(right, threshold)
        print(left_image.shape, front_image.shape, right_image.shape)
        single_image = np.concatenate((left_image, front_image, right_image), axis=1)
        self.publish_single_image(single_image)
        return single_image
    
    def publish_single_image(self, image):
        rospy.loginfo("Publishing single image")
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        compressed_image = self.bridge.cv2_to_compressed_imgmsg(image, dst_format='jpg')
        compressed_image.header.stamp = self.image_front.header.stamp
        self.concate_image_pub.publish(compressed_image)
        self.concat_image = compressed_image
        rospy.loginfo("Single Image Published")
        

    def crop_image(self, image, threshold):
        image[:, :threshold] = 0  # Left
        image[:, -threshold:] = 0  # Right
        return image

    # Gets the pixel value from combined image to single image
    def original_pixel(self, x):
        left_width = self.cv_image_left.shape[1]
        front_width = self.cv_image_front.shape[1]
        right_width = self.cv_image_right.shape[1]
        if x < left_width:
            return (x, 'left')   # Coordinate is in the left image
        elif x < left_width + front_width:
            return (x - left_width, 'front')  # Coordinate is in the front image
        else:
            return (x - left_width - front_width, 'right')  # Coordinate is in the right image
        


    def handle_navcon(self, req):
        rospy.loginfo("Recieved request")
        # Convert the most recent image to OpenCV format
        if self.image_front is not None and self.image_left is not None and self.image_right is not None:
            self.current_request = req
            # Send Image NavCon Server
            headers = {'Content-Type': 'image/jpeg'}
            files = self.create_request_data()
            data = {'sentence': str(req.sentence)}
            navcon_response = requests.post(self.url, files=files, data=data)
            self.parse_result(navcon_response.text)
            # Return the result
            return NavConCallResponse(success=True, x=1, y=1)
        else:
            return NavConCallResponse(success=False, x=-1, y=-1)

    def parse_result(self, response):
        json_response = None
        try:
            json_response = json.loads(response)
        except ValueError as e:
            rospy.logwarn('Invalid json response')
            print(e)
            return False
        if json_response['function'] == 'navigate_to_object':
            x = int(json_response['inputs'][0])
            y = int(json_response['inputs'][1])
            #x, frame = self.original_pixel(x)
            frame = str(json_response['frame'])
            frame_id = self.get_frame_id(frame)
            print("X", x, "Y", y)
            print("Frame ID", frame_id)
            point_to_pub = PointStamped()
            point_to_pub.point.x = x
            # Camera size offset
            y = 608 -y
            point_to_pub.point.y = y
            point_to_pub.header.frame_id = frame_id
            bounds = json_response['box']
            print(bounds)
            left, _  = self.original_pixel(bounds[0])
            lower = 608 - bounds[1]
            right, _ = self.original_pixel(bounds[2])
            upper = 608 - bounds[3]
            print(left, lower, right, upper)
            self.publish_box_image(left, lower, right, upper, frame)
            self.center_point_pub.publish(point_to_pub)
            self.publish_full_message(x,y,left, lower, right, upper)
            return True

     
        return False

    def get_frame_id(self, frame):
        if "right" in frame:
            return self.image_right_frame
        elif "left" in frame:
            return self.image_left_frame
        elif "front" in frame:
            return self.image_front_frame
        else:
            rospy.logwarn("No frame found")
            return "None"
        
    def publish_box_image(self, left, lower, right, upper, frame):
        image = None
        if "right" in frame:
            image = copy.deepcopy(self.cv_image_right)
            stamp = self.image_right.header.stamp
        elif "left" in frame:
            image = copy.deepcopy(self.cv_image_left)
            stamp = self.image_left.header.stamp
        elif "front" in frame:
            image = copy.deepcopy(self.cv_image_front)
            stamp = self.image_front.header.stamp
        else:
            rospy.logwarn("No frame found")
            return None
        cv2.rectangle(
            image,
            (int(left), int(upper)),
            (int(right), int(lower)),
            (255, 0, 0),
            4)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        compressed_image = self.bridge.cv2_to_compressed_imgmsg(image, dst_format='jpg')
        compressed_image.header.stamp = stamp
        self.bounding_box_pub.publish(compressed_image)
        self.bounding_box_image = compressed_image
        
    def publish_full_message(self, x, y, left, lower, right, upper):
        msg = NavClient3()
        msg.header.stamp = rospy.Time.now()
        msg.sentence = str(self.current_request.sentence)
        msg.box_image = self.bounding_box_image
        msg.left_image = self.image_left
        msg.front_image = self.image_front
        msg.right_image = self.image_right
        msg.x = x
        msg.y = y
        msg.left = left
        msg.lower = lower
        msg.right = right
        msg.upper = upper
        self.full_message_pub.publish(msg)    
    


        
if __name__ == '__main__':
    # Initialize the ROS node
    rospy.init_node('nav_client')

    # Create an instance of the ImageSubscriber class
    nav_client = NavConRequest()

    # Spin the ROS node
    rospy.spin()
