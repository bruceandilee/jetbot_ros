#!/usr/bin/env python
from __future__ import print_function
import os
import sys
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import json
import time
from Adafruit_MotorHAT import Adafruit_MotorHAT
from std_msgs.msg import String
from sensor_msgs.msg import Image
from flask import Flask, request, jsonify, make_response
from threading import Thread

# Bridge Between API and ROS
image_message = None

# Thanks to user hermanoid from the ROS answer forum for the conversion methods
# as cv_bridge does not work with my Ubuntu/ROS Melodic setup
# https://answers.ros.org/question/350904/cv_bridge-throws-boost-import-error-in-python-3-and-ros-melodic/ 
def imgmsg_to_cv2(img_msg):
    if img_msg.encoding != "bgr8":
        print("This Coral detect node has been hardcoded to the 'bgr8' encoding.  Come change the code if you're actually trying to implement a new camera")
    dtype = np.dtype("uint8") # Hardcode to 8 bits...
    dtype = dtype.newbyteorder('>' if img_msg.is_bigendian else '<')
    image_opencv = np.ndarray(shape=(img_msg.height, img_msg.width, 3), # and three channels of data. Since OpenCV works with bgr natively, we don't need to reorder the channels.
                    dtype=dtype, buffer=img_msg.data)
    # If the byt order is different between the message and the system.
    if img_msg.is_bigendian == (sys.byteorder == 'little'):
        image_opencv = image_opencv.byteswap().newbyteorder()
    return image_opencv

def cv2_to_imgmsg(cv_image):
    img_msg = Image()
    img_msg.height = cv_image.shape[0]
    img_msg.width = cv_image.shape[1]
    img_msg.encoding = "bgr8"
    img_msg.is_bigendian = 0
    img_msg.data = cv_image.tostring()
    img_msg.step = len(img_msg.data) // img_msg.height # That double line is actually integer division, not a comment
    return img_msg

# Flask API
app = Flask(__name__)

@app.route('/')
def index():
  return '<<h1>jetbot REST interaction</h1><h2>motors (GET)</h2><h3>sets power of motors</h3>left(standard: 0): float between 0 and 1<br>right (standard: 0): float between 0 and 1<br><h2>/camera (GET)</h2><h3>gets opencv-encoded image as string</h3><h2>/cameraandmotors (GET)</h2><h3>sets motor values and gets opencv-encoded image as string</h3>left(standard: 0): float between 0 and 1<br>right (standard: 0): float between 0 and 1<br>'

def set_motors(left_param, right_param):
  try:
    global motor_pub
    left = 0.0
    if not left_param is None:
      if abs(left_param) > 1.0:
        left = 1.0
      else:
        # Uncomment and comment correct line if jetbot is driving backwards
        left = -1 * abs(left_param)
        # left = abs(left_param)
    right = 0.0
    if not right_param is None:
      if abs(right_param) > 1.0:
        right = 1.0
      else:
        # Uncomment and comment correct line if jetbot is driving backwards
        right = -1 * abs(right_param)
        # right = abs(right_param)
    msg = String()
    msg.data = '{0},{1}'.format(left, right)
    motor_pub.publish(msg)
    return jsonify({'left': left,
                      'right': right})
  except Exception as e:
    return jsonify({'error': str(e)})

@app.route('/motors')
def get_motor_input():
  try:
    left_param = request.args.get('left', default=None, type=float)
    right_param = request.args.get('right', default=None, type=float)
    set_motors(left_param, right_param)
    return set_motors(left_param, right_param)
  except Exception as e:
    return jsonify({'error': str(e)})

def get_image():
  try:
    global image_message
    cv_image = imgmsg_to_cv2(image_message)
    retval, buf = cv2.imencode('.png', cv_image)
    response = make_response(buf.tobytes())
    response.headers['Content-Type'] = 'image/png'
    return response
  except Exception as e:
    return jsonify({'error': str(e)})

@app.route('/camera')
def return_image():
  try:
    return get_image()
  except Exception as e:
    return jsonify({'error': str(e)})

def run():
  app.run(host='0.0.0.0', port=8080)

def start_api():
  t = Thread(target=run)
  t.start()

# Motor publisher
motor_pub = None

class ApiHandlerNode(Node):
   def __init__(self):
        super().__init__('api_handler', namespace='jetbot')
        self.image_subscriber = self.create_subscription(Image, '/jetbot/camera/image_raw', self.image_listener, 10)
   
   def image_listener(self, image_msg):
    try:
      global image_message
      image_message = image_msg
    except Exception as e:
      print(e)

def main(args=None):
  start_api()
  try:
    rclpy.init(args=args)
    node = ApiHandlerNode()
    global motor_pub
    motor_pub = node.create_publisher(String, 'cmd_string', 10)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
