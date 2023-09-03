

from picamera.array import PiRGBArray
from picamera import PiCamera
from pygame import mixer
import RPi.GPIO as GPIO
import tensorflow as tf
import numpy as np
import argparse
import time
import cv2
import sys
import os

mixer.init(0)
sound = mixer.Sound('/home/pi/Desktop/NERVE V 2 TEST/Voice/Welcome Object.wav')
sound.play()

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO_SHUTDOWN=4
GPIO.setup(GPIO_SHUTDOWN,GPIO.IN,pull_up_down=GPIO.PUD_UP)

WIDTH = 800
HEIGHT = 464

camera_type = 'picamera'
parser = argparse.ArgumentParser()
args = parser.parse_args()


sys.path.append('..')

from utils import label_map_util
from utils import visualization_utils as vis_util
MODEL_NAME = 'ssdlite_mobilenet_v2_coco_2018_05_09'
CWD_PATH = os.getcwd()
PATH_TO_CKPT = os.path.join(CWD_PATH,MODEL_NAME,'frozen_inference_graph.pb')
PATH_TO_LABELS = os.path.join(CWD_PATH,'data','mscoco_label_map.pbtxt')
NUM_CLASSES = 90
label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=NUM_CLASSES, use_display_name=True)
category_index = label_map_util.create_category_index(categories)
detection_graph = tf.Graph()
with detection_graph.as_default():
    od_graph_def = tf.compat.v1.GraphDef()
    with tf.io.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
        serialized_graph = fid.read()
        od_graph_def.ParseFromString(serialized_graph)
        tf.import_graph_def(od_graph_def, name='')

    sess = tf.compat.v1.Session(graph=detection_graph)


# Define input and output tensors (i.e. data) for the object detection classifier

# Input tensor is the image
image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')

# Output tensors are the detection boxes, scores, and classes
# Each box represents a part of the image where a particular object was detected
detection_boxes = detection_graph.get_tensor_by_name('detection_boxes:0')

# Each score represents level of confidence for each of the objects.
# The score is shown on the result image, together with the class label.
detection_scores = detection_graph.get_tensor_by_name('detection_scores:0')
detection_classes = detection_graph.get_tensor_by_name('detection_classes:0')

# Number of objects detected
num_detections = detection_graph.get_tensor_by_name('num_detections:0')

# Initialize frame rate calculation
frame_rate_calc = 1
freq = cv2.getTickFrequency()
font = cv2.FONT_HERSHEY_SIMPLEX


if camera_type == 'picamera':
    # Initialize Picamera and grab reference to the raw capture
    camera = PiCamera()
    camera.resolution = (WIDTH,HEIGHT)
    camera.framerate = 10
    rawCapture = PiRGBArray(camera, size=(WIDTH,HEIGHT))
    rawCapture.truncate(0)
    time_start=0.00;time_stop=0.00
    for frame1 in camera.capture_continuous(rawCapture, format="bgr",use_video_port=True):

        t1 = cv2.getTickCount()
        frame = np.copy(frame1.array)
        frame.setflags(write=1)
        frame_expanded = np.expand_dims(frame, axis=0)
        (boxes, scores, classes, num) = sess.run(
            [detection_boxes, detection_scores, detection_classes, num_detections],
            feed_dict={image_tensor: frame_expanded})
        vis_util.visualize_boxes_and_labels_on_image_array(
            frame,
            np.squeeze(boxes),
            np.squeeze(classes).astype(np.int32),
            np.squeeze(scores),
            category_index,
            use_normalized_coordinates=True,
            line_thickness=8,
            min_score_thresh=0.40)

        cv2.putText(frame,"FPS: {0:.2f}".format(frame_rate_calc),(30,50),font,1,(12,243,26),2,cv2.LINE_AA)
        cv2.imshow("NERVE Camera Vision", frame)
        t2 = cv2.getTickCount()
        time1 = (t2-t1)/freq
        frame_rate_calc = 1/time1

        # Press 'q' to quit
        if cv2.waitKey(1) == ord('q'):
            break
        if(GPIO.input(GPIO_SHUTDOWN)==1):
            start_time= time.time()
        else:
            print('stop')
            stop_time= time.time()
            if((stop_time-start_time)>5):
                time.sleep(2)
                break

        rawCapture.truncate(0)

    camera.close()
    


cv2.destroyAllWindows()
sound = mixer.Sound('/home/pi/Desktop/NERVE V 2 TEST/Voice/End.wav')
sound.play()
GPIO.cleanup()
print('Program close by Dheeraj Sharma') 
time.sleep(3.5)

