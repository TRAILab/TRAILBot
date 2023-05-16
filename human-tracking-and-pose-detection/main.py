downloadModel=True
runTestImg=False
import tensorflow as tf
import tensorflow_hub as hub
from tensorflow_docs.vis import embed
import numpy as np
import cv2
# Some modules to display an animation using imageio.
import imageio
from IPython.display import HTML, display

import my_helpers 
from helper_for_visualization import *
import pickle, subprocess,time, os,sys, math
print("downloading...")

model_name = "movenet_lightning_f16" + "tflite"

if "tflite" in model_name:
  if "movenet_lightning_f16" in model_name:
    cmd = "wget -q -O model.tflite https://tfhub.dev/google/lite-model/movenet/singlepose/lightning/tflite/float16/4?lite-format=tflite"
    input_size = 192
  elif "movenet_thunder_f16" in model_name:
    cmd = "wget -q -O model.tflite https://tfhub.dev/google/lite-model/movenet/singlepose/thunder/tflite/float16/4?lite-format=tflite"
    input_size = 256
  elif "movenet_lightning_int8" in model_name:
    cmd = "wget -q -O model.tflite https://tfhub.dev/google/lite-model/movenet/singlepose/lightning/tflite/int8/4?lite-format=tflite"
    input_size = 192
  elif "movenet_thunder_int8" in model_name:
    cmd = "wget -q -O model.tflite https://tfhub.dev/google/lite-model/movenet/singlepose/thunder/tflite/int8/4?lite-format=tflite"
    input_size = 256
  else:
    raise ValueError("Unsupported model name: %s" % model_name)
  if downloadModel:
    print(subprocess.run(cmd, shell=True, stdout=subprocess.PIPE).stdout.decode())
  # Initialize the TFLite interpreter
  interpreter = tf.lite.Interpreter(model_path="model.tflite")
  interpreter.allocate_tensors()

  def movenet(input_image):
    """Runs detection on an input image.

    Args:
      input_image: A [1, height, width, 3] tensor represents the input image
        pixels. Note that the height/width should already be resized and match the
        expected input resolution of the model before passing into this function.

    Returns:
      A [1, 1, 17, 3] float numpy array representing the predicted keypoint
      coordinates and scores.
    """
    # TF Lite format expects tensor type of uint8.
    input_image = tf.cast(input_image, dtype=tf.uint8)
    input_details = interpreter.get_input_details()
    output_details = interpreter.get_output_details()
    interpreter.set_tensor(input_details[0]['index'], input_image.numpy())
    # Invoke inference.
    interpreter.invoke()
    # Get the model prediction.
    keypoints_with_scores = interpreter.get_tensor(output_details[0]['index'])
    return keypoints_with_scores

else:
  if "movenet_lightning" in model_name:
    if downloadModel:  
      module = hub.load("https://tfhub.dev/google/movenet/singlepose/lightning/4")
    input_size = 192
  elif "movenet_thunder" in model_name:
    module = hub.load("https://tfhub.dev/google/movenet/singlepose/thunder/4")
    input_size = 256
  elif "movenet_multipose" in model_name:
    module = hub.load("https://tfhub.dev/google/movenet/multipose/lightning/1")
    input_size = 256
  else:
    raise ValueError("Unsupported model name: %s" % model_name)

  def movenet(input_image):
    """Runs detection on an input image.

    Args:
      input_image: A [1, height, width, 3] tensor represents the input image
        pixels. Note that the height/width should already be resized and match the
        expected input resolution of the model before passing into this function.

    Returns:
      A [1, 1, 17, 3] float numpy array representing the predicted keypoint
      coordinates and scores.
    """
    model = module.signatures['serving_default']


    # SavedModel format expects tensor type of int32.
    input_image = tf.cast(input_image, dtype=tf.int32)
    # Run model inference.
    outputs = model(input_image)
    # Output is a [1, 1, 17, 3] tensor.
    keypoints_with_scores = outputs['output_0'].numpy()
    return keypoints_with_scores


def isTherePerson(keypoints_with_scores,score_threshold=0.3):
  points = keypoints_with_scores[0][0]
  visible_joints = np.sum(points[:,2]>score_threshold)
  return visible_joints>=3

def isPersonFacingCamera(keypoints_with_scores,score_threshold=0.3):
  LEFT_EYE=1
  NOSE = 0
  RIGHT_EYE=2
  points = keypoints_with_scores[0][0]
  visible_joints_face = np.sum(points[:5,2]>score_threshold)
  return visible_joints_face>=3 and points[LEFT_EYE][1]>points[NOSE][1]>points[RIGHT_EYE][1]

def get_angles(x, y=0, fov=90, image_width = 1,image_height = 1):
    # calculate the angle relative to the camera's center view
    x_angle = math.degrees(math.atan((x - (image_width / 2)) / (image_width / 2) * math.tan(math.radians(fov / 2))))
    # y_angle = math.degrees(math.atan(((image_height / 2) - y) / (image_height / 2) * math.tan(math.radians(fov / 2))))
    return x_angle #, y_angle

def getHeadingAngle(keypoints_with_scores,score_threshold=0.3):
  offset=0
  scaling=1 
  points=keypoints_with_scores[0][0]
  visible_points = points[points[:,2] > score_threshold]
  # sum up the x coordinate of the selected points
  x_mean = np.mean(visible_points[:, 1])
  return offset + scaling *get_angles(x_mean, None)

def keypoints_with_scores_to_img(keypoints_with_scores, nth_point_to_highlight=-1,score_threshold=0.2):
  img_size = 128
  points = keypoints_with_scores[0][0]
  img = np.zeros((img_size, img_size, 3), dtype=np.uint8)
  # scaled_points = (points * img_size).astype(np.int32)
  for i, (y,x, certainty_score) in enumerate(points):
    # print(x,y,certainty_score)
    if certainty_score < score_threshold:
      continue
    color = (0, 0, int(255*certainty_score))
    if i==nth_point_to_highlight:
      color = (0, 255, 0)
    coordinate = int((1-x)*img_size) ,int(y*img_size)
    cv2.circle(img, coordinate, 1, color, -1)
  return img

# import winsound
@my_helpers.fps_timer
def process_frame(image):
  input_image = tf.expand_dims(image, axis=0)
  input_image = tf.image.resize_with_pad(input_image, input_size, input_size)

  # Run model inference.
  keypoints_with_scores = movenet(input_image)
  nth_point_to_highlight=1# int(input("num:"))
  log = ""
  log += f'is there person:{isTherePerson(keypoints_with_scores)}'+'\n'
  if isPersonFacingCamera(keypoints_with_scores):
    log+=("Person is facing laptop!")+'\n'
    # winsound.Beep(2500, 100)  
  else:
    log+=("Person NOT facing")+'\n'
  angle =getHeadingAngle(keypoints_with_scores)
  log+=f"heading angle:{angle}"+'\n'
  if abs(angle)<10:
    log+=("CENTER")+'\n'
  elif angle<0:
    log+=("LEFT")+'\n'
  else:
    log+=("RIGHT")+'\n'
  print(log)
  return keypoints_with_scores_to_img(keypoints_with_scores,nth_point_to_highlight)

# Open the webcam stream.
cap = cv2.VideoCapture(0)

# Define the output window.
cv2.namedWindow('MoveNet', cv2.WINDOW_NORMAL)

# Process each frame of the webcam stream.
while True:
  start_time = time.time()
  ret, frame = cap.read()# Read a frame from the webcam stream.
  if not ret: break # If reading the frame fails, break out of the loop.

  output_frame = process_frame(frame)  # Process the frame using the MoveNet algorithm.
  cv2.imshow('MoveNet', output_frame)   # Show the output frame in the output window.
  # Exit the loop if the user presses the 'q' key.
  if cv2.waitKey(1) & 0xFF == ord('q'):
    cap.release()
    cv2.destroyAllWindows()
    break






# def load_image(image_path):
#   image = tf.io.read_file(image_path)
#   image = tf.image.decode_jpeg(image)
#   return image

# def run_inference(image):
#   # Resize and pad the image to keep the aspect ratio and fit the expected size.
#   input_image = tf.expand_dims(image, axis=0)
#   input_image = tf.image.resize_with_pad(input_image, input_size, input_size)

#   # Run model inference.
#   keypoints_with_scores = movenet(input_image)
#   # Visualize the predictions with image.
#   display_image = tf.expand_dims(image, axis=0)
#   display_image = tf.cast(tf.image.resize_with_pad(
#       display_image, 1280, 1280), dtype=tf.int32)
#   output_overlay = draw_prediction_on_image(
#       np.squeeze(display_image.numpy(), axis=0), keypoints_with_scores)

#   plt.figure(figsize=(5, 5))
#   plt.imshow(output_overlay)
#   _ = plt.axis('off')
#   plt.show()
#   return output_overlay

# # coords=(run_inference(load_image("sample_images/1person_fullbody.jpg")))
# if runTestImg:
#   image_dir = "sample_images/"
#   import os
#   for filename in os.listdir(image_dir):
#     print(image_dir+filename)
#     img = load_image(image_dir+filename)
#     run_inference(img)
