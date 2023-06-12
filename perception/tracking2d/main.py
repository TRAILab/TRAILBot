import argparse
import cv2
import math
import numpy as np
import os
import tarfile
import tensorflow as tf
import tensorflow_hub as hub
import time
import urllib.request

MODEL_URL = "https://tfhub.dev/google/movenet/multipose/lightning/1?tf-hub-format=compressed"
SAVED_MODEL_PATH = "./multipose_model"
parser_args = tuple()

def parse_arguments():
    """
    handle command line arguments
    """
    parser = argparse.ArgumentParser(description='Example command-line parser')
    parser.add_argument(
        '-v',
        '--verbose',
        action='store_true',
        help='Enable print_verbose_only outputs')
    parser.add_argument(
        '-k',
        '--display_keypoint',
        action='store_true',
        help='Enable display of keypoints')
    parser.add_argument(
        '-i',
        '--display_image',
        action='store_true',
        help='Enable display of image and keypoints')
    parser.add_argument(
        '-d',
        '--download_model', 
        action='store_true', 
        help='Flag to download the model. This must be ran at least once')
    return parser.parse_args()

def download_model():
    # Create the directory if it doesn't exist
    os.makedirs(os.path.dirname(SAVED_MODEL_PATH), exist_ok=True)

    # Download the compressed model from the URL
    model_path, _ = urllib.request.urlretrieve(MODEL_URL)

    # Extract the compressed model to the specified path
    with tarfile.open(model_path, "r:gz") as tar:
        tar.extractall(SAVED_MODEL_PATH)
    
    model = hub.load(SAVED_MODEL_PATH).signatures['serving_default']
    return model

def load_saved_model():
    """
    load the saved model from SAVED_MODEL_PATH
    """
    if not os.path.exists(SAVED_MODEL_PATH):
        raise FileNotFoundError(f"Model not found at {SAVED_MODEL_PATH}")
    model = hub.load(SAVED_MODEL_PATH).signatures['serving_default']
    return model

def print_verbose_only(*args, **kwargs):
    """
    print only if verbose==True
    """
    if parser_args.verbose:
        print(*args, **kwargs)


def print_fps(func):
    """
    function decorator for printing fps
    """
    def wrapper(*args, **kwargs):
        prev_time = time.time()
        result = func(*args, **kwargs)
        cur_time = time.time()
        if cur_time - prev_time > (0.000001):
            fps = 1.0 / (cur_time - prev_time)
            print_verbose_only('FPS: {:.2f}'.format(fps))
        return result
    return wrapper


def movenet(input_image, model):
    """
    movenet model:
    Gets input image and outputs array of keypoints with certainty score
    downloaded from #https://tfhub.dev/google/movenet/multipose/lightning/1
    """
    # SavedModel format expects tensor type of int32.
    input_image = tf.cast(input_image, dtype=tf.int32)
    outputs = model(input_image)  # Output is a [1, 6, 56] tensor.
    """
    The first 17 * 3 elements are the keypoint locations and scores in the format: [y_0, x_0, s_0, y_1, x_1, s_1, …, y_16, x_16, s_16], where y_i, x_i, s_i are the yx-coordinates (normalized to image frame, e.g. range in [0.0, 1.0]) and confidence scores of the i-th joint correspondingly. The order of the 17 keypoint joints is: [nose, left eye, right eye, left ear, right ear, left shoulder, right shoulder, left elbow, right elbow, left wrist, right wrist, left hip, right hip, left knee, right knee, left ankle, right ankle]. The remaining 5 elements [ymin, xmin, ymax, xmax, score] represent the region of the bounding box (in normalized coordinates) and the confidence score of the instance
    """
    keypoints = outputs['output_0'].numpy()

    threshold = 0.3
    count_of_people = np.sum(keypoints[0, :, -1] > threshold)
    print_verbose_only("count_of_people", count_of_people)
    
    #there are 6 people
    #there are 17 body points and therefore 3*17=51 numbers per person
    return keypoints[:, :, :51].reshape((6, 17, 3))[0]


def is_there_person(points, score_threshold=0.3):
    """
    return True/False of whether there is a person
    """
    visible_joints = np.sum(points[:, 2] > score_threshold)
    return visible_joints >= 3


def is_person_facing_camera(points, score_threshold=0.3):
    """
    return True/False depending on if the person is facing camera or not
    """
    LEFT_EYE = 1
    NOSE = 0
    RIGHT_EYE = 2
    visible_joints_face = np.sum(points[:5, 2] > score_threshold)
    facing_forward = points[LEFT_EYE][1] > points[NOSE][1] > points[RIGHT_EYE][1]
    return visible_joints_face >= 3 and facing_forward


def get_heading_angle(
        points,
        score_threshold=0.3,
        fov=90,
        image_width=1,
        offset=0,
        scaling=1):
    """
    get the heading angle of the person, in degree, relative to the center
    of the field of view
    """
    visible_points = points[points[:, 2] > score_threshold]
    x_mean = np.mean(visible_points[:, 1])
    x_angle_radian = math.atan(
        (x_mean - (image_width / 2)) / (image_width / 2) * math.tan(math.radians(fov / 2)))
    return offset + scaling * math.degrees(x_angle_radian)


def get_x_y_coord(points,score_threshold=0.3,image_width=1280, image_height=1024): 
    visible_points = points[ points[:,2] > score_threshold]
    x_mean = np.mean(visible_points[:, 1])
    y_mean = np.mean(visible_points[:, 0])
    return x_mean*image_width, y_mean*image_height

def keypoints_to_img(
        points,
        nth_point_to_highlight=None,
        score_threshold=0.2,
        img_size=128):
    """
    draw keypoints on a black image
    """
    img = np.zeros((img_size, img_size, 3), dtype=np.uint8)

    for i, (y, x, certainty_score) in enumerate(points):
        if certainty_score < score_threshold:
            continue

        if i == nth_point_to_highlight:
            color = (0, int(255 * certainty_score), 0)
        else:
            color = (0, 0, int(255 * certainty_score))

        coordinate = int((1 - x) * img_size), int(y * img_size)
        cv2.circle(img, coordinate, 1, color, -1)
    return img


@print_fps
def process_frame(image, model):
    """
    process a frame. Determine keypoints and number of people and
    heading angle.
    """
    input_size = 256
    input_image = tf.expand_dims(image, axis=0)
    input_image = tf.image.resize_with_pad(input_image, input_size, input_size)

    # Run model inference
    keypoints = movenet(input_image, model)

    angle = get_heading_angle(keypoints)
    log = ""
    log += f'is there person:{is_there_person(keypoints)}' + '\n'
    log += f"Person {'is' if is_person_facing_camera(keypoints) else 'NOT'} facing laptop!\n"
    log += f"heading angle:{angle}\n"
    if abs(angle) < 10:
        log += "CENTER\n"
    elif angle < 0:
        log += "LEFT\n"
    else:
        log += "RIGHT\n"
    print(get_x_y_coord(keypoints))
    print_verbose_only(log)
    return keypoints


def main():
    global parser_args
    parser_args = parse_arguments()
    if parser_args.display_image:
        from helper_for_visualization import draw_prediction_on_image

    if parser_args.download_model:
        model = download_model()
    else:
        model = load_saved_model()


    cap = cv2.VideoCapture(0)  # Open the webcam stream.

    if parser_args.display_image or parser_args.display_keypoint:
        # Define the output window.
        cv2.namedWindow('MoveNet', cv2.WINDOW_NORMAL)

    # Process each frame of the webcam stream.
    while True:
        ret, image = cap.read()  # Read a frame from the webcam stream.
        if not ret:
            break  # If reading the frame fails, break out of the loop.

        keypoints = process_frame(image, model)

        # Visualize the predictions keypoints only
        if parser_args.display_keypoint:
            output_frame = keypoints_to_img(
                keypoints, nth_point_to_highlight=1)
            # Show the output frame in the output window.
            cv2.imshow('MoveNet', output_frame)

        # Visualize the predictions with image
        if parser_args.display_image:
            display_image = tf.expand_dims(image, axis=0)
            display_image = tf.cast(
                tf.image.resize_with_pad(
                    display_image, 1280, 1280), dtype=tf.int32)
            output_overlay = draw_prediction_on_image(
                np.squeeze(
                    display_image.numpy(), axis=0), keypoints.reshape(
                    (1, 1, 17, 3)))
            cv2.imshow("MoveNet", output_overlay)

        # Exit the loop if the user presses the 'q' key.
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cap.release()
            cv2.destroyAllWindows()
            break


if __name__ == "__main__":
    main()
