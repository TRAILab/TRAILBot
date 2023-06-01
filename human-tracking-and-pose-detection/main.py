import argparse
import cv2
import math
import numpy as np
import tensorflow as tf
import tensorflow_hub as hub
import time
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
    return parser.parse_args()


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

    keypoints = outputs['output_0'].numpy()

    threshold = 0.3
    count_of_people = np.sum(keypoints[0, :, -1] > threshold)
    print_verbose_only("count_of_people", count_of_people)
    return keypoints[:, :, :51].reshape((6, 17, 3))[0]


def isTherePerson(points, score_threshold=0.3):
    """
    return True/False of whether there is a person
    """
    visible_joints = np.sum(points[:, 2] > score_threshold)
    return visible_joints >= 3


def isPersonFacingCamera(points, score_threshold=0.3):
    """
    return True/False depending on if the person is facing camera or not
    """
    LEFT_EYE = 1
    NOSE = 0
    RIGHT_EYE = 2
    visible_joints_face = np.sum(points[:5, 2] > score_threshold)
    facing_forward = points[LEFT_EYE][1] > points[NOSE][1] > points[RIGHT_EYE][1]
    return visible_joints_face >= 3 and facing_forward


def getHeadingAngle(
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

    angle = getHeadingAngle(keypoints)
    log = ""
    log += f'is there person:{isTherePerson(keypoints)}' + '\n'
    log += f"Person {'is' if isPersonFacingCamera(keypoints) else 'NOT'} facing laptop!\n"
    log += f"heading angle:{angle}" + '\n'

    if abs(angle) < 10:
        log += "CENTER\n"
    elif angle < 0:
        log += "LEFT\n"
    else:
        log += "RIGHT\n"

    print_verbose_only(log)
    return keypoints


def main():
    global parse_args
    parse_args = parse_arguments()
    if parser_args.display_image:
        from helper_for_visualization import draw_prediction_on_image

    model = hub.load(
        "models_downloaded/multipose_tf").signatures['serving_default']

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
