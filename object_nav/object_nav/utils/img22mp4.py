import cv2
import os
from natsort import natsorted

image_folder = '/home/trailbot/Documents/data_process_Kitti/results/ptp_test'
output_video = '/home/trailbot/Documents/data_process_Kitti/results/ptp_test/output.mp4'
fps = 10

# Define range
start_idx = 30
end_idx = 140  # inclusive

# Collect PNG files and filter by range
images = [img for img in os.listdir(image_folder) if img.endswith('.png')]

# Filter by number in filename
filtered_images = []
for img in images:
    try:
        idx = int(img.split('_')[-1].split('.')[0])
        if start_idx <= idx <= end_idx:
            filtered_images.append(img)
    except ValueError:
        continue

filtered_images = natsorted(filtered_images)

print(f"Selected {len(filtered_images)} images between {start_idx} and {end_idx}")

# Read first image to get size
first_frame = cv2.imread(os.path.join(image_folder, filtered_images[0]))
height, width, _ = first_frame.shape
height -= height % 2
width -= width % 2

fourcc = cv2.VideoWriter_fourcc(*'mp4v')
video_writer = cv2.VideoWriter(output_video, fourcc, fps, (width, height))

for image in filtered_images:
    frame = cv2.imread(os.path.join(image_folder, image))
    resized_frame = cv2.resize(frame, (width, height))
    video_writer.write(resized_frame)

video_writer.release()
print(f"✅ Video saved to {output_video}")
