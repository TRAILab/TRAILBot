import os
import sys
import glob
import time
from pathlib import Path
import pickle
import argparse
sys.path.append(sys.path[0] + '/..')
sys.path.append("/home/trailbot/RAG/remembr/")
# This forces Python to load utils from your intended folder
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))

import hydra
from PIL import Image
from omegaconf import DictConfig

from agentic_rag.star.captioners.nvila_captioner import NVILACaptioner
from agentic_rag.star.captioners.captioner import CaptionManager
# 2. Visible objects and infrastructure: Carefully describe notable objects like boxes, signs, poles, vehicles, cones, bollards, elevators, etc. Include their features like color, material, size, whether it is wrapped by something, and approximate position relative to you (e.g., in front, behind, to the left/right).\n\

SEQ: str = "103"
DEFAULT_QUERY: str = "You are a mobile robot navigating a warehouse. From your egocentric point of view, describe the current scene and your own behavior in as much detail as possible. Think step by step. \n\
    Focus on:\n\
        1. Your motion and activity: Are you moving forward, turning, stopped, or interacting with something? \n\
        2. Because the environment may contain multiple similar objects, please disambiguate the target using distinctive attributes—for example, “a large box wrapped in plastic film” versus “a large box without wrapping.” Also note any text labels, approximate size, material, and color, and flag any anomalies or dynamic events.\n\
        3. Environmental features: Describe the physical space. Are you indoors or outdoors? What is the surface type (brick, grass, tile, concrete)? Is the area narrow or spacious, crowded or empty?\n\
        4. Events or activities: Mention any dynamic actions or interactions - What is the forklift currently carrying etc.\n\
        5. Pay attention to the background of the main objects, and specify the location in your response:\n\
            - If the main objects are placed on a shelf or near the shelf, then they are in shelf area. \n\
            - If the main objects are on a shelf, please try to specify which shelf level (e.g., first, second, or third tier).\n\
            - If the main objects are not on a shelf but close to a shelf, then they are in the shelf vicinity.\n\
            - If the main objects are not on a shelf and far away from any shelf, then they are in the staging area.\n\
        Be very specific and descriptive. Imagine someone blind is listening and needs to visualize the entire moment. Focus on relevant and grounded details that could be remembered and queried later."


# DEFAULT_QUERY: str = "You are a mobile robot navigating a warehouse. From your egocentric point of view, describe the current scene and your own behavior in as much detail as possible. Think step by step. \n\
#     Focus on:\n\
#         1. Your motion and activity: Are you moving forward, turning, stopped, or interacting with something? \n\
#         2. Because the environment may contain multiple similar objects, please disambiguate the target using distinctive attributes—for example, “a large box wrapped in plastic film” versus “a large box without wrapping.” Also note any text labels, approximate size, material, and color, and flag any anomalies or dynamic events.\n\
#         3. Environmental features: Describe the physical space. Are you indoors or outdoors? What is the surface type (brick, grass, tile, concrete)? Is the area narrow or spacious, crowded or empty?\n\
#         4. Events or activities: Mention any dynamic actions or interactions - What is the forklift currently carrying etc.\n\
#         5. If possible, include the object’s location context—e.g., whether it’s on a shelf or in a staging/temporary storage area. If it’s on a shelf, try to specify which shelf level (e.g., first, second, or third tier).\n\
#         Be very specific and descriptive. Imagine someone blind is listening and needs to visualize the entire moment. Focus on relevant and grounded details that could be remembered and queried later."

def batch_array(arr, batch_size=3):
    print("Batching array of size:", len(arr))
    basedir = arr[0].split('/')[:-1]
    print("Basedir:", basedir)
    basedir = '/'.join(basedir)
    for i, path in enumerate(arr):
        pkl_dir = path.split('/')[-1][:-4]
        arr[i] = float(pkl_dir)

    i, j, batches = 0, 2, [] # slow pointer, fast pointer
    while j < len(arr):
        batch = arr[i:j+1]
        while batch[-1] - batch[0] > 3.0 and j > i: # 3 seconds
            j -= 1
            batch = arr[i:j+1]
        batches.append(batch)
        i = j + 1
        j = i + 2

    for i, batch in enumerate(batches):
        assert batch[-1] - batch[0] <= 3.0, "Batch size too large!" # each batch should be within 3 seconds
        for j, t in enumerate(batch):
            batch[j] = f"{basedir}/{t}.pkl"
            assert os.path.exists(batch[j]), f"File {batch[j]} does not exist!" # check if file exists
        batches[i] = batch

    return batches

def downsample_by_fps(timestamps, fps=10):
    result = []
    if not timestamps:
        return result

    next_time = timestamps[0]
    for t in timestamps:
        if t >= next_time:
            result.append(t)
            next_time = t + 1.0 / fps
    return result

def sample_by_fps(arr, fps=10):
    if len(arr['timestamps']) != len(arr['images']):
        # print("Timestamps and images length mismatch!")
        return arr['images']

    timestamps = arr['timestamps']
    images = arr['images']

    downsampled_images, downsampled_timestamps = [], []
    next_time = timestamps[0]
    for i, t in enumerate(timestamps):
        if t >= next_time:
            downsampled_images.append(images[i])
            downsampled_timestamps.append(t)
            next_time = t + 1.0 / fps

    return downsampled_images

def process_cfg(cfg: DictConfig, seq: str):
    '''
    配置文件预处理
    '''
    cfg.basedir = Path(cfg.basedir)
    cfg.save_vis_path = Path(cfg.save_vis_path)
    cfg.save_cap_path = Path(cfg.save_cap_path)
    cfg.start_timestamp = time.time()
    time_struct = time.localtime(cfg.start_timestamp)
    cfg.sequence = seq
    cfg.output_dir = f'{cfg.save_cap_path}'

    return cfg

@hydra.main(version_base=None, config_path="../configs", config_name="config")
def main(cfg : DictConfig):
    parser = argparse.ArgumentParser()
    parser.add_argument("--seq_id", type=str, default=SEQ)
    parser.add_argument("--out_path", type=str, default=f"./data/captions/{4}/captions")
    parser.add_argument("--data_path", type=str, default="./data/semantickitti_data") # coda_data
    parser.add_argument("--seconds_per_caption", type=int, default=3)

    parser.add_argument("--num-video-frames", type=int, default=15)
    parser.add_argument("--captioner_name", type=str, default="NVILA-Lite-2B") # NVILA-Lite-2B # NVILA-8B

    parser.add_argument("--model-path", type=str, default="Efficient-Large-Model/NVILA-Lite-2B") # Efficient-Large-Model/NVILA-8B
    parser.add_argument("--conv-mode", "-c", type=str, default="auto")
    parser.add_argument("--query", type=str, default=DEFAULT_QUERY)
    parser.add_argument("--text", type=str)
    # parser.add_argument("--media", type=str, nargs="+", default= [f"{base_path}{img}{200}.png", f"{base_path}{img}{400}.png", f"{base_path}{img}{800}.png"])#["/home/mfyuan/local_folder/OpenNav_v2/deps1/VILA/demo_images/cam0.png"])
    parser.add_argument("--media", type=str, default=["/home/mfyuan/local_folder/OpenNav_v2/deps1/VILA/demo_images/output.mp4"])#/media/ssd/Local_data/CODa_dataset/videos/0/output.mp4
    parser.add_argument("--json-mode", action="store_true")
    parser.add_argument("--json-schema", type=str, default=None)
    args = parser.parse_args()

    cfg = process_cfg(cfg['scenegraph'], seq=cfg.sequence)

    captioner: NVILACaptioner = NVILACaptioner(args)
    caption_manager = CaptionManager(args=cfg, captioner=captioner)

    pkl_files = glob.glob(f"{cfg.save_video_path}/*.pkl")
    pkl_files = sorted(pkl_files)
    if len(pkl_files) == 0:
        print("No file for inference, exitting...")
        return

    batches = batch_array(pkl_files, batch_size=3)

    pointer = 0
    for i, batch in enumerate(batches):
        data_in_timewindow = {
            'position': [],
            'rotation': [],
            'timestamps': [],
            'images': [],
            'file_start': batch[0].split('/')[-1][:-4],
            'file_end': batch[-1].split('/')[-1][:-4],
        }

        indices = []
        for j in range(len(batch)):
            indices.append(pointer)
            pointer += 1
        # print("Processing batches", batch, "and get", indices)

        for pkl_file in batch:
            with open(pkl_file, 'rb') as f:
                data = pickle.load(f)
            data_in_timewindow['position'] += data['position']
            data_in_timewindow['rotation'] += data['rotation']
            data_in_timewindow['timestamps'] += data['timestamps']
            for i, img in enumerate(data['images']):
                data['images'][i] = img[:, :, ::-1]  # Convert BGR to RGB
            data_in_timewindow['images'] += data['images']

        # data_in_timewindow['images'] = sample_by_fps(data_in_timewindow, fps=10)
        data_in_timewindow['frame_indices'] = indices
        pil_images = []
        for image in data_in_timewindow['images']:
            pil_image = Image.fromarray(image) # Convert to PIL Image
            pil_images.append(pil_image)
        data_in_timewindow['images'] = pil_images
        caption_manager.caption_video(data_in_timewindow, query=DEFAULT_QUERY, max_retries=2)

    caption_manager.save_caption_data()

if __name__ == "__main__":
    main()