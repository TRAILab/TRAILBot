import os
import json
import time
from datetime import datetime

import numpy as np
from PIL import Image
from omegaconf import DictConfig
from termcolor import colored
from langchain_huggingface import HuggingFaceEmbeddings

class Captioner:
    def caption(self, images: list[Image.Image]):
        raise NotImplementedError


class NumpyEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        return json.JSONEncoder.default(self, obj)

class CaptionManager:
    def __init__(self, args: DictConfig, captioner: Captioner) -> None:
        self.args: DictConfig = args
        self.init_json()
        self.embedder: HuggingFaceEmbeddings = HuggingFaceEmbeddings(model_name='mixedbread-ai/mxbai-embed-large-v1')
        # self.embedder: HuggingFaceEmbeddings = HuggingFaceEmbeddings(
        #     model_name="sentence-transformers/all-MiniLM-L6-v2",
        #     encode_kwargs={'normalize_embeddings': True}
        # )
        self.captioner: Captioner = captioner

    def init_json(self) -> None:
        self.outputs: list = []
        seq_timestamp: float = datetime.fromtimestamp(time.time())
        os.makedirs(self.args.output_dir, exist_ok=True)
        self.output_path: str = os.path.join(self.args.output_dir, f"captions_{self.args.model_name}.json")
        with open(self.output_path, 'w') as f:
            json.dump(self.outputs, f, cls=NumpyEncoder)

    # TODO: Directly save to vector DB later
    def save_caption_data(self) -> None:
        print(f"Saving caption data {len(self.outputs)}...")
        with open(self.output_path, "w", encoding="utf-8") as f:
            json.dump(self.outputs, f, cls=NumpyEncoder, indent=2)
        # print("First 2 captions:", self.outputs[:2])
        print(f"Caption data saved to {self.output_path}")

    def caption_video(self, data_in_timewindow: dict, query: str, max_retries: int = 2) -> None:
        position: np.ndarray = np.array(data_in_timewindow['position'])
        rotation: np.ndarray = np.array(data_in_timewindow['rotation'])
        timestamps: np.ndarray = np.array(data_in_timewindow['timestamps'])
        images: list = data_in_timewindow['images']

        images = images[::30 // self.args.num_video_frames]
        for attempt in range(max_retries):
            try:
                # print("Processing images", len(images))
                out_text: str = self.captioner.caption(images, query=query)
                print(f"scene details: {out_text}")
                break
            except json.JSONDecodeError as e:
                print(colored(f"[WARNING] JSON decoding failed (attempt {attempt+1}/{max_retries})", "red"))
                print(colored(f"Raw output:\n{repr(out_text)}", "yellow"))
                if attempt < max_retries - 1:
                    print("[INFO] Retrying caption generation...")
                else:
                    print(colored("[ERROR] All caption retries failed. Skipping this segment.", "red"))
                    scene: str = "Invalid caption"
                    location: str = "Unknown"

        text_embedding = self.embedder.embed_query(out_text)
        entity: dict = {
            'id': timestamps[0],
            'position': position.mean(axis=0).tolist(),
            'rotation': rotation.mean(axis=0).tolist(),
            'times': timestamps.mean(),
            'caption': out_text,
            'file_start': data_in_timewindow['file_start'],
            'file_end': data_in_timewindow['file_end'],
            'frame_indices': data_in_timewindow.get('frame_indices', []),
            'text_embedding': text_embedding
        }

        self.outputs.append(entity)