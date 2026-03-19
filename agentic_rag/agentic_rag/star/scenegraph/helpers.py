import os
import gzip
import pickle
from datetime import datetime

import torch
from tqdm import trange
from sentence_transformers import SentenceTransformer

from agentic_rag.star.some_class.map_calss import MapObjectList
from agentic_rag.star.utils.utils import get_observation_by_window

def iter_by_event_depth(observation_buffer, loop_event):
    while not loop_event.is_set():
        data = observation_buffer.get()
        yield data

def iter_by_event_pcd(observation_buffer, loop_event):
    while not loop_event.is_set():
        data_in_window = observation_buffer.get()
        data = get_observation_by_window(data_in_window) # 10 frames
        yield data

def iter_by_dataset(datasets, *args):
    for idx in trange(len(datasets)):
        data = datasets[idx]
        yield data

def read_scenegraph(scene_graph_path):
    with gzip.open(scene_graph_path, "rb") as f:
        data = pickle.load(f)

    objects = MapObjectList(device="cuda")
    objects.load_serializable(data["objects"])
    timestamps = data["timestamps"]
    poses = data["poses"]

    all_indices = set()
    for obj in objects:
        indices = obj['image_idx']
        all_indices.update(indices)
    idx = max(all_indices) + 1
    for obj in objects:
        obj['bbox'].color = (0, 1, 0)  # Reset bbox color to green

    return objects, timestamps, poses, idx

def init_scenegraph(last_graph_dir):
    if last_graph_dir is None:
        objects, timestamps, poses, idx = MapObjectList(device="cuda"), [], [], 0 # (idx, timestamp)
    else:
        objects, timestamps, poses, idx = read_scenegraph(last_graph_dir)
        print(f"Loaded last scene graph from {last_graph_dir}, containing {len(objects)} objects.")
    return objects, timestamps, poses, idx

def prepare_mos_model(cfg):
    mos_model = None
    if cfg.filter_dynamic:
        import mos4d.models.models as models
        weights = cfg.mos_path
        mos_cfg = torch.load(weights)["hyper_parameters"]
        ckpt = torch.load(weights)
        mos_model = models.MOSNet(mos_cfg)
        mos_model.load_state_dict(ckpt["state_dict"])
        # mos_model = mos_model.cuda()
        mos_model = mos_model.to("cuda")
        mos_model.eval()
        mos_model.freeze()
    return mos_model

def load_background_objects(cfg, BG_CAPTIONS_Pro_Sim, BG_CAPTIONS):
    if cfg.use_bg:
        bg_objects = {c: None for c in BG_CAPTIONS_Pro_Sim}
        # Load SBERT model for background caption encoding
        # sbert_model = SentenceTransformer(cfg.sbert_path)
        sbert_model = SentenceTransformer('sentence-transformers/all-MiniLM-L6-v2')
        sbert_model = sbert_model.to("cuda")
        # Encode background captions
        bg_fts = []
        for bg_cation in BG_CAPTIONS:
            bg_ft = sbert_model.encode(bg_cation, convert_to_tensor=True)
            bg_ft = bg_ft / bg_ft.norm(dim=-1, keepdim=True)
            bg_ft = bg_ft.squeeze()
            bg_fts.append(bg_ft)
    else:
        bg_objects = None
        bg_fts = []
    return bg_objects, bg_fts

def save_scene_graph(configs, objects, timestamps, poses):
    if configs.save_pcd:
        print("Saving point cloud map...")
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        results = {
            'objects': objects.to_serializable(),
            # 'bg_objects': None if bg_objects is None else bg_objects.to_serializable(),
            'cfg': configs,
            'timestamps': timestamps,
            'poses': poses
        }
        pcd_save_path = configs.save_pcd_path + "full_pcd.pkl.gz"
        # If it does not exist, create the new directory
        os.makedirs(os.path.dirname(pcd_save_path), exist_ok=True)
        with gzip.open(pcd_save_path, "wb") as f:
            pickle.dump(results, f)
        print(f"Saved point cloud map to {pcd_save_path}")
    else:
        print("Point cloud map not saved.")