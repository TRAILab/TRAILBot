
import re
import sys
import glob
import yaml
import time
import json
from openai import models
import tqdm
import os, sys
parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)
sys.path.append(sys.path[0] + '/..')

import json
import gzip
import pickle

import argparse
import traceback
import distinctipy
from string import Template

import hydra
import torch
import open3d as o3d
import networkx as nx
from pathlib import Path
from termcolor import colored
from collections import Counter
from omegaconf import DictConfig
import agentic_rag.star.clio_batch.helpers as helpers
import numpy as np
import pickle as pkl
from termcolor import colored
from dataclasses import asdict
from PIL import Image as PILImage
from time import strftime, localtime
from langchain_core.prompts import PromptTemplate
from agentic_rag.star.some_class.map_calss import MapObjectList
from langchain_community.chat_models import ChatOllama

from langchain_huggingface import HuggingFaceEmbeddings
from sentence_transformers import SentenceTransformer

# Add the parent directory to sys.path
parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)
sys.path.append(sys.path[0] + '/..')

from agentic_rag.star.memory.memory import MemoryItem
from agentic_rag.star.agents.non_agent import NonAgent
from agentic_rag.star.memory.text_memory import TextMemory

from agentic_rag.star.agents.vlm_non_agent import VLMNonAgent
from agentic_rag.star.agents.remembr_agent_aib import ReMEmbRAgent_AIB
from agentic_rag.star.agents.remembr_agent_vanila import ReMEmbRAgent_VANILA
from agentic_rag.star.agents.remembr_agent_sg import ReMEmbRAgent_SG
from agentic_rag.star.memory.milvus_memory_isaacsim import MilvusMemory
from agentic_rag.star.memory.video_memory import VideoMemory, ImageMemoryItem
from agentic_rag.star.utils.utils import print_to_cot_log
from agentic_rag.star.utils.util_isaacsim import (
    get_caption,
    assign_frame_indices,
    assign_object_ids,
    extract_gt_times,
    plot_multi_method_scores,
    plot_scenegraph_scores,
    # plot_retrieval_scores_with_gt
)

from agentic_rag.star.clio_batch.ib_cluster import ClusterIB, ClusterIBConfig
from agentic_rag.star.clio_batch.aib_helper import (
    cluster_task_scores,
    select_relevant_clusters,
    visualize_highlighted_clusters_open3d,
    visualize_graph_highlight,
    build_object_graph_smart
)

# Vivid, high-contrast base palette (RGB in [0,1]); adapted from Tableau/Glasbey-like sets
_BASE_VIVID = [
    (0.121, 0.466, 0.705),  # blue
    (1.000, 0.498, 0.054),  # orange
    (0.172, 0.627, 0.172),  # green
    (0.839, 0.152, 0.156),  # red
    (0.580, 0.404, 0.741),  # purple
    (0.549, 0.337, 0.294),  # brown
    (0.890, 0.467, 0.761),  # pink
    (0.498, 0.498, 0.498),  # gray
    (0.737, 0.741, 0.133),  # olive
    (0.090, 0.745, 0.811),  # cyan
    (0.000, 0.000, 0.000),  # black
    (1.000, 0.000, 0.000),  # bright red
]

background_terms = [
                    "wall", "ground", "floor", "ceiling", "pillar", "beam",
                    "staging area", "walkway", "window", "ramp", "shelf wall"
                ]

def get_background_terms(sbert_model= None, background_terms= None):
    def _encode_query_sbert(sbert_model, query: str):
        q = sbert_model.encode(query, convert_to_tensor=True)
        q = q / q.norm(dim=-1, keepdim=True)
        return q.squeeze()

    embs = []
    for term in background_terms:
        try:
            e = _encode_query_sbert(sbert_model, term)  # returns unit-norm tensor
            embs.append(e.detach().cpu().view(-1))
        except Exception as ex:
            print(f"[BG-ENC] Failed to encode '{term}': {ex}")
    if len(embs) > 0:
        bg_mat = torch.stack(embs, dim=0)  # [B, D]
    else:
        print("[BG-ENC] No valid background embeddings; filtering disabled.")
        bg_mat = None
    return bg_mat


def write_goal_to_file(pred_pos, path="/tmp/goal_pose.json"):
    goal_data = {
        "x": float(pred_pos[0]),
        "y": float(pred_pos[1]),
        "z": float(pred_pos[2]),
    }
    with open(path, "w") as f:
        json.dump(goal_data, f)
    print(f"Saved predicted goal to {path}")

def _get_distinct_palette(n: int, seed: int = 7):
    """
    Return a palette of n distinct colors in [0,1], guaranteeing at least 10
    highly distinct colors by starting from a vivid base palette and extending
    with distinctipy if necessary.
    """
    m = max(n, 10)  # ensure at least 10 colors
    if m <= len(_BASE_VIVID):
        return _BASE_VIVID[:m]
    # Extend palette with additional distinct colors while excluding the base ones
    extra = distinctipy.get_colors(m - len(_BASE_VIVID), exclude_colors=_BASE_VIVID, rng=seed)
    return _BASE_VIVID + extra


def parse_json(string):
    parsed = re.search(r"```json(.*?)```", string, re.DOTALL| re.IGNORECASE).group(1).strip()
    return eval(parsed)

# we can have binary, position-based, time-based, or description-based. let's answer accordingly
def evaluate_output(qa_instance, predicted, scenegrpahh=None, manual_qa=False):
    out_error = {}
    if manual_qa:
        out_error['position_error'] = 0.0
        return

    q_type = qa_instance['type']
    if 'position' in q_type:
        answer = np.array(qa_instance['answers']['position'])
        # answer = np.array(qa_instance['answers']['target_bbox']['center'])

        # compute L2 loss between predicted['binary'] and answer
        if type(predicted['position']) == str:
            predicted['position'] = eval(predicted['position'])
        try:
            if type(predicted['object_id']) == str:
                predicted['object_id'] = eval(predicted['object_id'])

            # check if object_id is valid, by default we set it to null
            if predicted['object_id'] is not None:
                pred_pos = np.array(scenegrpahh[int(predicted['object_id'])]['bbox'].center)
                print(colored(f"Using predicted object id {predicted['object_id']} with position {pred_pos}", "yellow"))
            else:
                pred_pos = np.array(predicted['position'])
                print(colored(f"Using predicted position {pred_pos}", "yellow"))
        except Exception as e:
            print(e)
            pred_pos = np.array(predicted['position'])
            print(colored(f"Using predicted position {pred_pos}", "yellow"))
        try:
            dist = np.linalg.norm(answer - pred_pos)
        except Exception as e:
            print(e)
            dist = 20

        out_error['position_error'] = dist

    elif 'binary' in q_type:

        answer = qa_instance['answers']['text'][1] # we made this assumption in other examples that binary answer is the second one

        if 'binary' in predicted and (predicted['binary'].lower() == "yes" or predicted['binary'].lower() == "no"):
            # get correct/incorrect label
            if predicted['binary'].lower() == answer.lower():
                correct = 1
            else:
                correct = 0

            out_error['binary_iscorrect'] = correct

    elif 'time' in q_type:

        answer = np.array(qa_instance['answers']['time'])

        # compute L2 loss between predicted['binary'] and answer
        if type(predicted['time']) == str:
            predicted['time'] = eval(predicted['time'])
        pred_time = np.array(predicted['time'])

        dist = abs(answer - pred_time)

        out_error['time_error'] = dist

    elif 'duration' in q_type:

        answer = np.array(qa_instance['answers']['duration'])

        # compute L2 loss between predicted['binary'] and answer
        if type(predicted['duration']) == str:
            predicted['duration'] = eval(predicted['duration'])
        pred_time = np.array(predicted['duration'])

        dist = abs(answer - pred_time)

        out_error['duration_error'] = dist

    elif 'text' in q_type:
        answer = qa_instance['answers']['text']
        out_error = {'answer': answer}
        print(colored(f"Ground truth answers: {answer}", "yellow"))
        print(colored(f"Robot's answer: {predicted['text']}", "yellow"))


    else:
        raise Exception("We do not support question type " + q_type)

    return out_error


def answer_squad_question(model, question, qa_instance, scenegrpahh=None, manual_qa=False):

    # print(f'Question: {question}')
    print(colored(f'Question: {question}', "green", attrs=["bold"]))

    parsed = None
    while True:
        try:

            start_time = time.time()
            response = model.query(question) # the key part of the code

            end_time = time.time()

            elapsed = end_time - start_time

            parsed = asdict(response)
            print(colored(f"Raw response: {parsed}", "blue"))

            out_error = evaluate_output(qa_instance, parsed, scenegrpahh, manual_qa)


            # print("Time elapsed", elapsed)

        except Exception as e:
            print(parsed)
            print(e)
            traceback.print_exception(*sys.exc_info())
            continue

        return_dict = {"response": parsed}
        return_dict.update(parsed)
        return_dict['error'] = out_error
        return_dict['elapsed'] = elapsed

        return return_dict


def load_scenegraph(result, start_time=None, end_time=None, dataset_start_timestamp=None, fps=10):
    '''
    Load the full scene graph, and optionally filter objects by time window.

    Args:
        result_path (str): path to scene graph pkl
        start_time (float): task start time (absolute, e.g., seconds since epoch)
        end_time (float): task end time
        dataset_start_timestamp (float): the timestamp corresponding to image_idx=0
        fps (int): frames per second (default 10)
    '''
    objects, objects_all = result[0], result[1]
    # with gzip.open(result_path, "rb") as f:
    #     results = pickle.load(f)
    #     # print(f"[load_result] Loaded {len(results)} objects from {result_path}")
    #     #print(colored(f"[load_result] Loaded {len(results)} objects from {result_path}", "grey"))

    # if isinstance(results, dict):
    #     objects = MapObjectList()
    #     objects.load_serializable(results["objects"])

    #     if results['bg_objects'] is None:
    #         bg_objects = None
    #     else:
    #         bg_objects = MapObjectList()
    #         bg_objects.load_serializable(results["bg_objects"])

    # elif isinstance(results, list):
    #     objects = MapObjectList()
    #     objects.load_serializable(results)
    #     bg_objects = None
    # else:
    #     raise ValueError("Unknown results type: ", type(results))

    # for i, obj in enumerate(objects):
    #     objects[i]['caption'] = get_caption(obj['caption'], method='lastone')
    #     objects[i]['image_idx'] = sorted(set(obj['image_idx']))
    #     objects[i]['obj_id'] = i
    #     objects[i]['time'] = [dataset_start_timestamp + (idx * 10 / fps) for idx in obj['image_idx']]
    #     # print(f"[load_result] Object {i} has {len(obj['image_idx'])} images, with time {obj['time']}")
    # objects_all = objects.copy()

    if start_time is not None and end_time is not None and dataset_start_timestamp is not None:
        filtered_objects = []
        for obj in objects:
            if 'image_idx' not in obj:
                continue
            # Remove duplicates and sort indices
            idx_list = obj['image_idx']
            matched = True

            # Check if any index falls into the time window
            # for idx in idx_list:
            #     obj_time = dataset_start_timestamp + (idx*10 / fps)
            #     if start_time <= obj_time <= end_time:
            #         matched = True
            #         break
            if matched:
                filtered_objects.append(obj)

        # print(f"[load_result] Total objects in scene graph: {len(objects)}")
        # print(f"[load_result] Objects kept after time filtering: {len(filtered_objects)}")
        print(colored(f"[load_result] Total objects in scene graph: {len(objects)}", "white", attrs=["dark"]))
        print(colored(f"[load_result] Objects kept after time filtering: {len(filtered_objects)}", "white", attrs=["dark"]))
        objects = MapObjectList(filtered_objects)  # only keep filtered objects

    # # 重新生成 instance colors
    # instance_colors = distinctipy.get_colors(len(objects) + (len(bg_objects) if bg_objects else 0), pastel_factor=0.5)
    # instance_colors = {str(i): c for i, c in enumerate(instance_colors)}

    return objects, objects_all#, bg_objects, instance_colors

def load_scenegraph_working(result_path, start_time=None, end_time=None, dataset_start_timestamp=None, fps=10):
    '''
    Load the full scene graph, and optionally filter objects by time window.

    Args:
        result_path (str): path to scene graph pkl
        start_time (float): task start time (absolute, e.g., seconds since epoch)
        end_time (float): task end time
        dataset_start_timestamp (float): the timestamp corresponding to image_idx=0
        fps (int): frames per second (default 10)
    '''
    with gzip.open(result_path, "rb") as f:
        results = pickle.load(f)
        # print(f"[load_result] Loaded {len(results)} objects from {result_path}")
        #print(colored(f"[load_result] Loaded {len(results)} objects from {result_path}", "grey"))

    if isinstance(results, dict):
        objects = MapObjectList()
        objects.load_serializable(results["objects"])

        if results['bg_objects'] is None:
            bg_objects = None
        else:
            bg_objects = MapObjectList()
            bg_objects.load_serializable(results["bg_objects"])

    elif isinstance(results, list):
        objects = MapObjectList()
        objects.load_serializable(results)
        bg_objects = None
    else:
        raise ValueError("Unknown results type: ", type(results))

    for i, obj in enumerate(objects):
        objects[i]['caption'] = get_caption(obj['caption'], method='lastone')
        objects[i]['image_idx'] = sorted(set(obj['image_idx']))
        objects[i]['obj_id'] = i
        objects[i]['time'] = [dataset_start_timestamp + (idx * 10 / fps) for idx in obj['image_idx']]
        # print(f"[load_result] Object {i} has {len(obj['image_idx'])} images, with time {obj['time']}")
    objects_all = objects.copy()

    if start_time is not None and end_time is not None and dataset_start_timestamp is not None:
        filtered_objects = []
        for obj in objects:
            if 'image_idx' not in obj:
                continue
            # Remove duplicates and sort indices
            idx_list = obj['image_idx']
            matched = False

            # Check if any index falls into the time window
            for idx in idx_list:
                obj_time = dataset_start_timestamp + (idx*10 / fps)
                if start_time <= obj_time <= end_time:
                    matched = True
                    break
            if matched:
                filtered_objects.append(obj)

        # print(f"[load_result] Total objects in scene graph: {len(objects)}")
        # print(f"[load_result] Objects kept after time filtering: {len(filtered_objects)}")
        print(colored(f"[load_result] Total objects in scene graph: {len(objects)}", "white", attrs=["dark"]))
        print(colored(f"[load_result] Objects kept after time filtering: {len(filtered_objects)}", "white", attrs=["dark"]))
        objects = MapObjectList(filtered_objects)  # only keep filtered objects

    # # 重新生成 instance colors
    # instance_colors = distinctipy.get_colors(len(objects) + (len(bg_objects) if bg_objects else 0), pastel_factor=0.5)
    # instance_colors = {str(i): c for i, c in enumerate(instance_colors)}

    return objects, objects_all#, bg_objects, instance_colors

def load_memory(args, qa_instance=None, use_milvus=True, use_optimal_context=False, ip_address='127.0.0.1', embedder=None, objects=None, objects_all=None):
    # Here we load everything needed to load a MilvusDB instance neatly
    # captions_path = os.path.join(args.data_dir, 'captions', str(args.sequence_id), f'{args.caption_file}_{str(args.sequence_id)}.json')
    captions_path = os.path.join(args.data_dir, str(args.sequence_id), 'caption', f'{args.caption_file}.json')
    with open(captions_path, 'r') as f:
        out = json.load(f)

    all_start_times = np.array([float(x['file_start'][:-4]) for x in out])
    all_end_times = np.array([float(x['file_end'][:-4]) for x in out])


    if args.all_mem:
        start_time =  all_start_times[0]
        end_time = all_end_times[-1]
        # if we want to use the full memory, we need to set the end time to the last frame
    else:
        start_time = np.float64(qa_instance['start_time'])
        end_time = qa_instance['end_time']

    # scene_graph = args.scene_graph
    if use_milvus:
        # milv = MilvusWrapper(ip_address=ip_address)
        memory = MilvusMemory(f"eval_memory_{args.sequence_id}", db_ip=ip_address, time_offset=start_time, embedder=embedder, args=args)
    elif 'vlm' in args.model:
        memory = VideoMemory()
    else:
        memory = TextMemory()

    memory.reset()

    outputs = []

    # Compute start idx
    diff = all_start_times - start_time
    start_idx = np.argmin(np.abs(diff))
    # Compute end idx
    diff = all_end_times - end_time
    end_idx = np.argmin(np.abs(diff))


    # updata caption to include frame index
    #out = assign_frame_indices(out, all_start_times, all_end_times, duration=3.0)


    pkl_files = glob.glob(os.path.join(args.coda_dir, str(args.sequence_id), '*.pkl'))
    pkl_files.sort(key=lambda x: float(x.split('/')[-1][:-4]))

    if 'remembr' in args.model:
        objects, objects_all = load_scenegraph(
                # result_path=scene_graph_path,
                result=(objects, objects_all),
                start_time=all_start_times[0],
                end_time=all_end_times[-1],
                dataset_start_timestamp=all_start_times[0],
                fps=10  # or your real fps
            )
    else:
        objects, objects_all = load_scenegraph(
                # result_path=scene_graph_path,
                result=(objects, objects_all),
                start_time=start_time,#qa_instance['start_time'],
                end_time=end_time,#qa_instance['end_time'],
                dataset_start_timestamp=all_start_times[0],
                fps=10  # or your real fps
            )

    out = assign_object_ids(out, objects)
    # for i, x in enumerate(out):
    #     print(x['frame_idx'])
    #     print(x['object_id'])
    for i in range(start_idx, end_idx+1):

        item = out[i]
        obj_id = item.get('object_id', None)
        if isinstance(obj_id, list):
            object_id = ','.join(map(str, obj_id))
        elif isinstance(obj_id, int):
            object_id = str(obj_id)
        elif obj_id is None:
            object_id = ''

        entity = {
            'position': item['position'],
            'theta': item['rotation'], # ignoring rotation
            'time': item['times'],
            'caption': item['caption'][:3000],
            'object_id': object_id,
        }

        outputs.append(entity)

        if type(memory) == VideoMemory:

            qa_start_path = os.path.join(args.coda_dir, str(args.sequence_id), out[i]['file_start'])
            qa_end_path = os.path.join(args.coda_dir, str(args.sequence_id), out[i+1]['file_start'])

            qa_start_idx = pkl_files.index(qa_start_path)
            qa_end_idx = pkl_files.index(qa_end_path)
            idxs = np.linspace(qa_start_idx, qa_end_idx, 6, dtype=int)

            for pkl_idx in idxs:
                # pkl_path = os.path.join(args.coda_dir, str(args.sequence_id), item['file_start'])
                pkl_path = pkl_files[pkl_idx]
                with open(pkl_path, 'rb') as f:
                    pkl_data = pkl.load(f)
                entity['image'] = PILImage.fromarray(pkl_data['cam0'].astype('uint8'), 'RGB')

            entity = ImageMemoryItem.from_dict(entity)
        else:
            entity = MemoryItem.from_dict(entity)

        if use_milvus:

            memory.insert(entity, text_embedding=item['text_embedding'])
            if args.all_mem:
                memory.set_scene_graph(objects_all)
            else:
                memory.set_scene_graph(objects)
        else:
            memory.insert(entity)

    if use_optimal_context:
        # then replace the full memory with the optimal context
        memory = TextMemory()
        memory.insert(qa_instance['context'])


    return memory, outputs, all_start_times[0], objects

def load_SG_data(args, fps=10):
    '''
    Load the full scene graph, and optionally filter objects by time window.

    Args:
        result_path (str): path to scene graph pkl
        start_time (float): task start time (absolute, e.g., seconds since epoch)
        end_time (float): task end time
        dataset_start_timestamp (float): the timestamp corresponding to image_idx=0
        fps (int): frames per second (default 10)
    '''
    # scene_graph_path = os.path.join(args.data_dir, 'scene_graphs', f'{args.scenegraph_file}_{str(args.sequence_id)}.pkl.gz')
    scene_graph_path = os.path.join(args.data_dir, str(args.sequence_id), 'pcd', f'{args.scenegraph_file}.pkl.gz')
    print("scene_graph_path\n", scene_graph_path)
    # captions_path = os.path.join(args.data_dir, 'captions', str(args.sequence_id), 'captions', f'{args.caption_file}_{str(args.sequence_id)}.json')
    captions_path = os.path.join(args.data_dir, str(args.sequence_id), 'caption', f'{args.caption_file}.json')
    with open(captions_path, 'r') as f:
        out = json.load(f)
    all_start_times = np.array([float(x['file_start'][:-4]) for x in out])
    dataset_start_timestamp = all_start_times[0]

    with gzip.open(scene_graph_path, "rb") as f:
        results = pickle.load(f)
        # print(f"[load_result] Loaded {len(results)} objects from {result_path}")
        #print(colored(f"[load_result] Loaded {len(results)} objects from {result_path}", "grey"))

    if isinstance(results, dict):
        objects = MapObjectList()
        objects.load_serializable(results["objects"])

        if results.get('bg_objects', None) is None:
            bg_objects = None
        else:
            bg_objects = MapObjectList()
            bg_objects.load_serializable(results["bg_objects"])

    elif isinstance(results, list):
        objects = MapObjectList()
        objects.load_serializable(results)
        bg_objects = None
    else:
        raise ValueError("Unknown results type: ", type(results))

    for i, obj in enumerate(objects):
        objects[i]['caption'] = get_caption(obj['caption'], method='majority')
        #print(f"[load_result] Object {i} has caption: {objects[i]['caption']}")
        objects[i]['image_idx'] = sorted(set(obj['image_idx']))
        objects[i]['obj_id'] = i
        objects[i]['time'] = [dataset_start_timestamp + (idx * 10 / fps) for idx in obj['image_idx']]
        # print(f"[load_result] Object {i} has {len(obj['image_idx'])} images, with time {obj['time']}")
    objects_all = objects.copy()

    timestamps = np.array(results['timestamps'])
    template = "%Y-%m-%d %H:%M:%S"
    # indice_dict = {datetime.fromtimestamp(t).strftime(template): i for i, t in enumerate(timestamps)}

    return objects, objects_all, timestamps # indice_dict

@torch.no_grad()
def sbert_encode_texts(model, texts):
    """Encode text prompts with SBERT and return L2-normalized numpy (M, D)."""
    if isinstance(texts, str):
        texts = [texts]
    embs = model.encode(texts, convert_to_tensor=True)         # torch [M, D]
    embs = torch.nn.functional.normalize(embs, dim=-1)
    return embs.detach().cpu().numpy().astype(np.float32)      # np [M, D]


def get_caption_features_from_objects(objects):
    """Collect per-object SBERT embeddings into (N, D) numpy array."""
    feats = []
    for obj in objects:
        ft = obj['ft']
        if torch.is_tensor(ft):
            ft = ft.detach().cpu().numpy()
        feats.append(np.asarray(ft, dtype=np.float32))
    return np.stack(feats, axis=0)  # (N, D)


def patch_helpers_for_sbert():
    """Patch Clio's helper to use cosine(task, region) on SBERT features."""
    def _sbert_cosine_sim(task_feats, region_feats, *_unused, **__unused):
        sims = task_feats @ region_feats.T   # (M, N), both L2-normalized
        sims = np.clip(sims, 0.0, None)      # match Clio behavior (no negatives)
        return sims
    helpers.compute_sim_to_tasks = _sbert_cosine_sim

def aabb_overlaps(aabb_i: o3d.geometry.AxisAlignedBoundingBox,
                  aabb_j: o3d.geometry.AxisAlignedBoundingBox,
                  eps: float = 0.0) -> bool:
    """Return True if two AABBs overlap (with optional dilation eps)."""
    mi = np.asarray(aabb_i.get_min_bound())
    Mi = np.asarray(aabb_i.get_max_bound())
    mj = np.asarray(aabb_j.get_min_bound())
    Mj = np.asarray(aabb_j.get_max_bound())
    # overlap if intervals intersect on all 3 axes
    return np.all(mi <= Mj + eps) and np.all(mj <= Mi + eps)

def build_object_graph(objects, region_features, eps=0.0):
    G = nx.Graph()

    for i, obj in enumerate(objects):
        G.add_node(
            i,
            position=np.asarray(obj['bbox'].center),
            semantic_feature=region_features[i].reshape(-1, 1),
            bounding_box=obj['bbox'],
        )

    # O(N^2) pass; fine for moderate N
    for i in range(len(objects)):
        aabb_i = objects[i]['bbox'].get_axis_aligned_bounding_box()
        for j in range(i + 1, len(objects)):
            aabb_j = objects[j]['bbox'].get_axis_aligned_bounding_box()
            if aabb_overlaps(aabb_i, aabb_j, eps=eps):
                G.add_edge(i, j)

    return G

def _axis_index(axis):
    if isinstance(axis, int):
        return axis
    return {'x': 0, 'y': 1, 'z': 2}[axis.lower()]

def _aabb_corners_from_obj(obj, dilate_eps=0.0):
    """Return (8,3) AABB corners; optionally dilate by eps (meters)."""
    aabb = obj['bbox'].get_axis_aligned_bounding_box()
    if dilate_eps > 0.0:
        mn = np.asarray(aabb.get_min_bound(), dtype=np.float32)
        mx = np.asarray(aabb.get_max_bound(), dtype=np.float32)
        ctr = 0.5 * (mn + mx)
        ext = 0.5 * (mx - mn) + dilate_eps
        aabb = o3d.geometry.AxisAlignedBoundingBox(ctr - ext, ctr + ext)
    return np.asarray(aabb.get_box_points(), dtype=np.float32)

def _vertical_overlap_mask(mn, mx, v_ax, use_overlap=True, slack=0.25):
    """(N,N) upper-tri mask for vertical overlap (or center diff <= slack)."""
    if use_overlap:
        vmin = mn[:, v_ax][:, None]; vmax = mx[:, v_ax][:, None]
        vmin2 = mn[:, v_ax][None, :]; vmax2 = mx[:, v_ax][None, :]
        inter_v = torch.minimum(vmax, vmax2) - torch.maximum(vmin, vmin2)
        M = (inter_v > 0)
    else:
        vc  = ((mn[:, v_ax] + mx[:, v_ax]) * 0.5)[:, None]
        vc2 = ((mn[:, v_ax] + mx[:, v_ax]) * 0.5)[None, :]
        M = (torch.abs(vc - vc2) <= slack)
    return torch.triu(M, diagonal=1)

def _ground_like_mask(mn, mx, v_ax, down_positive, height_thresh=0.05, floor_thresh=0.05):
    """
    Returns (N,) bool ground-ish: thin on vertical OR touching floor side.
    For down_positive=True (+Y is down): floor is at LARGE positive coordinate.
    For down_positive=False (usual +Z up): floor is at SMALL coordinate.
    """
    extent_v = (mx[:, v_ax] - mn[:, v_ax])              # thickness along vertical axis
    base_v   = (mx[:, v_ax] if down_positive else mn[:, v_ax])
    is_thin  = extent_v < height_thresh
    is_floor = (base_v > floor_thresh) if down_positive else (base_v < floor_thresh)
    return (is_thin | is_floor)

def build_object_graph_ground_safe(
    objects,
    region_features,
    *,
    vertical_axis='y',            # <-- your scene uses Y as vertical
    down_positive=True,           # <-- +Y is down toward the floor
    dist_radius=2.0,              # meters
    z_overlap=False,               # vertical interval overlap vs. slack
    z_slack=0.25,                 # meters, only used when z_overlap=False
    iou_thresh=0.02,              # small >0 to avoid floor-touch bridges
    covis_min=0,                  # require shared frames; 0 disables
    knn=6,                        # sparsify; None to add all gated edges
    ground_height_thresh=0.05,    # “thin” threshold on vertical extent
    ground_floor_thresh=0.05,     # floor contact threshold along vertical axis
    dilate_eps=0.02               # inflate all AABBs by a few cm (stability)
):
    """
    Build a sparse, well-gated object adjacency graph robust to Y-down frames.
    """
    G = nx.Graph()
    N = len(objects)
    v_ax = _axis_index(vertical_axis)

    # Add nodes
    for i, obj in enumerate(objects):
        G.add_node(
            i,
            position=np.asarray(obj['bbox'].center),
            semantic_feature=region_features[i].reshape(-1, 1),
            bounding_box=obj['bbox'],
        )
    if N <= 1:
        return G

    # Gather (N,8,3) corners (optionally dilated) and image sets
    corners = np.stack([_aabb_corners_from_obj(o, dilate_eps=dilate_eps) for o in objects], axis=0)
    corners = torch.from_numpy(corners)  # (N,8,3) torch.float32
    img_sets = [set(o.get('image_idx', [])) for o in objects]

    # Min/max/centers
    mn = corners.min(dim=1).values                              # (N,3)
    mx = corners.max(dim=1).values                              # (N,3)
    centers = 0.5 * (mn + mx)                                   # (N,3)

    # Pairwise distance gating
    D = torch.cdist(centers, centers, p=2)                      # (N,N)
    dist_mask = torch.triu((D <= dist_radius), diagonal=1)

    # Vertical gating (using vertical_axis)
    v_mask = _vertical_overlap_mask(mn, mx, v_ax, use_overlap=z_overlap, slack=z_slack)

    # AABB IoU (vectorized)
    b1_min = mn[:, None, :]; b1_max = mx[:, None, :]
    b2_min = mn[None, :, :]; b2_max = mx[None, :, :]
    inter_min = torch.maximum(b1_min, b2_min)
    inter_max = torch.minimum(b1_max, b2_max)
    inter_vol = torch.prod(torch.clamp(inter_max - inter_min, min=0), dim=2)
    vol1 = torch.prod(b1_max - b1_min, dim=2)
    vol2 = torch.prod(b2_max - b2_min, dim=2)
    union = vol1 + vol2 - inter_vol + 1e-10
    iou = inter_vol / union
    iou_mask = torch.triu((iou > iou_thresh), diagonal=1)

    # Co-visibility gating
    if covis_min > 0:
        covis = torch.zeros((N, N), dtype=torch.bool)
        for i in range(N):
            Si = img_sets[i]
            for j in range(i + 1, N):
                if len(Si & img_sets[j]) >= covis_min:
                    covis[i, j] = True
        covis_mask = covis
    else:
        covis_mask = torch.triu(torch.ones((N, N), dtype=torch.bool), diagonal=1)

    # Ground hygiene (block any pair where either is ground-like along vertical_axis)
    is_ground = _ground_like_mask(mn, mx, v_ax, down_positive,
                                  height_thresh=ground_height_thresh,
                                  floor_thresh=ground_floor_thresh)
    not_ground = ~is_ground
    ground_mask = torch.triu((not_ground[:, None] & not_ground[None, :]), diagonal=1)

    # Combine all gates
    gate = iou_mask & v_mask #dist_mask & v_mask & iou_mask & covis_mask & ground_mask

    # Sparsify with kNN among gated neighbors (per row)
    edges = []
    if knn is None:
        ii, jj = torch.where(gate)
        edges = [(int(i), int(j)) for i, j in zip(ii, jj)]
    else:
        for i in range(N):
            js = torch.nonzero(gate[i], as_tuple=False).flatten()
            if js.numel() == 0:
                continue
            dij = D[i, js]
            k = min(knn, js.numel())
            chosen = js[torch.topk(-dij, k).indices]  # k smallest distances
            edges.extend([(i, int(j)) for j in chosen])

    G.add_edges_from(edges)
    return G

def write_default_cluster_config(cfg_path):
    """Write a reasonable default IB config if not present."""
    if os.path.exists(cfg_path):
        return
    os.makedirs(os.path.dirname(cfg_path), exist_ok=True)
    with open(cfg_path, "w") as f:
        f.write(
            "debug: false\n"
            "debug_folder: ./ib_debug\n"
            "sims_thres: 0.20\n"          # null-task floor; try 0.15–0.30 for SBERT
            "delta: 0.10\n"               # higher => more merging
            "top_k_tasks: 1\n"            # crisp relevance
            "cumulative: false\n"
            "use_lerf_loss: false\n"
            "lerf_loss_cannonical_phrases: []\n"
        )


def run_ib_clustering(region_features, task_features, G_nx, cfg_path):
    """Run Agglomerative IB and return list[list[int]] cluster indices."""
    ib_cfg = ClusterIBConfig(cfg_path)
    ib = ClusterIB(ib_cfg)
    ib.setup_py_x(region_features, task_features)                 # p(x), p(y|x), p(y)
    ib.update_delta_as_part(region_features, task_features)       # scale for factorized case
    ib.initialize_nx_graph(G_nx)                                  # nodes/edges
    clusters = ib.find_clusters()                                  # list of lists (indices into objects)

    return clusters


def merge_cluster(objects, idxs):
    """Merge members into a single object dict (pcd, ft, caption, bbox, etc.)."""
    merged_pcd = o3d.geometry.PointCloud()
    fts, caps = [], []
    for k in idxs:
        merged_pcd += objects[k]['pcd']
        ft_k = objects[k]['ft']
        if torch.is_tensor(ft_k):
            ft_k = ft_k.detach().cpu().numpy()
        fts.append(np.asarray(ft_k, dtype=np.float32))
        caps.append(objects[k]['caption'])

    ft_mean = np.mean(fts, axis=0)
    norm = np.linalg.norm(ft_mean) + 1e-12
    ft_mean = (ft_mean / norm).astype(np.float32)

    caption = Counter(caps).most_common(1)[0][0]
    merged_bbox = merged_pcd.get_oriented_bounding_box()

    return {
        'image_idx': sum((objects[k]['image_idx'] for k in idxs), []),
        'num_detections': sum(objects[k]['num_detections'] for k in idxs),
        'n_points': np.asarray(merged_pcd.points).shape[0],
        'inst_color': objects[idxs[0]].get('inst_color', None),
        'bg_class': None,
        'class_sk': None,
        'caption': caption,
        'captions_ft': None,
        'ft': ft_mean,                  # merged SBERT embedding
        'pcd': merged_pcd,
        'bbox': merged_bbox,
        'img_bbox': None,
        'cluster_members': idxs,
    }


def merge_all_clusters(objects, clusters):
    return [merge_cluster(objects, c) for c in clusters]


def cosine_max_to_tasks(task_features, ft_vec):
    """Max cosine(task, vector)."""
    sims = task_features @ ft_vec.reshape(-1, 1)  # (M, 1)
    return float(sims.max())


def filter_clusters_by_task(clustered_objects, task_features, thresh=0.23):
    """Keep clusters whose embedding is sufficiently similar to any task."""
    kept = []
    for obj in clustered_objects:
        s = cosine_max_to_tasks(task_features, obj['ft'])
        if s >= thresh:
            kept.append(obj)
    return kept

def obb_to_lineset(obb, color=(0, 0, 0)):
    """Convert an Open3D OrientedBoundingBox/AABB to a LineSet for visualization."""
    if hasattr(obb, "get_box_points"):
        pts = np.asarray(obb.get_box_points())
    else:
        # OrientedBoundingBox: sample its corner points by converting to AABB or using its methods
        pts = np.asarray(obb.get_axis_aligned_bounding_box().get_box_points())
    lines = [
        [0,1],[0,2],[0,3],
        [4,5],[4,6],[4,7],
        [1,4],[2,6],[3,7],
        [1,5],[2,7],[3,6],
    ]
    ls = o3d.geometry.LineSet()
    ls.points = o3d.utility.Vector3dVector(pts)
    ls.lines  = o3d.utility.Vector2iVector(lines)
    ls.colors = o3d.utility.Vector3dVector([color for _ in lines])
    return ls

def visualize_clusters_open3d(objects, clusters, save_dir=None, show=True):
    """
    Color each cluster, draw its objects' PCDs + bboxes.
    objects[i] should have 'pcd' (o3d.geometry.PointCloud) and 'bbox' (Open3D OBB/AABB).
    """
    geoms = []

    # Build a palette that is guaranteed to have >= 10 very distinct colors
    full_palette = _get_distinct_palette(len(clusters))
    # Only take as many colors as clusters
    palette = full_palette[:len(clusters)]

    for ci, idxs in enumerate(clusters):
        color = np.array(palette[ci], dtype=float)
        # Slightly dimmer point color for better bbox contrast
        pcolor = np.clip(color * 0.8, 0.0, 1.0)

        # Merge cluster pcd for a big bbox (optional)
        cluster_pcd = o3d.geometry.PointCloud()
        for k in idxs:
            pcd = objects[k]['pcd']

            # color the points
            num_pts = np.asarray(pcd.points).shape[0]
            if num_pts > 0:
                pcd_colored = o3d.geometry.PointCloud(pcd)  # copy
                pcd_colored.colors = o3d.utility.Vector3dVector(
                    np.tile(pcolor, (num_pts, 1))
                )
                geoms.append(pcd_colored)
                cluster_pcd += pcd

            # add each object bbox in cluster color
            ls = obb_to_lineset(objects[k]['bbox'], color=color)
            geoms.append(ls)

        # add one big bbox for the cluster (overview)
        if len(cluster_pcd.points) > 0:
            big_obb = cluster_pcd.get_oriented_bounding_box()
            # Some Open3D versions draw OBB color via .color on OBB or via LineSet.
            # Here we also add a LineSet to ensure visible color across versions.
            try:
                big_obb.color = color
                geoms.append(big_obb)
            except Exception:
                # Fallback: draw as colored LineSet
                geoms.append(obb_to_lineset(big_obb, color=color))

    if save_dir:
        os.makedirs(save_dir, exist_ok=True)
        # save each cluster merged PLY
        for ci, idxs in enumerate(clusters):
            merged = o3d.geometry.PointCloud()
            for k in idxs:
                merged += objects[k]['pcd']
            o3d.io.write_point_cloud(os.path.join(save_dir, f"cluster_{ci:03d}.ply"), merged)

    if show:
        o3d.visualization.draw_geometries(geoms)


def print_task_related_captions(objects, clusters, task_texts, task_features, sim_threshold=0.3):
    """
    For each cluster, find the most related task and print captions of objects
    in that cluster that are related to that task.
    """
    for ci, idxs in enumerate(clusters):
        # 1. Average the cluster's object embeddings
        cluster_vecs = [objects[k]['ft'] for k in idxs if objects[k].get('ft') is not None]
        if len(cluster_vecs) == 0:
            continue
        cluster_mean = np.mean(np.stack(cluster_vecs), axis=0)  # (D,)
        cluster_mean = cluster_mean / np.linalg.norm(cluster_mean)

        # 2. Cosine sim with all task features
        sims = np.dot(task_features, cluster_mean)  # (num_tasks,)
        best_task_idx = int(np.argmax(sims))
        best_task_name = task_texts[best_task_idx]
        best_task_score = sims[best_task_idx]

        print(f"\n=== Cluster {ci} → Task: '{best_task_name}' (score={best_task_score:.3f}) ===")

        # 3. Print captions that match the best task above threshold
        for k in idxs:
            obj_ft = objects[k]['ft']
            if obj_ft is None:
                continue
            score = np.dot(task_features[best_task_idx], obj_ft)
            if score >= sim_threshold:
                print(f"  • {objects[k]['caption']} (sim={score:.3f})")

# -----------------------------
# Run eval: step-by-step
# -----------------------------

def run_eval(args):
    base_dir = args.base_dir # os.path.join(args.data_dir, str(args.sequence_id))
    latest_idx_path = args.latest_idx_path # os.path.join(base_dir, "latest_frame.txt")
    instruction_path = args.instruction_path # os.path.join(base_dir, "instructions", "instruction.json")

    use_milvus = False
    use_optimal_context = False
    # if 'ragbot' in args.model:
    #     base_llm = args.model.split('+')[-1]
    #     agent = RagAgent(llm_type=base_llm, num_ctx=args.num_ctx, temperature=args.temperature)
    #     use_milvus = True
    if 'remembr' in args.model:
        base_llm = args.model.split('+')[-1]
        agent = ReMEmbRAgent_AIB(llm_type=base_llm, num_ctx=args.num_ctx, temperature=args.temperature)
        use_milvus = True
        print(colored("Using Our Proposed Agent", "white", attrs=["dark"]))
    elif 'vanila' in args.model:
        base_llm = args.model.split('+')[-1]
        agent = ReMEmbRAgent_VANILA(llm_type=base_llm, num_ctx=args.num_ctx, temperature=args.temperature)
        use_milvus = True
        print(colored("Using ReMEmbR Agent", "white", attrs=["dark"]))
    elif 'opengraph' in args.model:
        base_llm = args.model.split('+')[-1]
        agent = ReMEmbRAgent_SG(llm_type=base_llm, num_ctx=args.num_ctx, temperature=args.temperature)
        use_milvus = True
        print(colored("Using OpenGraph-Based Agent", "white", attrs=["dark"]))
    elif 'optimal' in args.model:
        base_llm = args.model.split('+')[-1]
        agent = NonAgent(llm_type=base_llm, num_ctx=args.num_ctx, temperature=args.temperature)
        use_optimal_context = True
    elif 'vlm' in args.model:
        agent = VLMNonAgent(llm_type='gpt-4o')
    else:
        agent = NonAgent(llm_type=args.model, num_ctx=args.num_ctx*4, temperature=args.temperature)

    # Load question data
    data_path = os.path.join(args.data_dir, str(args.sequence_id), 'questions', args.qa_file+'.json')
    if os.path.exists(data_path):
        data = json.load(open(data_path, 'r'))
        data = data['data']
    else:
        data = []

    qa_instance = data[0] if len(data) > 0 else None
    # gt_info = extract_gt_times(qa_instance)
    # print("GT info: ", gt_info)

    # below is the variable to keep track of the evaluation
    running_successes = 0
    num_binary = 0
    sr_binary = 0

    running_pos_error = 0
    num_position = 0
    sr_position = 0

    running_time_error = 0
    num_time = 0
    sr_time = 0

    running_duration_error = 0
    num_duration = 0
    sr_duration = 0

    responses = []

    embedder = HuggingFaceEmbeddings(
        model_name='mixedbread-ai/mxbai-embed-large-v1',
        model_kwargs={
            "device": "cuda:0",  #  TITAN Xp
            #"torch_dtype": "auto",
        }
    )
    # object_vis = ObjectVisualizer()
    print("Initializing SBERT model...")
    #sbert_model = SentenceTransformer(cfg.sbert_path)
    sbert_model = SentenceTransformer('sentence-transformers/all-MiniLM-L6-v2')
    sbert_model = sbert_model.to("cuda")
    print(colored("Done initializing SBERT model.", "white", attrs=["dark"]))
    # save the outputs
    out_path = os.path.join(args.out_dir, str(args.sequence_id), args.log_file)
    print("Out path:", out_path)
    os.makedirs(out_path, exist_ok=True)
    # for i in tqdm.tqdm(range(0, len(data)), total=len(data)):
    i = 0
    objects, objects_all, timestamp_list = load_SG_data(args, fps=10)

    # bg_matrix = get_background_terms(sbert_model, background_terms)
    #visualize_objects_org(objects)

    if False:
        # 3) Prepare features
        object_cap_features = get_caption_features_from_objects(objects)  # (N, D)

        task_texts = ["carboard boxes"] # ["find the door", "find bike", "find the a sign", "find a painting"]  # <-- replace with your tasks
        task_features = sbert_encode_texts(sbert_model, task_texts)            # (M, D)

        # 4) Patch Clio helper for SBERT
        patch_helpers_for_sbert()
        # 5) Build primitive graph
        # G_nx = build_object_graph(objects, region_features)

        G_nx = build_object_graph_ground_safe(
        objects, object_cap_features,
        vertical_axis='y',        # your vertical axis
        down_positive=True,       # +Y is down
        dist_radius=3.0,          # start a bit generous, then tighten
        z_overlap=False,          # use slack first, then switch to True
        z_slack=0.40,             # 40 cm vertical slack
        iou_thresh=0.01,          # small positive
        covis_min=0,              # enable later when image_idx is reliable
        knn=6,                    # sparsify neighborhoods
        ground_height_thresh=0.1,
        ground_floor_thresh=0.05, # threshold on +Y for floor contact
        dilate_eps=0.02           # give thin boxes a tiny thickness margin
        )
        # G_nx = build_object_graph_smart(objects, region_features, dist_radius=3)


        # 6) Write config + run IB clustering
        out_path = os.path.join(args.out_dir, str(args.sequence_id), args.log_file)
        Path(out_path).mkdir(parents=True, exist_ok=True)
        cfg_yaml_path = os.path.join(out_path, "cluster_config.yaml")
        write_default_cluster_config(cfg_yaml_path)

        clusters = run_ib_clustering(object_cap_features, task_features, G_nx, cfg_yaml_path)
        print(colored(f"[AIB] Formed {len(clusters)} clusters from {len(objects)} primitives.", "cyan"))

        # 7) Merge cluster members and filter by task relevance
        clustered_objects = merge_all_clusters(objects, clusters)
        print(colored(f"[AIB] After merge: {len(clustered_objects)} objects.", "cyan"))

        TASK_FILTER_THRESH = 0.55  # tune for SBERT
        clustered_objects_filtered = filter_clusters_by_task(clustered_objects, task_features, TASK_FILTER_THRESH)
        print(colored(f"[AIB] Filtered to {len(clustered_objects_filtered)} task-relevant objects.", "cyan"))

        # Print per cluster
        print_task_related_captions(objects, clusters, task_texts, task_features, sim_threshold=0.35)
        # visualize_clusters_open3d(objects, clusters, save_dir=os.path.join(out_path, "clusters_vis"), show=True)
        # 1) Get per-cluster scores & best task
        scores, winners, _ = cluster_task_scores(objects, clusters, task_features)

        # 2) Pick which clusters to highlight
        HIGHLIGHT_THR = 0.5  # tune
        highlight_idxs = select_relevant_clusters(scores, HIGHLIGHT_THR)

        # 3) Open3D view (dim irrelevant)
        visualize_highlighted_clusters_open3d(
            objects, clusters, highlight_idxs, winners, task_texts,
            dim_alpha=0.10,  # lower → dimmer
            save_dir=os.path.join(out_path, "clusters_vis"),
            show=True
        )

        # 4) Graph view (blue = relevant, gray = irrelevant)
        visualize_graph_highlight(
            G_nx, clusters, highlight_idxs,
            out_png=os.path.join(out_path, "graph_highlight.png"),
            show=True
        )

    if args.evaluation_mode:
        print("Running in evaluation mode")
        memory, instance_captions, global_starttime, scene_graph = load_memory(args, qa_instance=None, use_milvus=use_milvus, use_optimal_context=use_optimal_context, ip_address=args.db_ip, embedder=[embedder, sbert_model], objects=objects, objects_all=objects_all)
        
        print("Initialization done, entering evaluation loop...")
        while True: # len(data):
            plot_data, sg_data = [], []
            print(colored(f"global_starttime: {global_starttime}", "white", attrs=["dark"]))
            if len(instance_captions) == 0: # ISSUEset_scene_graph
                print("Length of Instance Captions is 0. It should not be")
                import pdb; pdb.set_trace()

            # print("HISTORY LENGTH", len(instance_captions))
            print(colored(f"Video caption length: {len(instance_captions)}", "white", attrs=["dark"]))

            # 🚀 Set memory for agent
            test_num = i
            if isinstance(agent, ReMEmbRAgent_AIB) or isinstance(agent, ReMEmbRAgent_VANILA) or isinstance(agent, ReMEmbRAgent_SG):
                # If agent is ReMEmbRAgent, attach hybrid reranker with scene graph
                agent.set_memory(memory, scene_graph=scene_graph, dataset_start_timestamp=global_starttime, sbert_model=sbert_model, test_num=test_num, args=args, timestamp_list=timestamp_list)
            else:
                agent.set_memory(memory)

            # Create Chain of Thought log file
            cot_log_file = os.path.join(base_dir, "cot_log", f"cot_log_{test_num}.txt")
            with open(cot_log_file, "w") as f:
                f.write("Chain of Thought Log\n")

            if args.manual_evaluation:
                with open(instruction_path, 'r') as f:
                    instructions = json.load(f)

                while not instructions["trigger"]:
                    with open(instruction_path, 'r') as f:
                        instructions = json.load(f)
                    time.sleep(0.1)
                    continue

                question = instructions["command"]
                if question.isdigit():
                    print("Detected a question index input, jumping to that index")
                    i = int(question)
                    qa_instance = data[i]
                    gt_info = extract_gt_times(qa_instance)
                    question = qa_instance['question']
                    question = question.split("\n")[-1]
                else:
                    qa_instance = None #data[i]
                    gt_info = None

                agent.test_num = i
                agent.cot_log_file = os.path.join(base_dir, "cot_log", args.postfix, f"cot_log_{agent.test_num}.txt")
                agent.memory.cot_log_file = agent.cot_log_file
                print("Current question index:", i, f"saved to {agent.cot_log_file}")
                with open(latest_idx_path, 'w') as f:
                    f.write(str(i))

                # print(f"Question received {question}")
                print_to_cot_log(message=f"Question received: {question}", log_file=agent.cot_log_file)

                with open(instruction_path, "w", encoding="utf-8") as f:
                    json.dump({"trigger": False, "command": ""}, f)

                # question = input("Ask your question: \n") # qa_instance['question'] #
                use_visual = False # input("Do you want to provide an image? (y/n): ").strip().lower() == 'y'

                if use_visual:
                    from LLT import models
                    from LLT.prompts.main_prompt import MAIN_PROMPT
                    messages = []
                    new_prompt = MAIN_PROMPT.replace("[INSERT TASK]", question)

                    # image_path1 = f"/home/trailbot/project_ws/RAG/remembr/captured_images/{'cabinet'}.png" # cabinet  physics2
                    image_path1 = os.path.join(base_dir, "capture", "captured_rgb_0.png") # cabinet  physics2
                    print(f"Using image {image_path1}")
                    messages = models.get_chatgpt_output("gpt-4.1", new_prompt, messages, "system", question, image_path1)
                    print("GPT-4V Response:", messages[-1]['content'])
                    question = messages[-1]['content']
                agent.memory.search_text ={}
                out_dict = answer_squad_question(agent, question, qa_instance, scene_graph, args.manual_evaluation)

                agent_response = out_dict['response']
                if type(agent_response['position']) == str:
                    agent_response['position'] = eval(agent_response['position'])

                pred_pos = np.array(agent_response['position'])
                if agent_response.get('object_id', None) is not None:
                    pred_pos = np.array(scene_graph[int(agent_response['object_id'])]['bbox'].center)

                print("Predicted position", pred_pos)
                # publish_goal(pred_pos, initial_pose, ros_node)
                try:
                    write_goal_to_file(pred_pos, path="/tmp/goal_pose.json")
                except:
                    print("Failed to write goal to file")
                responses.append(out_dict)
            else:
                user_input = input(f"Press Enter to test task {i}, or type 'skip' to skip, or enter an index to jump: ").strip()

                if user_input.lower() == 'skip':
                    i += 1
                    continue
                elif user_input.isdigit():
                    i = int(user_input)
                else:
                    break

                print("Evaluating for question", i)

                qa_instance = data[i]
                question = qa_instance['question']
                context = qa_instance['context']
                start_time = qa_instance['start_time']
                answers = qa_instance['answers']
                length_category = qa_instance['length_category']
                id = qa_instance['id']
                q_type = qa_instance['type']
                question = f"Question type: {q_type}." + question
                out_dict = answer_squad_question(agent, question, qa_instance, scene_graph. args.manual_evaluation)
                gt_info = extract_gt_times(qa_instance)

                # Moved from here

                out_dict['question'] = qa_instance['question']
                out_dict['id'] = id

                error_dict = out_dict['error']

                # keep track of how many of each. usually all CSVs are one type only
                print("Results for question", i)
                if qa_instance['type'] == 'position':
                    num_position += 1
                    if 'position_error' in error_dict:
                        running_pos_error += error_dict['position_error']
                        if error_dict['position_error'] < 5.0: # 20.0:
                            sr_position += 1
                            print(colored(f"Position Error {error_dict['position_error']} m", "green", attrs=["bold"]))
                        else:
                            print(colored(f"Large Position Error {error_dict['position_error']} m", "red", attrs=["bold"]))

                elif qa_instance['type'] == 'binary':
                    num_binary += 1
                    if 'binary_iscorrect' in error_dict:
                        running_successes += error_dict['binary_iscorrect']
                        if error_dict['binary_iscorrect'] == 1:
                            sr_binary += 1
                            print(colored(f"Correct Binary Answer", "green", attrs=["bold"]))
                        else:
                            print(colored(f"Wrong Binary Answer", "red", attrs=["bold"]))
                elif qa_instance['type'] == 'time':
                    num_time += 1
                    if 'time_error' in error_dict:
                        running_time_error += error_dict['time_error']
                        if error_dict['time_error'] < 2.0:
                            sr_time += 1
                        else:
                            print(colored(f"Large Temporal Error {error_dict['time_error']} sec", "red", attrs=["bold"]))
                elif qa_instance['type'] == 'duration':
                    num_duration += 1
                    if 'duration_error' in error_dict:
                        running_duration_error += error_dict['duration_error']
                        if error_dict['duration_error'] < 2.0:
                            sr_duration += 1
                        else:
                            print(colored(f"Large Duration Error {error_dict['duration_error']} sec", "red", attrs=["bold"]))

                print("Question:", question)
                if 'response' in out_dict:
                    print("Response:", out_dict['response'])
                if num_binary > 0:
                    print("Avg Binary QA accuracy", running_successes/num_binary)
                    print(f"Success Rate: {sr_binary/num_binary}, {sr_binary}/{num_binary}", )
                if num_position > 0:
                    print("Avg Position Error", running_pos_error/num_position)
                    print(f"Success Rate: {sr_position/num_position}, {sr_position}/{num_position}", )
                if num_time > 0:
                    print("Temporal Error", running_time_error/num_time)
                    print(f"Success Rate: {sr_time/num_time}, {sr_time}/{num_time}")
                if num_duration > 0:
                    print("Duration Error", running_duration_error/num_duration)
                    print(f"Success Rate: {sr_duration/num_duration}, {sr_duration}/{num_duration}")

                # print("Running Binary QA accuracy", running_successes/(num_binary+1))
                # print("Running Spatial Error", running_pos_error/(num_position+1))
                # print("Running Temporal Error", running_time_error/(num_time+1))
                # print("Running Duration Error", running_duration_error/(num_duration+1))
                print()

                responses.append(out_dict)

            search_text = agent.memory.search_text
            for key, content in search_text.items():
                plot_data.append(content)
            search_time = agent.memory.search_time
            search_position = agent.memory.search_position
            plot_data.append(search_time)
            plot_data.append(search_position)
            search_SG = agent.memory.search_SG
            for key, content in search_SG.items():
                sg_data.append(content)

            out_path = os.path.join(args.out_dir, str(args.sequence_id), args.qa_file)
            os.makedirs(out_path, exist_ok=True)
            save_fig_path_db = os.path.join(args.out_dir, str(args.sequence_id), args.VDB, args.postfix)
            save_path = os.path.join(save_fig_path_db, f'retrieval_DB_{i}_{args.postfix}.png')
            print("Saved Retrieve status to", save_path)

            save_fig_path_sg = os.path.join(args.out_dir, str(args.sequence_id), args.SG, args.postfix)
            os.makedirs(save_fig_path_db, exist_ok=True)
            os.makedirs(save_fig_path_sg, exist_ok=True)
            # print(colored(f"DB: {plot_data}", "white", attrs=["dark"]))
            # print(colored(f"SG: {search_SG}", "white", attrs=["dark"]))

            if 'opengraph' in args.model:
                plot_scenegraph_scores(sg_data, gt_info=gt_info, save_path=os.path.join(save_fig_path_sg, f'scenegraph_{i}_{args.postfix}.png'))
            else:
                save_path = os.path.join(save_fig_path_db, f'retrieval_DB_{i}_{args.postfix}.png')
                plot_multi_method_scores(plot_data, k=args.topk, gt_info=gt_info, save_path=save_path)
            # plot_multi_method_scores(plot_data, gt_info=gt_info, save_path=os.path.join(args.out_dir, str(args.sequence_id), 'retrieval.png'))
            # plot_scenegraph_scores(search_SG, gt_info=gt_info, save_path=os.path.join(args.out_dir, str(args.sequence_id), 'scenegraph.png'))

            # save all_questions into json
            out_json = {
                "version": 0.1,
                "responses": responses
            }

            # save the outputs
            out_path = os.path.join(args.out_dir, str(args.sequence_id), args.qa_file)
            os.makedirs(out_path, exist_ok=True)

            name = args.model+'__'+args.caption_file+f"_{str(args.sequence_id)}_" +args.postfix
            with open(os.path.join(out_path, f'{name}.json'), 'w') as f:
                # to_save = json.dumps(out_json, indent=4)
                json.dump(out_json, f, indent=4)

            i += 1

def load_config_with_variables(file_path: str, max_iterations:int = 10) -> dict:
    with open(file_path, 'r') as f:
        content: dict = f.read()

    initial_config = yaml.safe_load(content)
    
    current_content: dict = content
    previous_content: dict = None
    iteration: int = 0
    while current_content != previous_content and iteration < max_iterations:
        previous_content = current_content
        template = Template(current_content)
        current_content = template.safe_substitute(initial_config)
        iteration += 1

    final_config: dict = yaml.safe_load(current_content)
    return final_config

@hydra.main(version_base=None, config_path="../configs", config_name="config")
def main(cfg: DictConfig):
    parser = argparse.ArgumentParser(
                        prog='Long Horizon Robot QA',
                        description='Runs various LLMs on the QA dataset',)

    cfg = cfg['inference']
    print("Loaded config:", cfg)

    # parser.add_argument("--model", type=str, default="remembr+llama3") #remembr+llama3
    # ChatGPT models: gpt-4o, gpt-4o-mini, gpt-4.1-mini, gpt-4o-mini; issue with gpt-4.1-nano
    parser.add_argument("--qa_file", type=str, default="human_qa")
    parser.add_argument("--log_file", type=str, default="test_log") #test_log
    parser.add_argument("--caption_file", type=str, default="captions_NVILA-8B") #captions_nvila
    parser.add_argument("--VDB", type=str, default="search_DB") #captions_nvila
    parser.add_argument("--SG", type=str, default="search_SG") #captions_nvila
    parser.add_argument("--coda_dir", type=str, default="./coda_data/")
    parser.add_argument("--data_dir", type=str, default="/workspace/results/")

    parser.add_argument("--scenegraph_file", type=str, default="full_pcd") # test_dam          seq: full_pcd_dam_{seq}

    parser.add_argument("--all_mem", type=bool, default=True, help="Whether to use the full memory. Default is True.")
    parser.add_argument("--evaluation_mode", type=bool, default=True, help="Whether to run in evaluation mode. Default is False.")
    parser.add_argument("--manual_evaluation", type=bool, default=True, help="Whether to run in manual evaluation mode. Default is False.")
    # Change this for EVAL
    # data-specific args
    parser.add_argument("--base_dir", type=str, default=cfg['base_path'])
    parser.add_argument("--sequence_id", type=int, default=cfg['sequence']) # 101
    parser.add_argument("--topk", type=int, default=6) # sg 10
    parser.add_argument("--latest_idx_path", type=str, default=cfg["latest_idx_path"])
    parser.add_argument("--instruction_path", type=str, default=cfg["instruction_path"])
    parser.add_argument("--out_dir", type=str, default="/workspace/results/") #out_time  out_descriptive_text  out_descriptive  out_spatial
    parser.add_argument("--model", type=str, default="remembr+gpt-4.1") #remembr+llama3  [vanila remembr opengraph][gpt-oss:20b gpt-4.1]
    parser.add_argument("--postfix", type=str, default='OpenMem_LONG_V19') # IB_cluster_MEDIUM3_grid IB_cluster   _long  _medium _short
    parser.add_argument("--categories", nargs="+", default=["text"], choices=["position","duration","time","binary","text"])
    parser.add_argument("--lengths", nargs="+", default=["LONG"], choices=["LONG","MEDIUM","SHORT"])

    # all model args
    # parser.add_argument("--use_gt_context", type=bool, default=False)

    # llm-specific argss
    parser.add_argument("--temperature", type=float, default=0.0)
    # parser.add_argument("--num_ctx", type=int, default=8192*8)
    parser.add_argument("--num_ctx", type=int, default=8192*4)

    # remembr specific args
    parser.add_argument("--window_size", type=int, default=2)
    parser.add_argument("--db_name", type=str, default='test')
    parser.add_argument("--db_ip", type=str, default='127.0.0.1')


    args = parser.parse_args()
    run_eval(args)


if __name__ == "__main__":
    main()
