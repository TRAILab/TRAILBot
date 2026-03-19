from dataclasses import dataclass, asdict

import re
import datetime, time
from time import strftime, localtime
from typing import Any, List, Optional, Tuple, Dict
from langchain_core.documents import Document
import numpy as np

import sys
sys.path.append("/home/mfyuan/local_folder/OpenNav_v2/remembr/")

from agentic_rag.star.memory.memory import Memory, MemoryItem

from langchain_community.vectorstores import Milvus
from langchain_huggingface import HuggingFaceEmbeddings

from pymilvus import connections, FieldSchema, CollectionSchema, DataType, Collection, utility
import torch
import torch.nn.functional as F
from termcolor import colored

# from remembr.utils.rerank_utils import should_rerank
# from remembr.memory.hybrid_reranker import deduplicate_and_rerank

import agentic_rag.star.clio_batch.helpers as helpers
from agentic_rag.star.clio_batch.ib_cluster import ClusterIB, ClusterIBConfig
from agentic_rag.star.clio_batch.aib_helper import cluster_task_scores, select_relevant_clusters, visualize_highlighted_clusters_open3d, visualize_graph_highlight, build_object_graph_smart

from agentic_rag.star.utils.world_map_isaacsim import ObjectVisualizer, visualize_objects, visualize_objects_org


FIXED_SUBTRACT=1721761000 # this is just a large value that brings us close to 1970

import math
import numpy as np
from pathlib import Path
import os

import networkx as nx
import torch
import open3d as o3d
from collections import Counter
import traceback
from agentic_rag.star.utils.utils import print_to_cot_log

# @torch.no_grad()
# def sbert_encode_texts(model, texts):
#     """Encode text prompts with SBERT and return L2-normalized numpy (M, D)."""
#     if isinstance(texts, str):
#         texts = [texts]
#     embs = model.encode(texts, convert_to_tensor=True)         # torch [M, D]
#     embs = torch.nn.functional.normalize(embs, dim=-1)
#     return embs.detach().cpu().numpy().astype(np.float32)      # np [M, D]


# def get_caption_features_from_objects(objects):
#     """Collect per-object SBERT embeddings into (N, D) numpy array."""
#     feats = []
#     for obj in objects:
#         ft = obj['ft']
#         if torch.is_tensor(ft):
#             ft = ft.detach().cpu().numpy()
#         feats.append(np.asarray(ft, dtype=np.float32))
#     return np.stack(feats, axis=0)  # (N, D)


# def patch_helpers_for_sbert():
#     """Patch Clio's helper to use cosine(task, region) on SBERT features."""
#     def _sbert_cosine_sim(task_feats, region_feats, *_unused, **__unused):
#         sims = task_feats @ region_feats.T   # (M, N), both L2-normalized
#         sims = np.clip(sims, 0.0, None)      # match Clio behavior (no negatives)
#         return sims
#     helpers.compute_sim_to_tasks = _sbert_cosine_sim


# ---- Helper: get absolute seconds for a caption doc ----
def _caption_time_seconds(doc, ref_time):
    """Return absolute seconds for a caption doc (handles float or [t, ...])."""
    mt = doc.metadata.get('time', 0.0)
    t = mt[0] if isinstance(mt, (list, tuple)) else mt
    return float(t + (ref_time or 0.0))

def _summarize_omitted_times(docs, ref_time, max_list=6):
    """Make a compact string listing omitted caption times and span."""
    if not docs:
        return ""
    ts = sorted(_caption_time_seconds(d, ref_time) for d in docs)
    # Pretty list (limit)
    shown = ts[:max_list]
    more = len(ts) - len(shown)
    # Span
    span = ts[-1] - ts[0] if len(ts) > 1 else 0.0
    # Render
    from time import localtime, strftime
    def fmt(sec): return strftime('%Y-%m-%d %H:%M:%S', localtime(sec))
    shown_str = ", ".join(fmt(s) for s in shown)
    # if more > 0:
    #     shown_str += f", +{more} more"
    return f"The scene area was also observed at the following times, {shown_str}. Therefore, do not select timestamps within this range repeatedly."


def _to_unit(x):
    if isinstance(x, np.ndarray):
        x = torch.from_numpy(x)
    x = x.float()
    if x.ndim == 1: x = x[None, :]
    return x / (x.norm(dim=-1, keepdim=True) + 1e-12)

def cluster_task_scores_cosine(objects, clusters, task_features, pool="max"):
    """
    objects: list[dict] each with 'ft' tensor/ndarray
    clusters: list[list[int]] (indices into `objects`)
    task_features: (T,D) numpy/torch (already SBERT)
    pool: 'max' (best member), 'mean', or 'topk-mean:k'
    returns: scores (C,), winners (C,) best task per cluster
    """
    T = _to_unit(task_features)  # (T,D)
    scores, winners = [], []

    for members in clusters:
        if not members:
            scores.append(0.0); winners.append(-1); continue
        F = _to_unit(torch.stack([_to_unit(objects[i]['ft']).squeeze(0) for i in members]))  # (M,D)
        S = (T @ F.T)  # (T, M)

        if pool == "mean":
            v = S.mean(dim=1)                  # (T,)
        elif pool.startswith("topk-mean:"):
            k = int(pool.split(":")[1])
            vals, _ = torch.topk(S, k=min(k, S.shape[1]), dim=1)
            v = vals.mean(dim=1)               # (T,)
        else:  # 'max'
            v, _ = S.max(dim=1)                # (T,)

        best_val, best_task = torch.max(v, dim=0)
        scores.append(float(best_val.item()))
        winners.append(int(best_task.item()))
    return np.array(scores), np.array(winners)


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

def _vertical_overlap_mask_dele(
    mn, mx, v_ax, use_overlap=True, slack=0.25, inclusive=False, eps=1e-8
):
    """
    返回 (N,N) 的上三角布尔掩码，描述在垂直轴上的“相容性”：
      - use_overlap=True  : 要求严格（或可选包含相切）的高度区间重叠
      - use_overlap=False : 允许两区间之间的空隙 gap <= slack
    参数:
      inclusive : True 则把相切也视为“重叠”
      eps       : 数值稳定用
    """
    # 取出两两组合的垂直最小/最大
    vmin  = mn[:, v_ax][:, None]       # (N,1)
    vmax  = mx[:, v_ax][:, None]       # (N,1)
    vmin2 = mn[:, v_ax][None, :]       # (1,N)
    vmax2 = mx[:, v_ax][None, :]       # (1,N)

    # 一维区间的重叠长度（>0 表示有交叠；<0 表示相离，数值为负的间隙）
    inter_v = torch.minimum(vmax, vmax2) - torch.maximum(vmin, vmin2)

    if use_overlap:
        # 严格重叠 vs. 含相切
        if inclusive:
            M = (inter_v >= -eps)
        else:
            M = (inter_v > eps)
    else:
        # 用“间隙”判定：gap = max(0, -inter_v)，允许 gap <= slack
        gap = torch.clamp(-inter_v, min=0)
        M = (gap <= slack + eps)

    return torch.triu(M, diagonal=1)


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
    dist_radius=3.0,              # meters
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
    gate = iou_mask & v_mask  #dist_mask & v_mask & iou_mask & covis_mask & ground_mask

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
            "delta: 0.20\n"               # higher => more merging
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


# import numpy as np

def filter_clusters_by_task_topk(clustered_objects, task_features, k=6):
    """Keep top-k clusters whose embedding is most similar to the tasks."""
    # Compute similarities for each object
    sims = []
    for obj in clustered_objects:
        s = cosine_max_to_tasks(task_features, obj['ft'])
        sims.append((s, obj))

    # Sort by similarity (descending) and pick top k
    sims.sort(key=lambda x: x[0], reverse=True)
    kept = [obj for _, obj in sims[:k]]

    return kept


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


def _encode_query_sbert(sbert_model, query: str):
    q = sbert_model.encode(query, convert_to_tensor=True)
    q = q / q.norm(dim=-1, keepdim=True)
    return q.squeeze()

def _parse_object_ids(meta_val):
    """
    meta_val can be:
      - a comma-separated string: '2225,2240,...'
      - a list[int] or list[str]
    Returns: List[int]
    """
    if meta_val is None:
        return []
    if isinstance(meta_val, str):
        return [int(x) for x in meta_val.split(',') if x.strip() != ""]
    if isinstance(meta_val, (list, tuple)):
        out = []
        for v in meta_val:
            try:
                out.append(int(v))
            except Exception:
                pass
        return out
    return []

def _global_ids_to_local_indices(global_ids, objid_to_idx=None):
    """
    Map global object ids to indices into self.objects.
    If objid_to_idx is None we assume objects are stored at index == global_id.
    """
    if objid_to_idx is None:
        return [gid for gid in global_ids if 0 <= gid]
    idxs = []
    for gid in global_ids:
        if gid in objid_to_idx:
            idxs.append(objid_to_idx[gid])
    return idxs

def _build_subgraph_from_indices_without_BG(
    objects,
    object_cap_features_np,
    idxs,
    graph_builder_kwargs,
    *,
    # --- new optional args for background filtering ---
    sbert_model=None,                 # SentenceTransformer model; required if you pass background_terms and want encoding fallback
    background_terms=None,            # e.g., ["wall", "ground", "staging area", "ceiling", "floor", ...]
    bg_threshold=0.45,                # cosine similarity threshold to treat an object as background
    background_mat=None,              # (optional) precomputed torch.Tensor [B, D] for background_terms to avoid recompute
    use_caption_fallback=True,        # if an object has no 'ft', encode its 'caption' with sbert_model
    verbose=True
):
    """
    Build a subgraph from selected indices while filtering out background-like objects
    using semantic similarity. If background_terms/background_mat are not provided,
    the function behaves exactly like the original one (no filtering).

    Filtering rule:
      - For each candidate object i in idxs, get a unit-norm embedding v_i.
      - Compute max cosine similarity with any background embedding.
      - If max_sim >= bg_threshold, the object is considered background and skipped.

    Notes:
      - Preferred per-object feature keys (first available): 'ft' -> 'captions_ft' -> encode('caption')
      - Falls back to object_cap_features_np[i] if none of the above available.
    """
    import numpy as np
    import torch
    from datetime import datetime

    def _to_unit_tensor(x):
        """Convert numpy/torch vector to 1D unit-norm torch tensor (on CPU)."""
        if x is None:
            return None
        if isinstance(x, np.ndarray):
            t = torch.from_numpy(x.astype(np.float32))
        elif torch.is_tensor(x):
            t = x.detach()
        else:
            return None
        if t.ndim > 1:
            t = t.view(-1)
        t = t.cpu()
        denom = torch.norm(t, p=2) + 1e-12
        return t / denom

    # ---- prepare background embedding matrix ----
    bg_mat = None
    if background_mat is not None:
        # user provided a precomputed [B, D] matrix (should already be unit-normalized)
        bg_mat = background_mat.detach().cpu()
    elif background_terms:
        if sbert_model is None and use_caption_fallback:
            raise ValueError("sbert_model is required to encode background_terms.")
        embs = []
        for term in background_terms:
            try:
                e = _encode_query_sbert(sbert_model, term)  # returns unit-norm tensor
                embs.append(e.detach().cpu().view(-1))
            except Exception as ex:
                if verbose:
                    print(f"[BG-ENC] Failed to encode '{term}': {ex}")
        if len(embs) > 0:
            bg_mat = torch.stack(embs, dim=0)  # [B, D]
        else:
            if verbose:
                print("[BG-ENC] No valid background embeddings; filtering disabled.")
            bg_mat = None
    # else: no background_terms and no background_mat -> filtering disabled

    # ---- collect filtered sub-objects and aligned features ----
    sub_objects = []
    kept_idxs = []

    for i in idxs:
        if not (0 <= i < len(objects)):
            if verbose:
                print(colored(f"Warning: index {i} is out of bounds [0, {len(objects)-1}]", "red"))
            continue

        obj = objects[i]

        '''text_query_ft = self.sbert_model.encode([query], convert_to_tensor=True)
        text_query_ft = text_query_ft / text_query_ft.norm(dim=-1, keepdim=True)
        top_k_scene = self.args.topk
        scored_objects = []
        for obj in self.scene_graph:
            if 'ft' not in obj or obj['ft'] is None:
                print(f"Object {obj.get('id', 'unknown')} does not have ft, skipping.")
                continue

            obj_ft = torch.tensor(obj['ft'], device=text_query_ft.device)
            obj_ft = obj_ft / obj_ft.norm(dim=-1, keepdim=True)

            score = F.cosine_similarity(text_query_ft, obj_ft.unsqueeze(0), dim=-1).item()
            scored_objects.append((obj, score))

        scored_objects.sort(key=lambda x: -x[1])'''
        # 1) pick the best available semantic feature for background check
        cand_feat = obj.get('ft', None)
        if cand_feat is None:
            cand_feat = obj.get('captions_ft', None)
            print(f"Object {obj.get('id', 'unknown')} does not have ft, using captions_ft instead.")

        v = _to_unit_tensor(cand_feat)  # unit vector or None

        # 2) decide keep/skip
        keep = True
        if bg_mat is not None and v is not None:
            # cosine similarity = dot since both are unit-norm
            sims = torch.matmul(bg_mat, v)  # [B]
            max_sim = float(torch.max(sims).item())
            if max_sim >= bg_threshold:
                keep = False
                print(colored(f"[BG-FILTER] drop idx={obj['caption']} (max_sim={max_sim:.3f} ≥ {bg_threshold})", "yellow"))

        # Optional: if your data already tags background explicitly, you can also skip by flag
        # if obj.get("bg_class", None) is not None:
        #     keep = False
        if keep:
            sub_objects.append(obj)
            kept_idxs.append(i)

    if len(kept_idxs) == 0:
        if verbose:
            print("[BG-FILTER] No objects left after filtering.")
        # still build an empty graph to keep the contract (or return None as you prefer)
        sub_feats = object_cap_features_np[np.array([], dtype=int)]
        G_sub = build_object_graph_ground_safe([], sub_feats, **graph_builder_kwargs)
        return G_sub, [], sub_feats

    # keep feature rows aligned with kept objects
    kept_idxs_np = np.array(kept_idxs, dtype=int)
    sub_feats = object_cap_features_np[kept_idxs_np]

    # build graph on the filtered subset
    G_sub = build_object_graph_ground_safe(sub_objects, sub_feats, **graph_builder_kwargs)
    return G_sub, sub_objects, sub_feats


def _build_subgraph_from_indices(objects, object_cap_features_np, idxs, graph_builder_kwargs):
    sub_objects = []
    new_idxs = []
    for i in idxs:
        if 0 <= i < len(objects):
            L, W, H = objects[i]['bbox'].extent
            if L * W < 7.0 and L < 2.50 and H <2.50:
                sub_objects.append(objects[i])
                new_idxs.append(i)
            else:
                print(colored(f"Filtered objects {objects[i]['caption']} {objects[i]['bbox'].extent}", "blue"))
        else:
            print(colored(f"Warning: index {i} is out of bounds [0, {len(objects)-1}]", "red"))
    # sub_objects = [objects[i] for i in idxs]
    # sub_feats   = object_cap_features_np[idxs]
    sub_feats   = object_cap_features_np[np.array(new_idxs, dtype=int)]

    G_sub = build_object_graph_ground_safe(sub_objects, sub_feats, **graph_builder_kwargs)
    return G_sub, sub_objects, sub_feats

def _time_to_frame_idx(t, time_offset, fps, stride=1):
    """
    Map absolute caption time `t` to a scene-graph frame index space.
    - time_offset: beginning-of-memory time (same origin for caps and SG)
    - fps: frames per second of the SG (or 1/period for keyframes)
    - stride: if you only kept every `stride`-th frame in SG objects['image_idx']
    """
    print(colored(f"Mapping caption time {t} with offset {time_offset}, fps {fps}, stride {stride}", "red"))
    rel = max(0.0, float(time_offset +t))
    frame = int(round(rel * fps))
    if stride > 1:
        frame //= int(stride)
    return frame

def _collect_obj_ids_in_time_window(objects, frame_idx, window_frames=3):
    """
    Return set of object indices observed in [f - w, f + w].
    Assumes objects[i]['image_idx'] is a list of frame indices.
    """
    lo, hi = frame_idx - window_frames, frame_idx + window_frames
    keep = []
    for oid, obj in enumerate(objects):
        idxs = obj.get('image_idx', [])
        if any((lo <= f <= hi) for f in idxs):
            keep.append(oid)
    return keep

def _subselect_features(features_np, idxs):
    # features_np: (N, D) numpy (SBERT/CLIP features for objects)
    return features_np[idxs]

def _build_subgraph(objects, object_cap_features, idxs, graph_builder_kwargs):
    # Reuse your Y-down safe builder, but pass only subset `idxs`.
    sub_objects = [objects[i] for i in idxs]
    sub_feats   = _subselect_features(object_cap_features, np.array(idxs))
    G_sub = build_object_graph_ground_safe(sub_objects, sub_feats, **graph_builder_kwargs)
    return G_sub, sub_objects, sub_feats

def _run_ib_subset(object_cap_features, task_features, G_nx, cfg_yaml_path):
    # thin wrapper to your existing IB entrypoint
    return run_ib_clustering(object_cap_features, task_features, G_nx, cfg_yaml_path)

def _group_caps_by_cluster(caps, cap_scores, cap_to_obj_ids, clusters, subset_obj_global_ids):
    """
    caps: list of retrieved docs (LangChain Document)
    cap_scores: parallel list of float scores (lower distance → better)
    cap_to_obj_ids: list[ set[int] ] objects touched by each caption (GLOBAL ids)
    clusters: list[list[int]] cluster members (subset index space)
    subset_obj_global_ids: list[int] mapping subset index → global object id

    Returns: dict cluster_id -> list of (doc_idx, score)
    """
    # Map global object id → cluster id
    obj_global_to_cluster = {}
    for cid, member_subset_idxs in enumerate(clusters):
        for sub_idx in member_subset_idxs:
            g = subset_obj_global_ids[sub_idx]
            obj_global_to_cluster[g] = cid

    groups = {}
    for i, obj_ids in enumerate(cap_to_obj_ids):
        # a caption may touch multiple objs → assign to the most frequent cluster hit
        cluster_hits = [obj_global_to_cluster.get(gid, None) for gid in obj_ids]
        cluster_hits = [h for h in cluster_hits if h is not None]
        if not cluster_hits:
            continue
        # pick the dominant cluster (or just the first)
        cid = max(set(cluster_hits), key=cluster_hits.count)
        groups.setdefault(cid, []).append((i, cap_scores[i]))
    return groups

def _pick_best_per_group(groups, caps, cap_scores, k):
    """
    Select at most one caption per group (lowest distance score is best), up to k groups.
    If there are more groups than k, choose the k groups with the best representatives.
    """
    representatives = []
    for cid, items in groups.items():
        # items: list[(doc_idx, score)] where score is distance/sim metric from vectorstore
        # lower score often means more similar (depending on store). If yours is "similarity",
        # flip the sign accordingly.
        best_idx, best_score = min(items, key=lambda t: t[1])
        representatives.append((cid, best_idx, best_score))

    # sort groups by their best item
    representatives.sort(key=lambda t: t[2])
    representatives = representatives[:k]
    return representatives


class MilvusWrapper:

    def __init__(self, collection_name='test', ip_address='127.0.0.1', port=19530, drop_collection=False):
        self.collection_name = collection_name
        self.collection = self.connect_to_milvus_collection(collection_name, 1024, address=ip_address, port=port, drop_collection=drop_collection)


    def drop_collection(self):
        utility.drop_collection(self.collection_name)

    def connect_to_milvus_collection(self, collection_name, dim, address='127.0.0.1', port=19530, drop_collection=False):
        connections.connect(host=address, port=port)

        if drop_collection:
            utility.drop_collection(collection_name)

        fields = [
            FieldSchema(name='id', dtype=DataType.VARCHAR, description='ids', is_primary=True, auto_id=False, max_length=1000),
            FieldSchema(name='text_embedding', dtype=DataType.FLOAT_VECTOR, description='embedding vectors', dim=dim),
            FieldSchema(name='position', dtype=DataType.FLOAT_VECTOR, description='position of robot', dim=3),
            FieldSchema(name='theta', dtype=DataType.FLOAT, description='rotation of robot', dim=1),
            FieldSchema(name='time', dtype=DataType.FLOAT_VECTOR, description='time', dim=2),
            FieldSchema(name='caption', dtype=DataType.VARCHAR, description='caption string', max_length=3000),
            FieldSchema(name='object_id', dtype=DataType.VARCHAR, description='object id', max_length=3000),

        ]
        schema = CollectionSchema(fields=fields, description='text image search')
        collection = Collection(name=collection_name, schema=schema)

        # create IVF_FLAT index for collection.
        index_params = {
            'metric_type':'L2',
            'index_type':"IVF_FLAT",
            'params':{"nlist":1024}
        }
        collection.create_index(field_name="text_embedding", index_params=index_params)

        index_params = {
            'metric_type':'L2',
            'index_type':"IVF_FLAT",
            'params':{"nlist":2}
        }
        collection.create_index(field_name="position", index_params=index_params)

        index_params = {
            'metric_type':'L2',
            'index_type':"IVF_FLAT",
            'params':{"nlist":2}
        }
        collection.create_index(field_name="time", index_params=index_params)

        return collection

    def insert(self, data_list):
        res = self.collection.insert(data_list)

    def search(self, data):

        self.collection.load()

        BATCH_SIZE = 2
        LIMIT = 10

        param = {
            "metric_type": "L2",
            "params": {
                "nprobe": 1024,
            }
        }

        res = self.collection.search(
            data=[data],
            anns_field="text_embedding",
            param=param,
            batch_size=BATCH_SIZE,
            limit=LIMIT,
            # expr="id > 3",
            output_fields=["id", "text_embedding"]
        )

        return res




class MilvusMemory(Memory):
    def __init__(self, db_collection_name: str, db_ip='127.0.0.1', db_port=19530, time_offset=FIXED_SUBTRACT, embedder=None, args=None):
        self.db_collection_name = db_collection_name
        self.db_ip = db_ip
        self.db_port = db_port
        self.time_offset = time_offset
        self.args = args
        self.cot_log_file = None

        self.embedder = embedder[0] or HuggingFaceEmbeddings(model_name='mixedbread-ai/mxbai-embed-large-v1')
        self.sbert_model = embedder[1]
        self.working_memory = []

        self.reset(drop_collection=False)
        self.scene_graph = None
        self.search_position = {
            "method_name": "position_search",
            "data": []
        }
        self.search_time = {
            "method_name": "time_search",
            "data": []
        }
        self.search_text = {}
        # self.search_text = {
        #     "method_name": "text_search",
        #     "data": []
        # }
        # self.search_SG = {
        #     "method_name": "scenegraph_search",
        #     "data": []
        # }
        self.search_SG = {}

    def insert(self, item: MemoryItem, text_embedding=None):
        # Convert the dataclass item to a dictionary
        memory_dict = asdict(item)
        # Assign a unique ID based on current timestamp
        memory_dict['id'] = str(time.time())
        # print("memory dict: ", memory_dict)
        # If no embedding is provided, compute one using the embedder
        if text_embedding is None:
            text_embedding = self.embedder.embed_query(memory_dict['caption'])
            print("text embedding is none, therefore generate it online")
        # Adjust the timestamp by a time offset (e.g. the start time of the current memory collection)
        memory_dict['time'] =  [(memory_dict['time'] - self.time_offset), 0.0]

        memory_dict['text_embedding'] = text_embedding

        self.milv_wrapper.insert([memory_dict])

    def get_working_memory(self) -> list[MemoryItem]:
        return self.working_memory

    def reset(self, drop_collection=True):

        if drop_collection:
            print("Resetting memory. We are dropping the current collection")

        self.milv_wrapper = MilvusWrapper(self.db_collection_name, self.db_ip, self.db_port, drop_collection=drop_collection)

        text_vector_db = Milvus(
            self.embedder,
            connection_args={"host": self.db_ip, "port": self.db_port},
            collection_name=self.db_collection_name,
            vector_field='text_embedding',
            text_field='caption',
        )

        self.text_retriever = text_vector_db.as_retriever(search_kwargs={"k": 5})


        self.position_vector_db = Milvus(
            self.embedder, # we will ignore this
            connection_args={"host": self.db_ip, "port": self.db_port},
            collection_name=self.db_collection_name,
            vector_field='position',
            text_field='caption',
        )

        self.time_vector_db = Milvus(
            self.embedder, # we will ignore this
            connection_args={"host": self.db_ip, "port": self.db_port},
            collection_name=self.db_collection_name,
            vector_field='time',
            text_field='caption',
        )

    def search_by_position(self, query: tuple) -> str:
        docs_with_scores = similarity_search_with_score_by_vector(
            self.position_vector_db, np.array(query).astype(float)
        )
        self.working_memory += [doc for doc, _ in docs_with_scores]

        # extract the time and score from the documents
        data = []
        for doc, score in docs_with_scores:
            t = doc.metadata['time']
            data.append({
                "time": t + self.time_offset,
                "score": score,
            })
        self.search_position = {
            "method_name": "position_search",
            "data": data
        }

        docs_str = self.memory_to_string([doc for doc, _ in docs_with_scores])
        # print("docs for Position search: ", docs_str)
        print_to_cot_log(message=f"docs for Position search: {docs_str}", cot_log_file=self.cot_log_file)
        return docs_str


    # def search_by_position(self, query: tuple) -> str:
    #     # docs = pos_db.similarity_search_by_vector(np.array(query))
    #     docs = similarity_search_with_score_by_vector(self.position_vector_db, np.array(query).astype(float))

    #     self.working_memory += docs

    #     docs = self.memory_to_string(docs)
    #     print("docs for Time search: ", docs)

    #     """Look up things online."""

    #     return docs

    # def search_by_time(self, hms_time: str) -> str:

    #     # Input is time like 08:20:30
    #     # need to convert to searchable time
    #     t = localtime(self.time_offset)
    #     mdy_date = strftime('%m/%d/%Y', t)
    #     template = "%m/%d/%Y %H:%M:%S"

    #     # if the hms_time is already in the mdy hms format without me doing anything, let's just use that.
    #     # bad llms don't listen :(
    #     try:
    #         res = bool(datetime.datetime.strptime(hms_time, template))
    #     except ValueError:
    #         res = False

    #     hms_time = hms_time.strip()
    #     if not res: # convert to the right format then
    #         hms_time = mdy_date + ' ' + hms_time

    #     query = time.mktime(datetime.datetime.strptime(hms_time,template).timetuple()) - self.time_offset
    #     # convert from hms_time to something searchable


    #     docs = similarity_search_with_score_by_vector(self.time_vector_db, np.array([query, 0]))

    #     self.working_memory += docs

    #     docs = self.memory_to_string(docs)
    #     # np.unique([doc.metadata['time'][0] for doc in docs])
    #     """Look up things online."""
    #     print("docs for time search: ", docs)
    #     return docs

    def search_by_time(self, hms_time: str) -> str:
        t = localtime(self.time_offset)
        mdy_date = strftime('%m/%d/%Y', t)
        template = "%m/%d/%Y %H:%M:%S"

        try:
            res = bool(datetime.datetime.strptime(hms_time, template))
        except ValueError:
            res = False

        hms_time = hms_time.strip()
        if not res:
            hms_time = mdy_date + ' ' + hms_time

        query = time.mktime(datetime.datetime.strptime(hms_time, template).timetuple()) - self.time_offset

        docs_with_scores = similarity_search_with_score_by_vector(
            self.time_vector_db, np.array([query, 0])
        )

        self.working_memory += [doc for doc, _ in docs_with_scores]

        # Extract data for plotting
        data = []
        for doc, score in docs_with_scores:
            t = doc.metadata['time']
            data.append({
                "time": t + self.time_offset,
                "score": score,
            })
        self.search_time = {
            "method_name": "time_search",
            "data": data
        }

        docs_str = self.memory_to_string([doc for doc, _ in docs_with_scores])
        # print("docs for time search: ", docs_str)
        print_to_cot_log(message=f"docs for time search: {docs_str}", cot_log_file=self.cot_log_file)
        return docs_str


    # def search_by_text(self, query: str, k =5) -> str:

    #     docs = self.text_retriever.invoke(query, k=k)
    #     # docs_with_scores = self.text_retriever.vectorstore.similarity_search_with_score(query, k)

    #     self.working_memory += docs

    #     docs = self.memory_to_string(docs)

    #     """Look up things online."""
    #     return docs

    # def search_by_text(self, query: str, k=5) -> str:
    #     # docs_with_scores = self.text_retriever.vectorstore.similarity_search_with_score(query, k=k)

    #     # self.working_memory += [doc for doc, _ in docs_with_scores]
    #     docs_with_scores = self.text_retriever.vectorstore.similarity_search_with_score(query, k=50)

    #     # ✅ Only add top-k to working memory
    #     self.working_memory += [doc for doc, _ in docs_with_scores[:k]]
    #     # Extract data for plotting
    #     data = []
    #     for doc, score in docs_with_scores:
    #         t = doc.metadata['time']
    #         data.append({
    #             "time": t + self.time_offset,
    #             "score": score,
    #         })
    #     self.latest_plot_data = {
    #         "method_name": "text_search",
    #         "data": data
    #     }

    #     docs_str = self.memory_to_string([doc for doc, _ in docs_with_scores])
    #     return docs_str
    def search_by_text_org(self, query: str, k=12) -> str:
        # Retrieve more results (e.g., top-50) to inspect the overall distribution,
        # but only add the top-k results to the working memory for downstream tasks.
        docs_with_scores = self.text_retriever.vectorstore.similarity_search_with_score(query, k=100)
        k = self.args.topk

        # ✅ Add only the top-k documents to the working memory.
        self.working_memory += [doc for doc, _ in docs_with_scores[:k]]

        # ✅ Extract data for plotting.
        data = []
        scores = []
        for doc, score in docs_with_scores:
            t = doc.metadata['time']
            data.append({
                "time": t + self.time_offset,  # Shift time by the offset for real-time alignment.
                "score": score,
            })
            scores.append(score)

        # ✅ Normalize scores between 0 and 1.
        min_score = min(scores)
        max_score = max(scores)
        if max_score - min_score == 0:
            normalized_scores = [0.0 for _ in scores]
        else:
            normalized_scores = [(s - min_score) / (max_score - min_score) for s in scores]

        # ✅ Add normalized scores to the plot data.
        for i in range(len(data)):
            data[i]['score_normalized'] = normalized_scores[i]

        # ✅ Store the plot data for later visualization.
        # self.search_text = {
        #     "method_name": f"{query}",
        #     "data": data,
        #     "score_min": min_score,
        #     "score_max": max_score,
        # }
        if not hasattr(self, 'search_text'):
            self.search_text = {}


        self.search_text[query] = {
        "method_name": f"cue: {query}",
        "data": data,
        "score_min": min_score,
        "score_max": max_score,
        }
        # ✅ Return string representation of the top-k results for downstream use.
        docs_str = self.memory_to_string([doc for doc, _ in docs_with_scores[:k]])
        return docs_str

    def search_by_text_IB_old(self, query: str, k=6) -> str:
        """
        Retrieve many captions, prefer task reasoning based on *video captions* with higher scores,
        group top-ranked captions that describe the same cluster, and include lower-ranked captions
        that describe *different* clusters to ensure diversity. Rank AIB clusters by the *best*
        linked video-caption score and keep only top-N clusters.
        """
        try:
            # ---------- Tunables ----------
            TOP_M        = 100     # retrieve a big pool
            KEEP_THRESH  = 0.4     # normalized similarity threshold to keep captions
            # NOTE: We keep AIB clustering, but cluster *ranking* is now driven by video-caption scores
            # instead of object-text similarity to the task (SBERT). You can still keep a very low filter if desired.
            TASK_FILTER_THRESH = 0.0   # CHANGED: effectively disable (or set small value) to not drop useful clusters early

            # NEW: how many clusters to keep after ranking them by best video-caption score
            KEEP_TOP_CLUSTERS = max(3, min(6, k))   # heuristic: keep between 3 and 6 clusters, capped by k

            # NEW: per-cluster cap of (temporally separated) captions we’re willing to consider
            AUTO_MAX_PER_CLUSTER = 3
            MIN_PER_CLUSTER      = 1
            TIME_SEP             = 90.0  # seconds, ensure temporal diversity within a cluster

            GRAPH_KW = dict(
                vertical_axis='y', down_positive=True,
                dist_radius=3.0, z_overlap=False, z_slack=0.40,
                iou_thresh=0.01, covis_min=0, knn=6,
                ground_height_thresh=0.1, ground_floor_thresh=0.05,
                dilate_eps=0.02
            )

            # ---------- 1) Retrieve ----------
            docs_with_scores = self.text_retriever.vectorstore.similarity_search_with_score(query, k=TOP_M)
            if not docs_with_scores:
                print(colored("Warning: No captions found for the query.", "red"))
                return ""

            # Normalize scores to [0,1] as SIMILARITY (higher is better).
            # If the vectorstore returns DISTANCE (smaller=better), convert → similarity.
            raw_scores = [s for _, s in docs_with_scores]
            smin, smax = min(raw_scores), max(raw_scores)
            if smax - smin < 1e-9:
                norm_topM = [1.0 for _ in raw_scores]
            else:
                norm_topM = [1.0 - (s - smin) / (smax - smin) for s in raw_scores]
            #print(colored(f"Normalized scores (top-M): {norm_topM}", "yellow"))

            # Keep by normalized SIMILARITY
            keep_idx = [i for i, v in enumerate(norm_topM) if v >= KEEP_THRESH]
            print(colored(f"Keeping {len(keep_idx)} / {len(norm_topM)} captions (>= {KEEP_THRESH:.2f})", "yellow"))
            if not keep_idx:
                keep_idx = list(range(min(TOP_M, len(docs_with_scores))))

            # Kept pools (aligned by kept index space)
            kept_docs      = [docs_with_scores[i][0] for i in keep_idx]
            kept_rawscore  = [docs_with_scores[i][1] for i in keep_idx]       # raw from store (likely distance)
            kept_normscore = [norm_topM[i] for i in keep_idx]                 # similarity in [0,1]

            # ---------- 2) Build subset of objects from object_id ----------
            cap_to_local_obj_ids = []
            subset_local_ids_set = set()
            for doc in kept_docs:
                gids = _parse_object_ids(doc.metadata.get('object_id'))
                lids = _global_ids_to_local_indices(gids, objid_to_idx=getattr(self, 'objid_to_idx', None))
                lids = list(set(lids))
                cap_to_local_obj_ids.append(lids)
                subset_local_ids_set.update(lids)

            subset_local_ids = sorted(list(subset_local_ids_set))
            print(colored(f"Subset has {subset_local_ids} unique objects mentioned by captions", "blue"))
            if len(subset_local_ids) == 0:
                print(colored("No objects covered by captions; falling back to top-k plain retrieval.", "red"))
                # Prefer top-k by highest similarity (norm_topM) for a better default
                order = np.argsort([-norm for norm in norm_topM])[:k]
                topk_docs = [docs_with_scores[i][0] for i in order]
                self.working_memory += topk_docs
                return self.memory_to_string(topk_docs)

            # ---------- 3) Subgraph + AIB on subset ----------
            object_cap_features_np = np.array([obj['ft'].flatten() for obj in self.scene_graph])  # (N,D)
            print(colored(f"Building subgraph from {len(subset_local_ids)} objects", "blue"))
            G_sub, sub_objects, sub_feats = _build_subgraph_from_indices(
                self.scene_graph, object_cap_features_np, subset_local_ids, GRAPH_KW
            )

            # Task feature (SBERT) – still used for embeddings/optionally for light filtering & viz
            task_ft = _encode_query_sbert(self.sbert_model, query).detach().cpu().numpy()[None, :]

            # IB config
            out_path = getattr(self, 'out_dir', './_tmp')
            Path(out_path).mkdir(parents=True, exist_ok=True)
            cfg_yaml_path = os.path.join(out_path, "cluster_config.yaml")
            write_default_cluster_config(cfg_yaml_path)

            # Run AIB to get clusters over subset positions
            clusters_subset = run_ib_clustering(sub_feats, task_ft, G_sub, cfg_yaml_path)
            print(colored(f"[AIB] Formed {len(clusters_subset)} clusters from {len(subset_local_ids)} primitives.", "cyan"))

            # (Optional) very light task relevance filter – keep permissive
            clustered_objects = merge_all_clusters(sub_objects, clusters_subset)
            if TASK_FILTER_THRESH > 0:
                clustered_objects_filtered = filter_clusters_by_task(clustered_objects, task_ft, TASK_FILTER_THRESH)
                print(colored(f"[AIB] Filtered to {len(clustered_objects_filtered)} task-relevant objects.", "cyan"))

            # Also compute scores/winners for visualization only (not used for ranking anymore)
            scores_task, winners = cluster_task_scores_cosine(sub_objects, clusters_subset, task_ft, pool="max")

            # ---------- 4) Map captions → dominant cluster & score clusters by *caption similarity* ----------
            # Build: subset local-id -> position in subset
            subset_local_to_pos = {lid: pos for pos, lid in enumerate(subset_local_ids)}
            # Map subset-position -> cluster id
            objpos_to_cid = {}
            for cid, members in enumerate(clusters_subset):
                for pos in members:
                    objpos_to_cid[pos] = cid

            # Group kept captions by their *dominant* cluster; collect kept_normscore for ranking
            groups = {}  # cid -> list[(cap_kept_idx, kept_normscore)]
            for cap_i, lids in enumerate(cap_to_local_obj_ids):
                hits = []
                for lid in lids:
                    pos = subset_local_to_pos.get(lid, None)
                    if pos is None:
                        continue
                    cid = objpos_to_cid.get(pos, None)
                    if cid is not None:
                        hits.append(int(cid))
                if not hits:
                    continue
                dom = max(set(hits), key=hits.count)
                groups.setdefault(dom, []).append((cap_i, kept_normscore[cap_i]))

            if not groups:
                print(colored("No captions map to clusters; falling back to top-k by similarity.", "red"))
                order = np.argsort([-norm for norm in norm_topM])[:k]
                docs = [docs_with_scores[i][0] for i in order]
                self.working_memory += docs
                return self.memory_to_string(docs)

            # NEW: Rank clusters by the *best* (highest) video-caption similarity linked to them
            cluster_best_sim = []
            for cid, items in groups.items():
                best = max(sim for _, sim in items)
                cluster_best_sim.append((cid, best))
            cluster_best_sim.sort(key=lambda t: t[1], reverse=True)

            # Keep only the top-N clusters by caption similarity
            keep_cluster_ids_sorted = [cid for cid, _ in cluster_best_sim[:KEEP_TOP_CLUSTERS]]
            print(colored(f"Keeping top {len(keep_cluster_ids_sorted)} clusters by video-caption score", "green"))

            # ---------- 5) Within each kept cluster, select temporally diverse top captions ----------
            ref_time = getattr(self, 'time_offset', 0.0)
            per_cluster_chosen = {}  # cid -> list[(cid, cap_kept_i, cap_sim, t_abs, omitted_summary)]
            for cid in keep_cluster_ids_sorted:
                items = groups.get(cid, [])
                if not items:
                    continue

                # Sort candidates by similarity DESC (higher is better)
                items_sorted = sorted(items, key=lambda t: t[1], reverse=True)

                chosen = []
                chosen_times = []
                chosen_docs = []
                for cap_i, sim in items_sorted:
                    doc = kept_docs[cap_i]
                    t_abs = _caption_time_seconds(doc, ref_time)
                    if all(abs(t_abs - prev) >= TIME_SEP for prev in chosen_times):
                        chosen.append((cid, cap_i, sim, t_abs))
                        chosen_times.append(t_abs)
                        chosen_docs.append(doc)
                    if len(chosen) >= AUTO_MAX_PER_CLUSTER:
                        break

                if len(chosen) < MIN_PER_CLUSTER:
                    cap_i, sim = items_sorted[0]
                    doc = kept_docs[cap_i]
                    t_abs = _caption_time_seconds(doc, ref_time)
                    chosen = [(cid, cap_i, sim, t_abs)]
                    chosen_times = [t_abs]
                    chosen_docs = [doc]

                # Omitted summary (for transparency) – append to chosen docs
                chosen_set = {cap_i for (_, cap_i, _, _) in chosen}
                omitted_docs = [kept_docs[cap_i] for (cap_i, _) in items_sorted if cap_i not in chosen_set]
                omitted_summary = _summarize_omitted_times(omitted_docs, ref_time) if omitted_docs else ""

                for doc in chosen_docs:
                    if omitted_summary:
                        doc.page_content += f" {omitted_summary}"

                per_cluster_chosen[cid] = [(cid, cap_i, sim, t_abs, omitted_summary) for (cid, cap_i, sim, t_abs) in chosen]

                print(f"\n=== Cluster {cid} (video-caption ranked) ===")
                for _, cap_i, sim, t_abs, _ in per_cluster_chosen[cid]:
                    print(f"  [SELECTED-CAND] cap(kept)={cap_i} sim={sim:.3f} t={t_abs:.2f}s")

            # ---------- 6) Global selection: round-robin across clusters to maximize cluster coverage ----------
            # Build queues per cluster ordered by similarity (already ordered)
            cluster_order = keep_cluster_ids_sorted  # already sorted by cluster-best-sim DESC
            queues = {cid: list(per_cluster_chosen.get(cid, [])) for cid in cluster_order}

            representatives = []  # tuples of (cid, cap_kept_idx, sim)
            round_idx = 0
            while len(representatives) < k and any(queues[cid] for cid in cluster_order):
                for cid in cluster_order:
                    if len(representatives) >= k:
                        break
                    if queues[cid]:
                        _cid, cap_kept_i, sim, _t, _om = queues[cid].pop(0)
                        representatives.append((cid, cap_kept_i, sim))
                round_idx += 1

            if not representatives:
                print(colored("No representatives after round-robin; fallback to top-k by similarity.", "red"))
                order = np.argsort([-norm for norm in norm_topM])[:k]
                docs = [docs_with_scores[i][0] for i in order]
                self.working_memory += docs
                return self.memory_to_string(docs)

            # Build final docs (note: cap_kept_i indexes kept_docs)
            final_docs = [kept_docs[cap_kept_i] for (_, cap_kept_i, _) in representatives]

            # ---------- 7) Persist & visualization ----------
            # Fix index-space: convert kept indices back to TOP-M indices for logging/flags
            selected_topM_indices = { keep_idx[cap_kept_i] for (_, cap_kept_i, _) in representatives }  # NEW: correct mapping

            time_offset = getattr(self, 'time_offset', 0.0)
            all_retrieved = []
            for topm_i, (doc, score) in enumerate(docs_with_scores):
                mt = doc.metadata.get('time', 0.0)
                t_val = mt[0] if isinstance(mt, (list, tuple)) else mt
                all_retrieved.append({
                    "idx": int(topm_i),
                    "time": float(t_val + time_offset),
                    "score": float(score),
                    "score_norm": float(norm_topM[topm_i]),
                    "selected": (topm_i in selected_topM_indices)
                })

            if not hasattr(self, 'search_text'):
                self.search_text = {}
            self.search_text[query] = {
                "method_name": f"text_search_diverse_video_ranked_{query}",
                "data": all_retrieved,
                "representatives": [(int(cid), int(keep_idx[cap_kept_i]), float(sim))  # store TOP-M idx for convenience
                                    for (cid, cap_kept_i, sim) in representatives],
                "selected_indices": list(selected_topM_indices),  # NEW: JSON-safe
            }

            # Visualizations (unchanged; ranking changed but viz still useful)
            visualize_highlighted_clusters_open3d(
                sub_objects, clusters_subset, keep_cluster_ids_sorted, winners, [query],
                dim_alpha=0.10, save_dir=os.path.join(out_path, "clusters_vis"), show=True
            )
            visualize_graph_highlight(
                G_sub, clusters_subset, keep_cluster_ids_sorted,
                out_png=os.path.join(out_path, "graph_highlight.png"), show=True
            )

            return self.memory_to_string(final_docs)

        except Exception as e:
            import traceback
            print(colored(f"[search_by_text_IB] ERROR: {e}", "red"))
            traceback.print_exc()
            # graceful fallback to simple top-k (by normalized similarity)
            docs_with_scores = self.text_retriever.vectorstore.similarity_search_with_score(query, k=k)
            # sort desc by similarity if needed
            raw_scores = [s for _, s in docs_with_scores]
            smin, smax = min(raw_scores), max(raw_scores)
            if smax - smin < 1e-9:
                order = list(range(len(docs_with_scores)))
            else:
                norm = [1.0 - (s - smin) / (smax - smin) for s in raw_scores]
                order = np.argsort([-x for x in norm])[:k]
            docs = [docs_with_scores[i][0] for i in order]
            self.working_memory += docs
            return self.memory_to_string(docs)


    def search_by_text_IB_New(self, query: str, k=6) -> str:
        """
        Retrieve many captions, prefer task reasoning based on *video captions* with higher scores,
        group top-ranked captions that describe the same cluster, and include lower-ranked captions
        that describe *different* clusters to ensure diversity. Rank AIB clusters by the *best*
        linked video-caption score and keep only top-N clusters.

        Adds per-cluster omitted-time summaries to the selected caption(s).
        """
        try:
            # ---------- Local helpers (NEW) ----------
            def _fmt_hms(t: float) -> str:
                # format seconds -> HH:MM:SS (no days)
                t = max(0.0, float(t))
                h = int(t // 3600); m = int((t % 3600) // 60); s = int(t % 60)
                return f"{h:02d}:{m:02d}:{s:02d}"

            def _summarize_omitted_times(docs, ref_time: float, max_list: int = 5) -> str:
                """Return a short human-readable summary of omitted caption times."""
                times = []
                for d in docs:
                    mt = d.metadata.get('time', 0.0)
                    t_val = mt[0] if isinstance(mt, (list, tuple)) else mt
                    times.append(float(t_val) + float(ref_time))
                if not times:
                    return ""
                times = sorted(times)
                shown = ", ".join(_fmt_hms(t) for t in times[:max_list])
                more = len(times) - max_list
                tail = f" (+{more} more)" if more > 0 else ""
                return f"[Omitted same-cluster timestamps] {shown}{tail}"

            # Choose whether to attach the summary to only the first selected caption of each cluster
            APPEND_SUMMARY_ONCE_PER_CLUSTER = True

            # ---------- Tunables ----------
            TOP_M        = 100
            KEEP_THRESH  = 0.4
            TASK_FILTER_THRESH = 0.0  # permissive; video captions drive ranking
            KEEP_TOP_CLUSTERS = max(3, min(6, k))

            AUTO_MAX_PER_CLUSTER = 3
            MIN_PER_CLUSTER      = 1
            TIME_SEP             = 90.0  # seconds

            GRAPH_KW = dict(
                vertical_axis='y', down_positive=True,
                dist_radius=3.0, z_overlap=False, z_slack=0.40,
                iou_thresh=0.01, covis_min=0, knn=6,
                ground_height_thresh=0.1, ground_floor_thresh=0.05,
                dilate_eps=0.02
            )

            # ---------- 1) Retrieve ----------
            docs_with_scores = self.text_retriever.vectorstore.similarity_search_with_score(query, k=TOP_M)
            if not docs_with_scores:
                print(colored("Warning: No captions found for the query.", "red"))
                return ""

            # Normalize to [0,1] similarities (monotonic if raw is distance)
            raw_scores = [s for _, s in docs_with_scores]
            smin, smax = min(raw_scores), max(raw_scores)
            if smax - smin < 1e-9:
                norm_topM = [1.0 for _ in raw_scores]
            else:
                norm_topM = [1.0 - (s - smin) / (smax - smin) for s in raw_scores]
            print(colored(f"Normalized scores (top-M): {norm_topM}", "yellow"))

            keep_idx = [i for i, v in enumerate(norm_topM) if v >= KEEP_THRESH]
            print(colored(f"Keeping {len(keep_idx)} / {len(norm_topM)} captions (>= {KEEP_THRESH:.2f})", "yellow"))
            if not keep_idx:
                keep_idx = list(range(min(TOP_M, len(docs_with_scores))))

            kept_docs      = [docs_with_scores[i][0] for i in keep_idx]
            kept_rawscore  = [docs_with_scores[i][1] for i in keep_idx]
            kept_normscore = [norm_topM[i] for i in keep_idx]

            # ---------- 2) Build subset of objects ----------
            cap_to_local_obj_ids = []
            subset_local_ids_set = set()
            for doc in kept_docs:
                gids = _parse_object_ids(doc.metadata.get('object_id'))
                lids = _global_ids_to_local_indices(gids, objid_to_idx=getattr(self, 'objid_to_idx', None))
                lids = list(set(lids))
                cap_to_local_obj_ids.append(lids)
                subset_local_ids_set.update(lids)

            subset_local_ids = sorted(list(subset_local_ids_set))
            print(colored(f"Subset has {len(subset_local_ids)} unique objects mentioned by captions", "blue"))
            if len(subset_local_ids) == 0:
                print(colored("No objects covered by captions; fallback to top-k by similarity.", "red"))
                order = np.argsort([-norm for norm in norm_topM])[:k]
                topk_docs = [docs_with_scores[i][0] for i in order]
                self.working_memory += topk_docs
                return self.memory_to_string(topk_docs)

            # ---------- 3) Subgraph + AIB ----------
            object_cap_features_np = np.array([obj['ft'].flatten() for obj in self.scene_graph])  # (N,D)
            print(colored(f"Building subgraph from {len(subset_local_ids)} objects", "blue"))
            G_sub, sub_objects, sub_feats = _build_subgraph_from_indices(
                self.scene_graph, object_cap_features_np, subset_local_ids, GRAPH_KW
            )

            task_ft = _encode_query_sbert(self.sbert_model, query).detach().cpu().numpy()[None, :]

            out_path = getattr(self, 'out_dir', './_tmp')
            Path(out_path).mkdir(parents=True, exist_ok=True)
            cfg_yaml_path = os.path.join(out_path, "cluster_config.yaml")
            write_default_cluster_config(cfg_yaml_path)

            clusters_subset = run_ib_clustering(sub_feats, task_ft, G_sub, cfg_yaml_path)
            print(colored(f"[AIB] Formed {len(clusters_subset)} clusters from {len(subset_local_ids)} primitives.", "cyan"))

            clustered_objects = merge_all_clusters(sub_objects, clusters_subset)
            if TASK_FILTER_THRESH > 0:
                clustered_objects_filtered = filter_clusters_by_task(clustered_objects, task_ft, TASK_FILTER_THRESH)
                print(colored(f"[AIB] Filtered to {len(clustered_objects_filtered)} task-relevant objects.", "cyan"))

            scores_task, winners = cluster_task_scores_cosine(sub_objects, clusters_subset, task_ft, pool="max")

            # ---------- 4) Map captions → dominant cluster; rank clusters by caption similarity ----------
            subset_local_to_pos = {lid: pos for pos, lid in enumerate(subset_local_ids)}
            objpos_to_cid = {}
            for cid, members in enumerate(clusters_subset):
                for pos in members:
                    objpos_to_cid[pos] = cid

            groups = {}  # cid -> list[(cap_kept_idx, kept_normscore)]
            for cap_i, lids in enumerate(cap_to_local_obj_ids):
                hits = []
                for lid in lids:
                    pos = subset_local_to_pos.get(lid, None)
                    if pos is None:
                        continue
                    cid = objpos_to_cid.get(pos, None)
                    if cid is not None:
                        hits.append(int(cid))
                if not hits:
                    continue
                dom = max(set(hits), key=hits.count)
                groups.setdefault(dom, []).append((cap_i, kept_normscore[cap_i]))

            if not groups:
                print(colored("No captions map to clusters; fallback to top-k by similarity.", "red"))
                order = np.argsort([-norm for norm in norm_topM])[:k]
                docs = [docs_with_scores[i][0] for i in order]
                self.working_memory += docs
                return self.memory_to_string(docs)

            cluster_best_sim = []
            for cid, items in groups.items():
                best = max(sim for _, sim in items)
                cluster_best_sim.append((cid, best))
            cluster_best_sim.sort(key=lambda t: t[1], reverse=True)

            keep_cluster_ids_sorted = [cid for cid, _ in cluster_best_sim[:KEEP_TOP_CLUSTERS]]
            print(colored(f"Keeping top {len(keep_cluster_ids_sorted)} clusters by video-caption score", "green"))

            # ---------- 5) Per-cluster selection + OMITTED SUMMARY (NEW) ----------
            ref_time = getattr(self, 'time_offset', 0.0)
            per_cluster_chosen = {}  # cid -> list[(cid, cap_kept_i, cap_sim, t_abs, omitted_summary)]
            for cid in keep_cluster_ids_sorted:
                items = groups.get(cid, [])
                if not items:
                    continue
                items_sorted = sorted(items, key=lambda t: t[1], reverse=True)

                chosen = []
                chosen_times = []
                chosen_docs = []
                for cap_i, sim in items_sorted:
                    doc = kept_docs[cap_i]
                    t_abs = _caption_time_seconds(doc, ref_time)
                    if all(abs(t_abs - prev) >= TIME_SEP for prev in chosen_times):
                        chosen.append((cid, cap_i, sim, t_abs))
                        chosen_times.append(t_abs)
                        chosen_docs.append(doc)
                    if len(chosen) >= AUTO_MAX_PER_CLUSTER:
                        break

                if len(chosen) < MIN_PER_CLUSTER:
                    cap_i, sim = items_sorted[0]
                    doc = kept_docs[cap_i]
                    t_abs = _caption_time_seconds(doc, ref_time)
                    chosen = [(cid, cap_i, sim, t_abs)]
                    chosen_times = [t_abs]
                    chosen_docs = [doc]

                # Compute omitted summary ONCE per cluster
                chosen_set = {cap_i for (_, cap_i, _, _) in chosen}
                omitted_docs = [kept_docs[cap_i] for (cap_i, _) in items_sorted if cap_i not in chosen_set]
                omitted_summary = _summarize_omitted_times(omitted_docs, ref_time) if omitted_docs else ""

                # Append omitted summary to each selected doc's text
                for doc in chosen_docs:
                    doc.page_content += f" {omitted_summary}"
                # Append summary to either the first selected caption (default) or all selected captions
                targets = [chosen_docs[0]] if (APPEND_SUMMARY_ONCE_PER_CLUSTER and chosen_docs) else chosen_docs
                for doc in targets:
                    if omitted_summary:
                        # also drop into metadata for structured renderers
                        doc.metadata['omitted_summary'] = omitted_summary
                        doc.metadata['cluster_id'] = int(cid)
                        doc.page_content = (doc.page_content.rstrip() + "\n\n" + omitted_summary)

                per_cluster_chosen[cid] = [(cid, cap_i, sim, t_abs, omitted_summary) for (cid, cap_i, sim, t_abs) in chosen]

                print(f"\n=== Cluster {cid} (video-caption ranked) ===")
                for _, cap_i, sim, t_abs, _ in per_cluster_chosen[cid]:
                    print(f"  [SELECTED-CAND] cap(kept)={cap_i} sim={sim:.3f} t={_fmt_hms(t_abs)}")

            # ---------- 6) Global selection (round-robin across clusters) ----------
            cluster_order = keep_cluster_ids_sorted
            queues = {cid: list(per_cluster_chosen.get(cid, [])) for cid in cluster_order}

            representatives = []  # (cid, cap_kept_idx, sim)
            while len(representatives) < k and any(queues[cid] for cid in cluster_order):
                for cid in cluster_order:
                    if len(representatives) >= k:
                        break
                    if queues[cid]:
                        _cid, cap_kept_i, sim, _t, _om = queues[cid].pop(0)
                        representatives.append((cid, cap_kept_i, sim))

            if not representatives:
                print(colored("No representatives after round-robin; fallback to top-k by similarity.", "red"))
                order = np.argsort([-norm for norm in norm_topM])[:k]
                docs = [docs_with_scores[i][0] for i in order]
                self.working_memory += docs
                return self.memory_to_string(docs)

            final_docs = [kept_docs[cap_kept_i] for (_, cap_kept_i, _) in representatives]

            # ---------- 7) Persist & visualization ----------
            selected_topM_indices = { keep_idx[cap_kept_i] for (_, cap_kept_i, _) in representatives }

            time_offset = getattr(self, 'time_offset', 0.0)
            all_retrieved = []
            for topm_i, (doc, score) in enumerate(docs_with_scores):
                mt = doc.metadata.get('time', 0.0)
                t_val = mt[0] if isinstance(mt, (list, tuple)) else mt
                all_retrieved.append({
                    "idx": int(topm_i),
                    "time": float(t_val + time_offset),
                    "score": float(score),
                    "score_norm": float(norm_topM[topm_i]),
                    "selected": (topm_i in selected_topM_indices)
                })

            if not hasattr(self, 'search_text'):
                self.search_text = {}
            self.search_text[query] = {
                "method_name": f"text_search_diverse_video_ranked_{query}",
                "data": all_retrieved,
                "representatives": [(int(cid), int(keep_idx[cap_kept_i]), float(sim))
                                    for (cid, cap_kept_i, sim) in representatives],
                "selected_indices": list(selected_topM_indices),
            }

            # visualize_highlighted_clusters_open3d(
            #     sub_objects, clusters_subset, keep_cluster_ids_sorted, winners, [query],
            #     dim_alpha=0.10, save_dir=os.path.join(out_path, "clusters_vis"), show=True
            # )
            # visualize_graph_highlight(
            #     G_sub, clusters_subset, keep_cluster_ids_sorted,
            #     out_png=os.path.join(out_path, "graph_highlight.png"), show=True
            # )

            return self.memory_to_string(final_docs)

        except Exception as e:
            import traceback
            print(colored(f"[search_by_text_IB] ERROR: {e}", "red"))
            traceback.print_exc()
            docs_with_scores = self.text_retriever.vectorstore.similarity_search_with_score(query, k=k)
            raw_scores = [s for _, s in docs_with_scores]
            smin, smax = min(raw_scores), max(raw_scores)
            if smax - smin < 1e-9:
                order = list(range(len(docs_with_scores)))
            else:
                norm = [1.0 - (s - smin) / (smax - smin) for s in raw_scores]
                order = np.argsort([-x for x in norm])[:k]
            docs = [docs_with_scores[i][0] for i in order]
            self.working_memory += docs
            return self.memory_to_string(docs)


    def search_by_text_IB_debug(self, query: str, k=6) -> str:
        """
        Retrieve many captions, restrict to those covering objects (object_id) related to the query,
        cluster the union subset via IB (task = query), and return one (or a few, time-separated) best
        caption(s) per *selected* cluster to promote diversity.

        Changes vs. previous version:
        - Introduces MAX_GROUPS (N): when too many clusters are returned, we keep only the clusters
            whose *best* caption (lowest retriever distance) ranks highest, up to N clusters.
        - For every selected caption, we append a per-cluster summary including merged object ids,
            object captions, and 3D bbox center/extent to aid task reasoning.
        - Ensures `selected_indices` are indices into the full TOP_M pool (not the kept subset),
            which avoids downstream indexing errors while preserving the same key names.
        """
        try:
            # ---------- Tunables ----------
            TOP_M        = 100      # retrieve a big pool
            KEEP_THRESH  = 0.4       # normalized similarity threshold to keep captions
            TASK_FILTER_THRESH = 0.4 # SBERT task relevance after merging
            MAX_GROUPS   = 6         # <-- N: maximum number of clusters to keep for reasoning
            USE_TASK_SCORE = False   # <-- NEW: set to False to disable Step 5(a) gating
            GRAPH_KW = dict(
                vertical_axis='y', down_positive=True,
                dist_radius=3.0, z_overlap=False, z_slack=0.40,
                iou_thresh=0.01, covis_min=0, knn=6,
                ground_height_thresh=0.1, ground_floor_thresh=0.05,
                dilate_eps=0.02
            )

            # ---------- 1) Retrieve ----------
            docs_with_scores = self.text_retriever.vectorstore.similarity_search_with_score(query, k=TOP_M)
            if not docs_with_scores:
                print(colored("Warning: No captions found for the query.", "red"))
                return ""

            # Normalize scores to [0,1] for filtering (assume distance: smaller=better → similarity=1-normed_distance)
            raw_scores = [s for _, s in docs_with_scores]
            smin, smax = min(raw_scores), max(raw_scores)
            if smax - smin < 1e-9:
                norm = [1.0 for _ in raw_scores]
            else:
                norm = [1.0 - (s - smin) / (smax - smin) for s in raw_scores]

            keep_idx = [i for i, v in enumerate(norm) if v >= KEEP_THRESH]
            if not keep_idx:
                keep_idx = list(range(min(TOP_M, len(docs_with_scores))))

            # Kept views (subset) and a mapping back to TOP_M pool indices
            kept_docs         = [docs_with_scores[i][0] for i in keep_idx]
            kept_scores_dist  = [docs_with_scores[i][1] for i in keep_idx]  # distance; lower is better
            kept_norm         = [norm[i] for i in keep_idx]                  # similarity-like [0,1]
            kept_pool_indices = keep_idx[:]                                  # indices into the original TOP_M pool

            # ---------- 2) Build subset of objects from object_id ----------
            cap_to_local_obj_ids = []
            subset_local_ids_set = set()
            for doc in kept_docs:
                gids = _parse_object_ids(doc.metadata.get('object_id'))

                lids = _global_ids_to_local_indices(gids, objid_to_idx=getattr(self, 'objid_to_idx', None))
                # print(colored(f"Doc {doc.metadata.get('id', 'unknown')} has lids: {lids}", "blue"))
                lids = list(set(lids))
                # print(colored(f"Doc {doc.metadata.get('id', 'unknown')} unique lids: {lids}", "blue"))

                cap_to_local_obj_ids.append(lids)
                subset_local_ids_set.update(lids)

            subset_local_ids = sorted(list(subset_local_ids_set))
            if len(subset_local_ids) == 0:
                print(colored("No objects covered by captions; falling back to top-k plain retrieval.", "red"))
                topk_docs = [doc for doc, _ in docs_with_scores[:k]]
                self.working_memory += topk_docs
                return self.memory_to_string(topk_docs)

            # ---------- 3) Subgraph + IB on subset ----------
            object_cap_features_np = np.array([obj['ft'].flatten() for obj in self.scene_graph])  # (N,D)
            print(colored(f"Building subgraph from {len(subset_local_ids)} objects", "blue"))
            G_sub, sub_objects, sub_feats = _build_subgraph_from_indices(
                self.scene_graph, object_cap_features_np, subset_local_ids, GRAPH_KW
            )
            #print(colored(f"Subgraph has {subset_local_ids}", "blue"))

            # Task feature from query (SBERT) -> (1, D)
            task_ft = _encode_query_sbert(self.sbert_model, query).detach().cpu().numpy()[None, :]

            # IB config
            out_path = getattr(self, 'out_dir', './_tmp')
            Path(out_path).mkdir(parents=True, exist_ok=True)
            cfg_yaml_path = os.path.join(out_path, "cluster_config.yaml")
            write_default_cluster_config(cfg_yaml_path)

            # IB clusters in subset index space (clusters contain positions into sub_objects)
            clusters_subset = run_ib_clustering(sub_feats, task_ft, G_sub, cfg_yaml_path)
            print(colored(f"[AIB] Formed {len(clusters_subset)} clusters from {len(subset_local_ids)} primitives.", "cyan"))

            # ---------- 4) Merge + filter by task relevance ----------
            clustered_objects = merge_all_clusters(sub_objects, clusters_subset)
            print(colored(f"[AIB] After merge: {len(clustered_objects)} objects.", "cyan"))

            clustered_objects_filtered = filter_clusters_by_task(clustered_objects, task_ft, TASK_FILTER_THRESH)
            print(colored(f"[AIB] Filtered to {len(clustered_objects_filtered)} task-relevant objects.", "cyan"))

            # ---------- 5) Score clusters by task & choose which clusters to keep ----------
            # (a) task scores from features (still useful for logging/highlighting)
            if USE_TASK_SCORE:
                scores_task, winners = cluster_task_scores_cosine(sub_objects, clusters_subset, task_ft, pool="max")
                HIGHLIGHT_THR = TASK_FILTER_THRESH
                initial_keep_cluster_idxs = np.where(scores_task >= HIGHLIGHT_THR)[0]
                if initial_keep_cluster_idxs.size == 0:
                    initial_keep_cluster_idxs = np.argsort(-scores_task)[:min(3, len(clusters_subset))]
                    print(f"No clusters above {HIGHLIGHT_THR:.2f}, keeping top {len(initial_keep_cluster_idxs)} by task score.")
                else:
                    print(f"Clusters above {HIGHLIGHT_THR:.2f}: {len(initial_keep_cluster_idxs)}")
            else:
                # If not using task scores, keep all clusters (no filtering)
                initial_keep_cluster_idxs = np.arange(len(clusters_subset))

            # (b) NEW: Rank clusters by each cluster's *best caption* (lowest distance) among captions mapped to it.
            # Build caption→cluster groups first (dominant cluster voting across a caption's object ids).
            print("\n=== Step 5b: Mapping captions to clusters for ranking ===")
            subset_local_to_pos = {lid: pos for pos, lid in enumerate(subset_local_ids)}
            objpos_to_cid = {}
            for cid in range(len(clusters_subset)):
                for pos in clusters_subset[cid]:
                    objpos_to_cid[pos] = cid

            # Group candidate captions per cluster (use only clusters that passed (a) task screening)
            groups_all = {int(cid): [] for cid in range(len(clusters_subset))}
            for cap_kept_i, lids in enumerate(cap_to_local_obj_ids):
                hits = []
                for lid in lids:
                    pos = subset_local_to_pos.get(lid, None)
                    if pos is None:
                        continue
                    cid = objpos_to_cid.get(pos, None)
                    if cid is not None:
                        hits.append(int(cid))
                if not hits:
                    continue
                dominant = max(set(hits), key=hits.count)
                # Store: (kept_caption_idx, pool_distance_score, pool_index)
                groups_all[dominant].append(
                    (cap_kept_i, float(kept_scores_dist[cap_kept_i]), int(kept_pool_indices[cap_kept_i]))
                )

            # Compute best (lowest) distance per cluster among candidate captions (only clusters that pass task screening)
            cluster_best_distance = {}
            for cid in initial_keep_cluster_idxs:
                items = groups_all.get(int(cid), [])
                if not items:
                    # No caption mapped → set to +inf so it falls to the bottom but still eligible if needed
                    cluster_best_distance[int(cid)] = float('inf')
                else:
                    best = min(items, key=lambda t: t[1])  # t[1] is distance (lower is better)
                    cluster_best_distance[int(cid)] = best[1]

            # Rank by best caption distance (ascending), then keep up to MAX_GROUPS clusters
            ranked_cids = sorted(cluster_best_distance.keys(), key=lambda x: cluster_best_distance[x])
            kept_cluster_ids = ranked_cids[:min(MAX_GROUPS, len(ranked_cids))]
            print(colored(f"Selected {len(kept_cluster_ids)} clusters (MAX_GROUPS={MAX_GROUPS}) by best-caption rank.", "cyan"))

            # ---------- 6) Within each kept cluster, select top captions with time separation ----------
            print("\n=== Step 6: Selecting top captions per kept cluster ===")
            TIME_SEP = 90.0            # min temporal separation (seconds) within a cluster’s chosen reps
            AUTO_MAX_PER_CLUSTER = 3   # per-cluster cap
            MIN_PER_CLUSTER = 1        # always keep at least one if the cluster has any candidates

            representatives = []  # will store tuples of (cluster_id, pool_caption_idx, pool_distance_score)
            ref_time = getattr(self, 'time_offset', 0.0)

            def _ensure_o3d_aabb(bbox):
                """
                Convert to Open3D AxisAlignedBoundingBox if possible.
                - Keep AABB as-is
                - Convert OOBB -> AABB
                - Convert Nx3 array -> AABB
                """
                if bbox is None:
                    return None
                if isinstance(bbox, o3d.geometry.AxisAlignedBoundingBox):
                    return bbox
                if isinstance(bbox, o3d.geometry.OrientedBoundingBox):
                    return bbox.get_axis_aligned_bounding_box()
                try:
                    arr = np.asarray(bbox)
                    if arr.ndim == 2 and arr.shape[1] == 3:
                        return o3d.geometry.AxisAlignedBoundingBox(arr.min(axis=0), arr.max(axis=0))
                except Exception:
                    pass
                return None
            # Build a compact per-cluster object summary string
            def _cluster_object_summary(cid, subset_local_ids):
                members = clusters_subset[cid]
                print(colored(f"Cluster {cid} has {members} members.", "cyan"))

                parts = []
                for m in members:
                    o = sub_objects[m]
                    # Prefer global id if present; fallback to any available identifier
                    oid = o.get('global_id', o.get('object_id', o.get('id', f"{subset_local_ids[m]}")))
                    cap = o.get('caption', o.get('desc', ''))

                    bbox  = _ensure_o3d_aabb(o.get("bbox", None))
                    if bbox is None:
                        print(colored(f"Warning: No bbox for object {oid} in cluster {cid}.", "yellow"))
                        continue

                    center = np.round(np.asarray(bbox.get_center(), dtype=float), 3)  # (cx, cy, cz)
                    extent = np.round(np.asarray(bbox.get_extent(), dtype=float), 3)  # (dx, dy, dz)

                    if center is None: center = "center=None"
                    if extent is None: extent = "extent=None"
                    if cap:
                        # parts.append(f"Object {oid}: caption= {cap}; center: {center}: extent:{extent}")
                        parts.append(f"Object {oid}: center: {center}")
                    else:
                        parts.append(f"[obj:{oid}] {center} {extent}")
                if not parts:
                    return "Objects: (none)"
                return "Detected objects : " + " | ".join(parts)

            # Map from kept-subset caption index → original top-M pool index for later consistency
            kept_to_pool = {i_kept: pool_i for i_kept, pool_i in enumerate(kept_pool_indices)}

            for cid in kept_cluster_ids:
                items = groups_all.get(int(cid), [])
                if not items:
                    continue  # no candidate captions mapped to this cluster

                # Sort within-cluster by distance ascending (best first)
                items_sorted = sorted(items, key=lambda t: t[1])  # (kept_i, dist, pool_i)

                chosen = []
                chosen_times = []
                chosen_docs = []
                for kept_i, dist_val, pool_i in items_sorted:
                    doc = kept_docs[kept_i]
                    t_abs = _caption_time_seconds(doc, ref_time)
                    if all(abs(t_abs - prev) >= TIME_SEP for prev in chosen_times):
                        chosen.append((cid, kept_i, dist_val, t_abs, pool_i))
                        chosen_times.append(t_abs)
                        chosen_docs.append(doc)
                    if len(chosen) >= AUTO_MAX_PER_CLUSTER:
                        break

                if len(chosen) < MIN_PER_CLUSTER:
                    kept_i, dist_val, pool_i = items_sorted[0]
                    doc = kept_docs[kept_i]
                    t_abs = _caption_time_seconds(doc, ref_time)
                    chosen = [(cid, kept_i, dist_val, t_abs, pool_i)]
                    chosen_times = [t_abs]
                    chosen_docs = [doc]

                # Build summary for omitted times (from *this* cluster only)
                chosen_kept_set = set((kept_i for (_, kept_i, _, _, _) in chosen))
                omitted_items = [(kept_i, dist_val, pool_i) for (kept_i, dist_val, pool_i) in items_sorted if kept_i not in chosen_kept_set]
                omitted_docs = [kept_docs[kept_i] for (kept_i, _, _) in omitted_items]
                omitted_summary = _summarize_omitted_times(omitted_docs, ref_time)

                # NEW: Build object summary for this cluster (object ids, captions, bbox centers/extents)
                obj_summary = _cluster_object_summary(int(cid), subset_local_ids)

                # Append both summaries to each selected doc's text
                addon = f" {omitted_summary} {obj_summary}"
                for doc in chosen_docs:
                    doc.page_content += addon

                # Keep representatives using POOL indices for downstream consistency
                for (_, kept_i, dist_val, _, pool_i) in chosen:
                    representatives.append((int(cid), int(pool_i), float(dist_val)))  # (cluster_id, pool_caption_idx, distance)

            # (Optional) Global cap across clusters by best distance; still respect final output k
            representatives.sort(key=lambda t: t[2])   # lower distance = better
            # Representatives may exceed k after per-cluster picks; we will cap the *returned docs* by k.
            # Keep all reps in metadata so downstream can visualize more if needed.

            # ---------- 7) Build final docs from POOL indices and cap return by k ----------
            # Map pool index → (doc, score)
            pool_docs = [doc for (doc, _) in docs_with_scores]
            pool_scores = [score for (_, score) in docs_with_scores]

            # unique pool indices in rep order
            seen = set()
            final_pool_indices = []
            for (_, pool_i, _) in representatives:
                if pool_i not in seen:
                    seen.add(pool_i)
                    final_pool_indices.append(pool_i)
            final_pool_indices = final_pool_indices[:min(k, len(final_pool_indices))]

            final_docs = [pool_docs[i] for i in final_pool_indices]
            final_docs_with_scores = [(pool_docs[i], pool_scores[i]) for i in final_pool_indices]

            # ---------- 8) Persist searchable debug info ----------
            # Build a full list for the top-M retrieved pool (times + original scores)
            time_offset = getattr(self, 'time_offset', 0.0)
            all_retrieved = []
            for pool_i, (doc, score) in enumerate(docs_with_scores):
                mt = doc.metadata.get('time', 0.0)
                t_val = mt[0] if isinstance(mt, (list, tuple)) else mt
                all_retrieved.append({
                    "idx": int(pool_i),                          # caption index within the top-M pool
                    "time": float(t_val + time_offset),          # absolute time
                    "score": float(score),                       # original score from retriever (distance)
                    "selected": (pool_i in set(final_pool_indices))  # whether it was picked as representative (returned)
                })

            # Save into self.search_text (keys unchanged)
            if not hasattr(self, 'search_text'):
                self.search_text = {}

            # NOTE: representatives now use POOL indices to ensure consistency downstream.
            #  - "representatives": list[(cluster_id, pool_caption_idx, distance)]
            #  - "selected_indices": set of POOL indices returned (matches all_retrieved[idx]['selected'] == True)
            self.search_text[query] = {
                "method_name": f"text_search-{query}",
                "data": all_retrieved,
                "representatives": representatives,
                "selected_indices": set(final_pool_indices),  # pool-space indices
            }

            return self.memory_to_string(final_docs)

        except Exception as e:
            import traceback
            print(colored(f"[search_by_text_IB] ERROR: {e}", "red"))
            traceback.print_exc()
            # graceful fallback to simple top-k
            docs = [doc for doc, _ in self.text_retriever.vectorstore.similarity_search_with_score(query, k=k)]
            self.working_memory += docs
            return self.memory_to_string(docs)

    def search_by_text_IB_v0(self, query: str, k=8) -> str:
            """
            Retrieve many captions, restrict to those covering objects (object_id) related to the query,
            cluster the union subset via IB (task = query), and return one best caption per cluster (diverse).
            """
            initial_tops = self.args.topk
            try:
                # ---------- Tunables ----------
                TOP_M        = 100     # retrieve a big pool
                KEEP_THRESH  = 0.35    # normalized similarity threshold to keep captions
                TASK_FILTER_THRESH = 0.4  # SBERT task relevance after merging
                GRAPH_KW = dict(
                    vertical_axis='y', down_positive=True,
                    dist_radius=3.0, z_overlap=False, z_slack=0.40,
                    iou_thresh=0.01, covis_min=0, knn=6,
                    ground_height_thresh=0.1, ground_floor_thresh=0.05,
                    dilate_eps=0.02
                )

                # ---------- 1) Retrieve ----------
                docs_with_scores = self.text_retriever.vectorstore.similarity_search_with_score(query, k=TOP_M)
                if not docs_with_scores:
                    print(colored("Warning: No captions found for the query.", "red"))
                    return ""

                # Normalize scores to [0,1]
                raw_scores = [s for _, s in docs_with_scores]
                smin, smax = min(raw_scores), max(raw_scores)
                if smax - smin < 1e-9:
                    norm = [1.0 for _ in raw_scores]
                else:
                    # If vectorstore returns DISTANCE (smaller=better), convert to similarity
                    norm = [1.0 - (s - smin) / (smax - smin) for s in raw_scores]
                # print(colored(f"Normalized scores: {norm}", "yellow"))
                # TODO: can be removed
                keep_idx = [i for i, v in enumerate(norm) if v >= KEEP_THRESH]
                # print(colored(f"Initial keep indices (>= {KEEP_THRESH:.2f}): {keep_idx}", "yellow"))
                # idx_sorted = sorted(range(len(norm)), key=lambda i: norm[i], reverse=True)
                # keep_idx = idx_sorted[:min(10, len(idx_sorted))]
                # fallback: force top-10 (or fewer if not enough items)
                if len(keep_idx) < 12:
                    idx_sorted = sorted(range(len(norm)), key=lambda i: norm[i], reverse=True)
                    keep_idx = idx_sorted[:min(10, len(idx_sorted))]
                    # print top-10 norm scores
                    # print(colored(f"Top-10 norm scores: {[norm[i] for i in idx_sorted[:10]]}", "yellow"))

                print(colored(f"Keeping {len(keep_idx)} / {len(norm)} captions (>= {KEEP_THRESH:.2f})", "yellow"))
                if not keep_idx:
                    keep_idx = list(range(min(TOP_M, len(docs_with_scores))))
                #keep_idx = list(range(len(docs_with_scores)))
                kept_docs   = [docs_with_scores[i][0] for i in keep_idx]
                kept_scores = [docs_with_scores[i][1] for i in keep_idx]

                kept_times = [docs_with_scores[i][0].metadata.get('time', 0.0) for i in keep_idx]
                # for doc, score, t in zip(kept_docs, kept_scores, kept_times):
                #     t = t[0] if isinstance(t, (list, tuple)) else t
                #     t += self.time_offset
                #     t = localtime(t)
                #     t = strftime('%Y-%m-%d %H:%M:%S', t)
                #     print(f"  [KEPT-CAND] id={doc.metadata.get('id', 'unknown')} score={score:.3f} text='At {t}{doc.page_content}...'")
                kept_norm   = [norm[i] for i in keep_idx]

                # ---------- 2) Build subset of objects from object_id ----------
                cap_to_local_obj_ids = []
                subset_local_ids_set = set()
                for doc in kept_docs:
                    gids = _parse_object_ids(doc.metadata.get('object_id'))
                    lids = _global_ids_to_local_indices(gids, objid_to_idx=getattr(self, 'objid_to_idx', None))
                    lids = list(set(lids))
                    cap_to_local_obj_ids.append(lids)
                    subset_local_ids_set.update(lids)

                subset_local_ids = sorted(list(subset_local_ids_set))
                if len(subset_local_ids) == 0:
                    print(colored("No objects covered by captions; falling back to top-k plain retrieval.", "red"))
                    topk_docs = [doc for doc, _ in docs_with_scores[:k]]
                    self.working_memory += topk_docs
                    return self.memory_to_string(topk_docs)

                # ---------- 3) Subgraph + IB on subset ----------
                # NOTE: if your objects live in self.objects, replace self.scene_graph with self.objects below
                object_cap_features_np = np.array([obj['ft'].flatten() for obj in self.scene_graph])  # (N,D)
                # visualize_objects_org(self.scene_graph)
                print(colored(f"Building subgraph from {len(subset_local_ids)} objects", "blue"))
                background_terms = [
                    "wall", "ground", "floor", "ceiling", "pillar", "beam",
                    "staging area", "walkway", "window", "ramp", "shelf wall"
                ]
                # G_sub, sub_objects, sub_feats = _build_subgraph_from_indices_without_BG(
                #     self.scene_graph, object_cap_features_np, subset_local_ids, GRAPH_KW,
                #     sbert_model=self.sbert_model,                # 你的 SBERT 模型实例
                #     background_terms=background_terms,      # 背景词表
                #     bg_threshold=0.45,                      # 阈值可按需要微调
                #     verbose=True
                # )
                G_sub, sub_objects, sub_feats = _build_subgraph_from_indices(
                    self.scene_graph, object_cap_features_np, subset_local_ids, GRAPH_KW
                )
                # visualize_objects_org(sub_objects)
                # Task feature from query (SBERT) -> (1, D)
                task_ft = _encode_query_sbert(self.sbert_model, query).detach().cpu().numpy()[None, :]

                # IB config
                out_path = getattr(self, 'out_dir', './_tmp')
                Path(out_path).mkdir(parents=True, exist_ok=True)
                cfg_yaml_path = os.path.join(out_path, "cluster_config.yaml")
                write_default_cluster_config(cfg_yaml_path)

                # IB clusters in subset index space
                clusters_subset = run_ib_clustering(sub_feats, task_ft, G_sub, cfg_yaml_path)
                print(colored(f"[AIB] Formed {len(clusters_subset)} clusters from {len(subset_local_ids)} primitives.", "cyan"))

                # ---------- 4) Merge + filter by task relevance ----------
                # clustered_objects = merge_all_clusters(sub_objects, clusters_subset)
                # # print(colored(f"[AIB] After merge: {len(clustered_objects)} objects.", "cyan"))

                # clustered_objects_filtered = filter_clusters_by_task(clustered_objects, task_ft, TASK_FILTER_THRESH)
                # # clustered_objects_filtered = filter_clusters_by_task_topk(clustered_objects, task_ft, topk=6)
                # print(colored(f"[AIB] Filtered to {len(clustered_objects_filtered)} task-relevant objects.", "cyan"))


                # ----- 5) Score clusters by task & keep only task-relevant ones -----
                # print("\n=== Step 5: Scoring clusters by task relevance ===")
                scores, winners = cluster_task_scores_cosine(sub_objects, clusters_subset, task_ft, pool="max")
                #print(f"Cluster scores vs task: {scores}")
                #print(f"Best matching task index per cluster: {winners}")

                HIGHLIGHT_THR = 0.4  # align with TASK_FILTER_THRESH

                keep_cluster_idxs = np.where(scores >= HIGHLIGHT_THR)[0]
                print(colored(f"Clusters above {HIGHLIGHT_THR:.2f}: {len(keep_cluster_idxs)}", "cyan"))
                keep_cluster_idxs = np.argsort(-scores)[:min(k, len(clusters_subset))]
                print(f"Keeping top {len(keep_cluster_idxs)} clusters by score. \n{keep_cluster_idxs}")
                print(f"Cluster scores: {[scores[cid] for cid in keep_cluster_idxs]}")

                # if keep_cluster_idxs.size < k:
                #     keep_cluster_idxs = np.argsort(-scores)[:k] #min(k, len(clusters_subset))
                #     print(f"No clusters above {HIGHLIGHT_THR:.2f}, keeping top {len(keep_cluster_idxs)} by score. \n{keep_cluster_idxs}")
                # else:
                #     keep_cluster_idxs = np.argsort(-scores)[:min(k, len(clusters_subset))]
                #     print(f"Keeping {len(keep_cluster_idxs)} clusters above {HIGHLIGHT_THR:.2f} \n{keep_cluster_idxs}")
                for cid in keep_cluster_idxs:
                    print(f"cluster {cid}: score={scores[cid]:.4f}")
                # ----- 6) Map captions → dominant kept cluster via object ids -----
                print("\n=== Step 6: Mapping captions to dominant clusters ===")
                subset_local_to_pos = {lid: pos for pos, lid in enumerate(subset_local_ids)}
                objpos_to_keptcid = {}
                for cid in keep_cluster_idxs:
                    for pos in clusters_subset[cid]:
                        objpos_to_keptcid[pos] = cid

                ##print(f"subset_local_to_pos: {subset_local_to_pos}")
                ##print(f"objpos_to_keptcid: {objpos_to_keptcid}")

                groups = {int(cid): [] for cid in keep_cluster_idxs}
                for cap_i, lids in enumerate(cap_to_local_obj_ids):
                    hits = []
                    for lid in lids:
                        pos = subset_local_to_pos.get(lid, None)
                        if pos is None:
                            continue
                        cid = objpos_to_keptcid.get(pos, None)
                        if cid is not None:
                            hits.append(int(cid))
                    if not hits:
                        continue
                    dom = max(set(hits), key=hits.count)
                    groups[dom].append((cap_i, kept_scores[cap_i]))
                    ##print(f"Under query: {query}; Caption {cap_i} with lids {lids} -> dominant cluster {dom}")

                print(f"Grouped captions by cluster (before selection):\n{groups}")

                # ----- 7) Pick top-N per cluster (by score) for diversity -----
                ##print("\n=== Step 7: Selecting top captions per cluster ===")

                # ---- Parameters (tune) ----
                TIME_SEP = 30.0            # min temporal separation (seconds) within a cluster’s chosen reps
                AUTO_MAX_PER_CLUSTER = 3   # hard cap (we’ll choose as many as pass TIME_SEP, up to this)
                MIN_PER_CLUSTER = 1        # always keep at least one if the cluster has any candidates

                representatives = []
                ref_time = getattr(self, 'time_offset', 0.0)
                def _ensure_o3d_aabb(bbox):
                                """
                                Convert to Open3D AxisAlignedBoundingBox if possible.
                                - Keep AABB as-is
                                - Convert OOBB -> AABB
                                - Convert Nx3 array -> AABB
                                """
                                if bbox is None:
                                    return None
                                if isinstance(bbox, o3d.geometry.AxisAlignedBoundingBox):
                                    return bbox
                                if isinstance(bbox, o3d.geometry.OrientedBoundingBox):
                                    return bbox.get_axis_aligned_bounding_box()
                                try:
                                    arr = np.asarray(bbox)
                                    if arr.ndim == 2 and arr.shape[1] == 3:
                                        return o3d.geometry.AxisAlignedBoundingBox(arr.min(axis=0), arr.max(axis=0))
                                except Exception:
                                    pass
                                return None
                # Build a compact per-cluster object summary string
                def _cluster_object_summary(cid, subset_local_ids):
                    members = clusters_subset[cid]
                    print(colored(f"Cluster {cid} has {members} members.", "cyan"))

                    parts = []
                    for m in members:
                        o = sub_objects[m]
                        # Prefer global id if present; fallback to any available identifier
                        oid = o.get('global_id', o.get('object_id', o.get('id', f"{subset_local_ids[m]}")))
                        cap = o.get('caption', o.get('desc', ''))

                        bbox  = _ensure_o3d_aabb(o.get("bbox", None))
                        if bbox is None:
                            print(colored(f"Warning: No bbox for object {oid} in cluster {cid}.", "yellow"))
                            continue

                        center = np.round(np.asarray(bbox.get_center(), dtype=float), 3)  # (cx, cy, cz)
                        extent = np.round(np.asarray(bbox.get_extent(), dtype=float), 3)  # (dx, dy, dz)

                        if center is None: center = "center=None"
                        if extent is None: extent = "extent=None"
                        if cap:
                            # parts.append(f"Object {oid}: caption= {cap}; center: {center}: extent:{extent}")
                            parts.append(f"Object {oid}: center: {center}")
                        else:
                            parts.append(f"[obj:{oid}] {center} {extent}")
                    if not parts:
                        return "Objects: (none)"
                    return "Task relavante objects: " + " | ".join(parts)

                for cid in keep_cluster_idxs:
                    items = groups.get(int(cid), [])
                    if not items:
                        # print(f"\n=== Cluster {cid} ===")
                        # print("No captions.")
                        continue

                    items_sorted = sorted(items, key=lambda t: t[1])  # adjust if similarity instead of distance

                    chosen = []
                    chosen_times = []
                    chosen_docs = []
                    for cap_i, sc in items_sorted:
                        doc = kept_docs[cap_i]
                        t_abs = _caption_time_seconds(doc, ref_time)
                        if all(abs(t_abs - prev) >= TIME_SEP for prev in chosen_times):
                            chosen.append((cid, cap_i, sc, t_abs))
                            chosen_times.append(t_abs)
                            chosen_docs.append(doc)
                        if len(chosen) >= AUTO_MAX_PER_CLUSTER:
                            break

                    if len(chosen) < MIN_PER_CLUSTER:
                        cap_i, sc = items_sorted[0]
                        doc = kept_docs[cap_i]
                        t_abs = _caption_time_seconds(doc, ref_time)
                        chosen = [(cid, cap_i, sc, t_abs)]
                        chosen_times = [t_abs]
                        chosen_docs = [doc]

                    chosen_set = set((cap_i for (_, cap_i, _, _) in chosen))
                    omitted_items = [(cap_i, sc) for (cap_i, _) in items_sorted if cap_i not in chosen_set]
                    omitted_docs = [kept_docs[cap_i] for (cap_i, _) in omitted_items]

                    omitted_summary = _summarize_omitted_times(omitted_docs, ref_time)
                    # NEW: Build object summary for this cluster (object ids, captions, bbox centers/extents)
                    # obj_summary = _cluster_object_summary(int(cid), subset_local_ids)

                    # Append both summaries to each selected doc's text
                    addon = f" {omitted_summary}" # {obj_summary}"
                    for doc in chosen_docs:
                        doc.page_content += addon
                    # Append omitted summary to each selected doc's text
                    # for doc in chosen_docs:
                    #     doc.page_content += f" {omitted_summary}"

                    #print(f"\n=== Cluster {cid} ===")
                    for _, cap_i, sc, t_abs in chosen:
                        print(f"  [SELECTED] cap={cap_i} score={sc:.3f} t={t_abs:.2f}s")

                    # print(colored(f"\nSelected captions for Cluster {cid}:", "yellow"))
                    # _ = self.memory_to_string(chosen_docs)  # will now include omitted summary at end

                    representatives.extend([(cid, cap_i, sc) for (_, cap_i, sc, _) in chosen])

                # Global cap across clusters
                representatives.sort(key=lambda t: t[2])   # lower distance = better
                representatives = representatives[:k]

                final_docs = [kept_docs[cap_i] for (_, cap_i, _) in representatives]
                final_docs_with_scores = [(kept_docs[cap_i], kept_scores[cap_i]) for (_, cap_i, _) in representatives]
                # final_docs = [doc for doc, _ in final_docs_with_scores]

                # # Selected indices (caption indices within the TOP_M pool)
                selected_indices = [cap_i for (_, cap_i, _) in representatives]
                selected_set = set(selected_indices)
                # print(colored(f"Selected caption indices: {selected_indices}"), "green")

                # # Build a full list for the top-M retrieved pool (times + original scores)
                time_offset = getattr(self, 'time_offset', 0.0)
                all_retrieved = []  # every retrieved item (top-M), marked if selected
                for cap_i, (doc, score) in enumerate(docs_with_scores):
                    mt = doc.metadata.get('time', 0.0)
                    t_val = mt[0] if isinstance(mt, (list, tuple)) else mt
                    all_retrieved.append({
                        "idx": cap_i,                                # caption index within the top-M pool
                        "time": float(t_val + time_offset),          # absolute time
                        "score": float(score),                       # original score from retriever
                        "selected": (cap_i in selected_set)          # whether it was picked as representative
                    })

                # Optional: also keep the final docs in working memory (comment out if you don't want this)
                # self.working_memory += final_docs

                # Persist to self.search_text
                if not hasattr(self, 'search_text'):
                    self.search_text = {}

                self.search_text[query] = {
                    "method_name": f"text_search-{query}",
                    # full top-M pool: times, scores, selection flags
                    "data": all_retrieved,
                    # the tuples you already produce: (cluster_id, caption_idx, score)
                    "representatives": representatives,
                    # the plain list of selected caption indices (easy to consume later)
                    "selected_indices": selected_set,
                }

                # visualize_highlighted_clusters_open3d(
                #     sub_objects, clusters_subset, keep_cluster_idxs, winners, [query],
                #     dim_alpha=0.10, save_dir=os.path.join(out_path, "clusters_vis"), show=True
                # )
                # visualize_graph_highlight(
                #     G_sub, clusters_subset, keep_cluster_idxs,
                #     out_png=os.path.join(out_path, "graph_highlight.png"), show=True
                # )
                return self.memory_to_string(final_docs)
            except Exception as e:
                import traceback
                print(colored(f"[search_by_text_IB] ERROR: {e}", "red"))
                traceback.print_exc()
                # graceful fallback to simple top-k
                docs = [doc for doc, _ in self.text_retriever.vectorstore.similarity_search_with_score(query, k=k)]
                self.working_memory += docs
                return self.memory_to_string(docs)


    def search_by_text_IB(self, query: str, k=8) -> str:
            """
            Retrieve many captions, restrict to those covering objects (object_id) related to the query,
            cluster the union subset via IB (task = query), and return one best caption per cluster (diverse).
            """
            initial_tops = self.args.topk
            try:
                # ---------- Tunables ----------
                TOP_M        = 100     # retrieve a big pool
                KEEP_THRESH  = 0.35    # normalized similarity threshold to keep captions
                TASK_FILTER_THRESH = 0.4  # SBERT task relevance after merging
                GRAPH_KW = dict(
                    vertical_axis='y', down_positive=True,
                    dist_radius=3.0, z_overlap=False, z_slack=0.40,
                    iou_thresh=0.01, covis_min=0, knn=6,
                    ground_height_thresh=0.1, ground_floor_thresh=0.05,
                    dilate_eps=0.02
                )

                # ---------- 1) Retrieve ----------
                docs_with_scores = self.text_retriever.vectorstore.similarity_search_with_score(query, k=TOP_M)
                if not docs_with_scores:
                    print(colored("Warning: No captions found for the query.", "red"))
                    return ""

                # Normalize scores to [0,1]
                raw_scores = [s for _, s in docs_with_scores]
                smin, smax = min(raw_scores), max(raw_scores)
                if smax - smin < 1e-9:
                    norm = [1.0 for _ in raw_scores]
                else:
                    # If vectorstore returns DISTANCE (smaller=better), convert to similarity
                    norm = [1.0 - (s - smin) / (smax - smin) for s in raw_scores]
                # print(colored(f"Normalized scores: {norm}", "yellow"))
                # TODO: can be removed
                keep_idx = [i for i, v in enumerate(norm) if v >= KEEP_THRESH]
                # print(colored(f"Initial keep indices (>= {KEEP_THRESH:.2f}): {keep_idx}", "yellow"))
                # idx_sorted = sorted(range(len(norm)), key=lambda i: norm[i], reverse=True)
                # keep_idx = idx_sorted[:min(10, len(idx_sorted))]
                # fallback: force top-10 (or fewer if not enough items)
                if len(keep_idx) < 12:
                    idx_sorted = sorted(range(len(norm)), key=lambda i: norm[i], reverse=True)
                    keep_idx = idx_sorted[:min(10, len(idx_sorted))]
                    # print top-10 norm scores
                    # print(colored(f"Top-10 norm scores: {[norm[i] for i in idx_sorted[:10]]}", "yellow"))

                print(colored(f"Keeping {len(keep_idx)} / {len(norm)} captions (>= {KEEP_THRESH:.2f})", "yellow"))
                if not keep_idx:
                    keep_idx = list(range(min(TOP_M, len(docs_with_scores))))
                #keep_idx = list(range(len(docs_with_scores)))
                kept_docs   = [docs_with_scores[i][0] for i in keep_idx]
                kept_scores = [docs_with_scores[i][1] for i in keep_idx]

                kept_times = [docs_with_scores[i][0].metadata.get('time', 0.0) for i in keep_idx]
                # for doc, score, t in zip(kept_docs, kept_scores, kept_times):
                #     t = t[0] if isinstance(t, (list, tuple)) else t
                #     t += self.time_offset
                #     t = localtime(t)
                #     t = strftime('%Y-%m-%d %H:%M:%S', t)
                #     print(f"  [KEPT-CAND] id={doc.metadata.get('id', 'unknown')} score={score:.3f} text='At {t}{doc.page_content}...'")
                kept_norm   = [norm[i] for i in keep_idx]

                # ---------- 2) Build subset of objects from object_id ----------
                cap_to_local_obj_ids = []
                subset_local_ids_set = set()
                for doc in kept_docs:
                    gids = _parse_object_ids(doc.metadata.get('object_id'))
                    lids = _global_ids_to_local_indices(gids, objid_to_idx=getattr(self, 'objid_to_idx', None))
                    lids = list(set(lids))
                    cap_to_local_obj_ids.append(lids)
                    subset_local_ids_set.update(lids)

                subset_local_ids = sorted(list(subset_local_ids_set))
                if len(subset_local_ids) == 0:
                    print(colored("No objects covered by captions; falling back to top-k plain retrieval.", "red"))
                    topk_docs = [doc for doc, _ in docs_with_scores[:k]]
                    self.working_memory += topk_docs
                    return self.memory_to_string(topk_docs)

                # ---------- 3) Subgraph + IB on subset ----------
                # NOTE: if your objects live in self.objects, replace self.scene_graph with self.objects below
                object_cap_features_np = np.array([obj['ft'].flatten() for obj in self.scene_graph])  # (N,D)
                # visualize_objects_org(self.scene_graph)
                print(colored(f"Building subgraph from {len(subset_local_ids)} objects", "blue"))
                background_terms = [
                    "wall", "ground", "floor", "ceiling", "pillar", "beam",
                    "staging area", "walkway", "window", "ramp", "shelf wall"
                ]
                # G_sub, sub_objects, sub_feats = _build_subgraph_from_indices_without_BG(
                #     self.scene_graph, object_cap_features_np, subset_local_ids, GRAPH_KW,
                #     sbert_model=self.sbert_model,                # 你的 SBERT 模型实例
                #     background_terms=background_terms,      # 背景词表
                #     bg_threshold=0.45,                      # 阈值可按需要微调
                #     verbose=True
                # )
                G_sub, sub_objects, sub_feats = _build_subgraph_from_indices(
                    self.scene_graph, object_cap_features_np, subset_local_ids, GRAPH_KW
                )
                visualize_objects_org(sub_objects)
                # Task feature from query (SBERT) -> (1, D)
                task_ft = _encode_query_sbert(self.sbert_model, query).detach().cpu().numpy()[None, :]

                # IB config
                out_path = getattr(self, 'out_dir', './_tmp')
                Path(out_path).mkdir(parents=True, exist_ok=True)
                cfg_yaml_path = os.path.join(out_path, "cluster_config.yaml")
                write_default_cluster_config(cfg_yaml_path)


                # IB clusters in subset index space
                clusters_subset = run_ib_clustering(sub_feats, task_ft, G_sub, cfg_yaml_path)
                print(colored(f"[AIB] Formed {len(clusters_subset)} clusters from {len(subset_local_ids)} primitives.", "cyan"))

                # ---------- 4) Merge + filter by task relevance ----------
                # clustered_objects = merge_all_clusters(sub_objects, clusters_subset)
                # # print(colored(f"[AIB] After merge: {len(clustered_objects)} objects.", "cyan"))

                # clustered_objects_filtered = filter_clusters_by_task(clustered_objects, task_ft, TASK_FILTER_THRESH)
                # # clustered_objects_filtered = filter_clusters_by_task_topk(clustered_objects, task_ft, topk=6)
                # print(colored(f"[AIB] Filtered to {len(clustered_objects_filtered)} task-relevant objects.", "cyan"))


                # ----- 5) Score clusters by task & keep only task-relevant ones -----
                # print("\n=== Step 5: Scoring clusters by task relevance ===")
                scores, winners = cluster_task_scores_cosine(sub_objects, clusters_subset, task_ft, pool="max")
                #print(f"Cluster scores vs task: {scores}")
                #print(f"Best matching task index per cluster: {winners}")

                HIGHLIGHT_THR = 0.4  # align with TASK_FILTER_THRESH

                keep_cluster_idxs = np.where(scores >= HIGHLIGHT_THR)[0]
                print(colored(f"Clusters above {HIGHLIGHT_THR:.2f}: {len(keep_cluster_idxs)}", "cyan"))
                keep_cluster_idxs = np.argsort(-scores)[:min(10, len(clusters_subset))]
                print(f"Keeping top {len(keep_cluster_idxs)} clusters by score. \n{keep_cluster_idxs}")
                print(f"Cluster scores: {[scores[cid] for cid in keep_cluster_idxs]}")

                # if keep_cluster_idxs.size < k:
                #     keep_cluster_idxs = np.argsort(-scores)[:k] #min(k, len(clusters_subset))
                #     print(f"No clusters above {HIGHLIGHT_THR:.2f}, keeping top {len(keep_cluster_idxs)} by score. \n{keep_cluster_idxs}")
                # else:
                #     keep_cluster_idxs = np.argsort(-scores)[:min(k, len(clusters_subset))]
                #     print(f"Keeping {len(keep_cluster_idxs)} clusters above {HIGHLIGHT_THR:.2f} \n{keep_cluster_idxs}")
                for cid in keep_cluster_idxs:
                    print(f"cluster {cid}: score={scores[cid]:.4f}")
                # ----- 6) Map captions → dominant kept cluster via object ids -----
                print("\n=== Step 6: Mapping captions to dominant clusters ===")
                subset_local_to_pos = {lid: pos for pos, lid in enumerate(subset_local_ids)}
                objpos_to_keptcid = {}
                for cid in keep_cluster_idxs:
                    for pos in clusters_subset[cid]:
                        objpos_to_keptcid[pos] = cid

                ##print(f"subset_local_to_pos: {subset_local_to_pos}")
                ##print(f"objpos_to_keptcid: {objpos_to_keptcid}")

                groups = {int(cid): [] for cid in keep_cluster_idxs}
                for cap_i, lids in enumerate(cap_to_local_obj_ids):
                    hits = []
                    for lid in lids:
                        pos = subset_local_to_pos.get(lid, None)
                        if pos is None:
                            continue
                        cid = objpos_to_keptcid.get(pos, None)
                        if cid is not None:
                            hits.append(int(cid))
                    if not hits:
                        continue
                    dom = max(set(hits), key=hits.count)
                    groups[dom].append((cap_i, kept_scores[cap_i]))
                    ##print(f"Under query: {query}; Caption {cap_i} with lids {lids} -> dominant cluster {dom}")

                print(f"Grouped captions by cluster (before selection):\n{groups}")

                # ----- 7) Pick top-N per cluster (by score) for diversity -----
                ##print("\n=== Step 7: Selecting top captions per cluster ===")
                # --- add near the top of Step 7 (before looping clusters) ---

                # (You already defined these above; keep as-is or tune.)
                W_CAP = 1         # caption text weight
                W_OBJ = 0.4         # object relevance weight
                W_CLUSTER = 0.5
                # W_CLUSTER already defined above

                representatives = []
                ref_time = getattr(self, 'time_offset', 0.0)

                # Optional: if your cosine helper expects vectors, task_ft[0] is fine.
                task_ft_vec = task_ft[0]

                for cid in keep_cluster_idxs:
                    items = groups.get(int(cid), [])
                    if not items:
                        continue

                    best = None  # (final_score, cap_i, cap_sim, obj_rel, t_abs)

                    for (cap_i, _raw_dist) in items:
                        # 1) caption similarity in [0,1]
                        cap_sim = float(kept_norm[cap_i])

                        # 2) absolute time of the caption
                        doc   = kept_docs[cap_i]
                        t_abs = _caption_time_seconds(doc, ref_time)

                        # 3) task-relevant object evidence near this caption (or fallback to caption-linked objects)
                        #    Your helper takes (cap_i, task_ft); it uses sub_feats + objpos_to_keptcid internally.
                        obj_rel = 0.0 #object_relevance_for_caption(cap_i, task_ft)  # returns [0,1]

                        # 4) caption-level composite
                        cap_comp = W_CAP * cap_sim + (1.0 - W_CAP) * obj_rel

                        # 5) final cluster-aware score
                        cscore = float(scores[cid])  # [0,1]
                        final  = W_CLUSTER * cscore + (1.0 - W_CLUSTER) * cap_comp

                        if (best is None) or (final > best[0]):
                            best = (final, cap_i, cap_sim, obj_rel, t_abs)

                    if best is None:
                        continue

                    final, cap_i, cap_sim, obj_rel, t_abs = best

                    # (Optional) add your omitted-times summary for context
                    omitted_docs   = [kept_docs[i0] for (i0, _) in items if i0 != cap_i]
                    omitted_summary = _summarize_omitted_times(omitted_docs, ref_time)
                    kept_docs[cap_i].page_content += f" {omitted_summary}"

                    print(f"[REP] cluster={cid} final={final:.3f} cap_sim={cap_sim:.3f} obj_rel={obj_rel:.3f} t={t_abs:.2f}s")

                    # store (cluster, caption, final_score)
                    representatives.append((int(cid), int(cap_i), float(final)))

                # ---- Global ranking across clusters; keep top-k ----
                representatives.sort(key=lambda t: t[2], reverse=True)  # larger final score is better
                representatives = representatives[:k]
                final_docs = [kept_docs[cap_i] for (_, cap_i, _) in representatives]



                # Global cap across clusters
                '''
                representatives.sort(key=lambda t: t[2])   # lower distance = better
                representatives = representatives[:k]

                final_docs = [kept_docs[cap_i] for (_, cap_i, _) in representatives]
                '''
                final_docs_with_scores = [(kept_docs[cap_i], kept_scores[cap_i]) for (_, cap_i, _) in representatives]
                # final_docs = [doc for doc, _ in final_docs_with_scores]

                # # Selected indices (caption indices within the TOP_M pool)
                selected_indices = [cap_i for (_, cap_i, _) in representatives]
                selected_set = set(selected_indices)
                # print(colored(f"Selected caption indices: {selected_indices}"), "green")

                # # Build a full list for the top-M retrieved pool (times + original scores)
                time_offset = getattr(self, 'time_offset', 0.0)
                all_retrieved = []  # every retrieved item (top-M), marked if selected
                for cap_i, (doc, score) in enumerate(docs_with_scores):
                    mt = doc.metadata.get('time', 0.0)
                    t_val = mt[0] if isinstance(mt, (list, tuple)) else mt
                    all_retrieved.append({
                        "idx": cap_i,                                # caption index within the top-M pool
                        "time": float(t_val + time_offset),          # absolute time
                        "score": float(score),                       # original score from retriever
                        "selected": (cap_i in selected_set)          # whether it was picked as representative
                    })

                # Optional: also keep the final docs in working memory (comment out if you don't want this)
                # self.working_memory += final_docs

                # Persist to self.search_text
                if not hasattr(self, 'search_text'):
                    self.search_text = {}

                self.search_text[query] = {
                    "method_name": f"cue: {query}",
                    # full top-M pool: times, scores, selection flags
                    "data": all_retrieved,
                    # the tuples you already produce: (cluster_id, caption_idx, score)
                    "representatives": representatives,
                    # the plain list of selected caption indices (easy to consume later)
                    "selected_indices": selected_set,
                }

                visualize_highlighted_clusters_open3d(
                    sub_objects, clusters_subset, keep_cluster_idxs, winners, [query],
                    dim_alpha=0.10, save_dir=os.path.join(out_path, "clusters_vis"), show=True, task_tf=task_ft
                )
                # visualize_graph_highlight(
                #     G_sub, clusters_subset, keep_cluster_idxs,
                #     out_png=os.path.join(out_path, "graph_highlight.png"), show=True
                # )
                return self.memory_to_string(final_docs)
            except Exception as e:
                import traceback
                print(colored(f"[search_by_text_IB] ERROR: {e}", "red"))
                traceback.print_exc()
                # graceful fallback to simple top-k
                docs = [doc for doc, _ in self.text_retriever.vectorstore.similarity_search_with_score(query, k=k)]
                self.working_memory += docs
                return self.memory_to_string(docs)

    # function for searching by text, which is more complex and involves clustering
    def search_by_text_IB_working(self, query: str, k=6) -> str:
        """
        Retrieve many captions, restrict to those covering objects (object_id) related to the query,
        cluster the union subset via IB (task = query), and return one best caption per cluster (diverse).
        """
        try:
            # ---------- Tunables ----------
            TOP_M        = 100     # retrieve a big pool
            KEEP_THRESH  = 0.45    # normalized similarity threshold to keep captions
            TASK_FILTER_THRESH = 0.55  # SBERT task relevance after merging
            GRAPH_KW = dict(
                vertical_axis='y', down_positive=True,
                dist_radius=3.0, z_overlap=False, z_slack=0.40,
                iou_thresh=0.01, covis_min=0, knn=6,
                ground_height_thresh=0.1, ground_floor_thresh=0.05,
                dilate_eps=0.02
            )

            # ---------- 1) Retrieve ----------
            docs_with_scores = self.text_retriever.vectorstore.similarity_search_with_score(query, k=TOP_M)
            if not docs_with_scores:
                print(colored("Warning: No captions found for the query.", "red"))
                return ""

            # Normalize scores to [0,1]
            raw_scores = [s for _, s in docs_with_scores]
            smin, smax = min(raw_scores), max(raw_scores)
            if smax - smin < 1e-9:
                norm = [1.0 for _ in raw_scores]
            else:
                # If vectorstore returns DISTANCE (smaller=better), convert to similarity
                norm = [1.0 - (s - smin) / (smax - smin) for s in raw_scores]

            keep_idx = [i for i, v in enumerate(norm) if v >= KEEP_THRESH]
            print(colored(f"Keeping {len(keep_idx)} / {len(norm)} captions (>= {KEEP_THRESH:.2f})", "yellow"))
            if not keep_idx:
                keep_idx = list(range(min(TOP_M, len(docs_with_scores))))

            kept_docs   = [docs_with_scores[i][0] for i in keep_idx]
            kept_scores = [docs_with_scores[i][1] for i in keep_idx]
            kept_norm   = [norm[i] for i in keep_idx]

            # ---------- 2) Build subset of objects from object_id ----------
            cap_to_local_obj_ids = []
            subset_local_ids_set = set()
            for doc in kept_docs:
                gids = _parse_object_ids(doc.metadata.get('object_id'))
                lids = _global_ids_to_local_indices(gids, objid_to_idx=getattr(self, 'objid_to_idx', None))
                lids = list(set(lids))
                cap_to_local_obj_ids.append(lids)
                subset_local_ids_set.update(lids)

            subset_local_ids = sorted(list(subset_local_ids_set))
            if len(subset_local_ids) == 0:
                print(colored("No objects covered by captions; falling back to top-k plain retrieval.", "red"))
                topk_docs = [doc for doc, _ in docs_with_scores[:k]]
                self.working_memory += topk_docs
                return self.memory_to_string(topk_docs)

            # ---------- 3) Subgraph + IB on subset ----------
            # NOTE: if your objects live in self.objects, replace self.scene_graph with self.objects below
            object_cap_features_np = np.array([obj['ft'].flatten() for obj in self.scene_graph])  # (N,D)
            print(colored(f"Building subgraph from {len(subset_local_ids)} objects", "blue"))
            G_sub, sub_objects, sub_feats = _build_subgraph_from_indices(
                self.scene_graph, object_cap_features_np, subset_local_ids, GRAPH_KW
            )

            # Task feature from query (SBERT) -> (1, D)
            task_ft = _encode_query_sbert(self.sbert_model, query).detach().cpu().numpy()[None, :]

            # IB config
            out_path = getattr(self, 'out_dir', './_tmp')
            Path(out_path).mkdir(parents=True, exist_ok=True)
            cfg_yaml_path = os.path.join(out_path, "cluster_config.yaml")
            write_default_cluster_config(cfg_yaml_path)

            # IB clusters in subset index space
            clusters_subset = run_ib_clustering(sub_feats, task_ft, G_sub, cfg_yaml_path)
            print(colored(f"[AIB] Formed {len(clusters_subset)} clusters from {len(subset_local_ids)} primitives.", "cyan"))

            # ---------- 4) Merge + filter by task relevance ----------
            clustered_objects = merge_all_clusters(sub_objects, clusters_subset)
            print(colored(f"[AIB] After merge: {len(clustered_objects)} objects.", "cyan"))

            clustered_objects_filtered = filter_clusters_by_task(clustered_objects, task_ft, TASK_FILTER_THRESH)
            print(colored(f"[AIB] Filtered to {len(clustered_objects_filtered)} task-relevant objects.", "cyan"))

            # ---------- 5) Group captions by IB cluster and pick one per cluster (diversity) ----------
            # map: subset local id -> subset position (0..Ns-1)
            subset_local_to_pos = {lid: pos for pos, lid in enumerate(subset_local_ids)}
            # map: obj subset pos -> cluster id
            objpos_to_cid = {}
            for cid, members in enumerate(clusters_subset):
                for pos in members:
                    objpos_to_cid[pos] = cid

            # group captions by the cluster they “hit” most through their object ids
            groups = {}
            for cap_i, lids in enumerate(cap_to_local_obj_ids):
                hits = []
                for lid in lids:
                    pos = subset_local_to_pos.get(lid, None)
                    if pos is None:
                        continue
                    cid = objpos_to_cid.get(pos, None)
                    if cid is not None:
                        hits.append(cid)
                if not hits:
                    continue
                cid = max(set(hits), key=hits.count)
                groups.setdefault(cid, []).append((cap_i, kept_scores[cap_i]))

            # pick best doc per cluster by score (assuming score is distance → lower is better)
            reps = []
            for cid, items in groups.items():
                best_cap_i, best_score = min(items, key=lambda t: t[1])
                reps.append((cid, best_cap_i, best_score))
            reps.sort(key=lambda t: t[2])
            reps = reps[:k]

            final_docs = [kept_docs[best_i] for (_, best_i, _) in reps]

            # Fill if < k (optional): add other strong-but-unused captions
            if len(final_docs) < k:
                used = set(id(d) for d in final_docs)
                for d, _ in docs_with_scores:
                    if id(d) not in used:
                        final_docs.append(d)
                    if len(final_docs) >= k:
                        break

            # ---------- 6) Optional diagnostics / visuals ----------
            # Print per cluster
            task_texts = [query]
            print_task_related_captions(sub_objects, clusters_subset, task_texts, task_ft, sim_threshold=0.45)

            # Cluster scoring/selection for highlight (doesn’t affect return)
            # scores, winners, _ = cluster_task_scores(sub_objects, clusters_subset, task_ft)
            # HIGHLIGHT_THR = 0.5
            # highlight_idxs = select_relevant_clusters(scores, HIGHLIGHT_THR)
            # visualize_highlighted_clusters_open3d(
            #     sub_objects, clusters_subset, highlight_idxs, winners, task_texts,
            #     dim_alpha=0.10,
            #     save_dir=os.path.join(out_path, "clusters_vis"),
            #     show=True
            # )
            # visualize_graph_highlight(
            #     G_sub, clusters_subset, highlight_idxs,
            #     out_png=os.path.join(out_path, "graph_highlight.png"),
            #     show=True
            # )

            # ---------- 7) Bookkeeping + plotting payload ----------
            self.working_memory += final_docs

            # Store plot data for all M retrieved caps
            data = []
            for (doc, score), n in zip(docs_with_scores, norm):
                mt = doc.metadata.get('time', 0.0)
                t_val = mt[0] if isinstance(mt, (list, tuple)) else mt
                data.append({
                    "time": t_val + getattr(self, 'time_offset', 0.0),
                    "score": score,
                    "score_normalized": n
                })
            if not hasattr(self, 'search_text'):
                self.search_text = {}
            self.search_text[query] = {
                "method_name": f"text_search-{query}",
                "data": data,
                "score_min": smin,
                "score_max": smax,
            }

            # ---------- 8) Return concise, diverse memory string ----------
            return self.memory_to_string(final_docs)

        except Exception as e:
            import traceback
            print(colored(f"[search_by_text_IB] ERROR: {e}", "red"))
            traceback.print_exc()
            # graceful fallback to simple top-k
            docs = [doc for doc, _ in self.text_retriever.vectorstore.similarity_search_with_score(query, k=k)]
            self.working_memory += docs
            return self.memory_to_string(docs)



    def search_by_text_hybrid(self, query: str, k =5) -> str:

        docs = self.text_retriever.invoke(query, k=k)
        # docs_with_scores = self.text_retriever.vectorstore.similarity_search_with_score(query, k)

        self.working_memory += docs
        memory_list = docs
        docs = self.memory_to_string(docs)

        """Look up things online."""
        return docs, memory_list



    ### Doc formatting for the last LLM
    def memory_to_string(self, memory_list: list[MemoryItem], ref_time: float=None):
        if ref_time == None:
            ref_time = self.time_offset
        # print(f"memory_list: {memory_list}")
        out_string = ""
        for doc in memory_list:
            if len(doc.metadata['time']) == 2:
                t = doc.metadata['time'][0]
            else:
                t = doc.metadata['time']

            if ref_time:
                t += ref_time
            t = localtime(t)
            t = strftime('%Y-%m-%d %H:%M:%S', t)

            content = doc.page_content or ""
            content = re.sub(r'\s+', ' ', content).strip()

            s = f"At time={t}, the robot was at an average position of {np.array(doc.metadata['position']).round(3).tolist()}."
            s += f"The robot saw the following: {content}\n\n"
            # s += f"The robot saw the following: {doc.page_content}\n\n"
            out_string += s
        # print(f"retrieved video captions: \n{out_string}")
        # print(colored(f"Retrieved video captions: \n{out_string}", "yellow"))
        print_to_cot_log(f"Retrieved video captions: \n{out_string}", self.cot_log_file, "yellow")
        return out_string

    def format_scene_objects_for_llm(self, objects: List[Dict]) -> str:
        """
        Format retrieved scene objects into a readable string for LLM consumption.
        Each object will be described with its caption, ID, and approximate location from bbox center.
        """
        if not objects:
            return "No scene objects matched the query."

        lines = ["Here are the top matched scene objects:"]
        for idx, obj in enumerate(objects, start=1):
            caption = obj.get('caption', 'unknown')
            obj_id = obj.get('obj_id', 'N/A')
            # time info
            times = obj.get('time', 'unknown')
            # time_dt = [datetime.fromtimestamp(t) for t in times]
            time_dt = [datetime.datetime.fromtimestamp(t).strftime('%Y-%m-%d %H:%M:%S') for t in times]
            intervals = self.merge_time_intervals(time_dt)
            description = self.format_intervals_compact(intervals)
            try:
                center = obj['bbox'].center  # Use OrientedBoundingBox center
                pos_str = f"at position [{center[0]:.2f}, {center[1]:.2f}, {center[2]:.2f}]"
            except Exception:
                pos_str = "with unknown position"
            lines.append(f"Object ID {obj_id}: Located at {pos_str}; \"{caption}\". detected at times {description}.") #f"{idx}. Object ID {obj_id}: \"{caption}\" {pos_str}. detected at times {time_dt}.")
            print(colored(f"Object ID {obj_id}: Located at {pos_str}; \"{caption}\". detected at times {description}.", "yellow"))

        return "\n".join(lines)

    # from datetime import datetime, timedelta

    def merge_time_intervals(self, time_list_str, max_gap_sec=1):
        times = sorted([datetime.datetime.strptime(t, '%Y-%m-%d %H:%M:%S') for t in time_list_str])
        if not times:
            return []

        intervals = []
        start = times[0]
        end = times[0]

        for curr in times[1:]:
            if (curr - end).total_seconds() <= max_gap_sec:
                end = curr
            else:
                intervals.append((start, end))
                start = curr
                end = curr
        intervals.append((start, end))

        return intervals

    def format_intervals_compact(self, intervals):
        if not intervals:
            return "No observation intervals found."

        first_day = intervals[0][0].strftime('%Y-%m-%d')
        lines = [f"The object was observed on {first_day} during:"]

        for s, e in intervals:
            if s.date() != e.date():
                # 如果跨天，则分别显示
                lines.append(f" {s.strftime('%Y-%m-%d %H:%M:%S')} ~ {e.strftime('%Y-%m-%d %H:%M:%S')}")
            elif s == e:
                lines.append(f" {s.strftime('%H:%M:%S')}")
            else:
                lines.append(f" {s.strftime('%H:%M:%S')} ~ {e.strftime('%H:%M:%S')}")

        return ",".join(lines)


    # def format_scene_objects_for_llm(self, objects: List[Dict]) -> str:
    #     """
    #     Format retrieved scene objects into a readable string for LLM consumption.
    #     Each object will be described with its caption, ID, and approximate location.
    #     """
    #     if not objects:
    #         return "No scene objects matched the query."

    #     lines = ["Here are the top matched scene objects:"]
    #     for idx, obj in enumerate(objects, start=1):
    #         caption = obj.get('caption', 'unknown')
    #         obj_id = obj.get('obj_id', 'N/A')
    #         try:
    #             pos = np.mean(obj['pcd_np'], axis=0).tolist()
    #             pos_str = f"at position {pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}"
    #         except Exception:
    #             pos_str = "with unknown position"
    #         lines.append(f"{idx}. Object ID {obj_id}: \"{caption}\" {pos_str}.")

    #     return "\n".join(lines)


    def set_scene_graph(self, scene_graph):
        """Attach a SceneGraph for scene object retrieval."""
        self.scene_graph = scene_graph


    # def search_scenegraph(self, query: str, top_k_scene=10) -> List[Dict]:
    #     """
    #     Search scene objects using SBERT features for matching.
    #     """

    #     # use sbert model to encode the query
    #     text_query_ft = self.sbert_model.encode([query], convert_to_tensor=True)  # List -> Tensor
    #     text_query_ft = text_query_ft / text_query_ft.norm(dim=-1, keepdim=True)   # normalize

    #     scored_objects = []
    #     for obj in self.scene_graph:
    #         if 'ft' not in obj or obj['ft'] is None:
    #             print(f"Object {obj.get('id', 'unknown')} does not have ft, skipping.")
    #             continue

    #         obj_ft = torch.tensor(obj['ft'], device=text_query_ft.device)
    #         obj_ft = obj_ft / obj_ft.norm(dim=-1, keepdim=True)  # normalize


    #         score = F.cosine_similarity(text_query_ft, obj_ft.unsqueeze(0), dim=-1).item()
    #         scored_objects.append((obj, score))

    #     scored_objects.sort(key=lambda x: -x[1])
    #     retrieved_objects = []
    #     retrieved_scores = []
    #     for obj, score in scored_objects[:top_k_scene]:
    #         retrieved_objects.append(obj)
    #         retrieved_scores.append(score)
    #     retrieved_objects = [obj for obj, score in scored_objects[:top_k_scene]]
    #     doc = self.format_scene_objects_for_llm(retrieved_objects)
    #     # print("docs for scene graph search: ", doc)
    #     return doc

    def search_scenegraph(self, query: str, top_k_scene=10) -> str:
        """
        Search scene objects using SBERT features for matching and store plotting data.
        """
        text_query_ft = self.sbert_model.encode([query], convert_to_tensor=True)
        text_query_ft = text_query_ft / text_query_ft.norm(dim=-1, keepdim=True)
        top_k_scene = self.args.topk
        scored_objects = []
        for obj in self.scene_graph:
            if 'ft' not in obj or obj['ft'] is None:
                print(f"Object {obj.get('id', 'unknown')} does not have ft, skipping.")
                continue

            obj_ft = torch.tensor(obj['ft'], device=text_query_ft.device)
            obj_ft = obj_ft / obj_ft.norm(dim=-1, keepdim=True)

            score = F.cosine_similarity(text_query_ft, obj_ft.unsqueeze(0), dim=-1).item()
            scored_objects.append((obj, score))

        scored_objects.sort(key=lambda x: -x[1])

        # Store plot data
        plot_data = []
        for idx, (obj, score) in enumerate(scored_objects[:top_k_scene]):
            print(f"{obj.get('caption', 'N/A')} - Score: {score:.4f}")
            times = obj.get('time', [])
            if not isinstance(times, list):
                times = [times]  # Make sure it's a list

            plot_data.append({
                "object_id": obj.get('obj_id', f"obj_{idx}"),
                "times": times,
                "score": score
            })
        # Save for downstream plotting
        # self.search_SG = {
        #     "method_name": "scenegraph_search",
        #     "data": plot_data
        # }
        if not hasattr(self, 'search_text'):
            self.search_text = {}

        self.search_SG[query] = {
        "method_name": f"SG_{query}",
        "data": plot_data,
        # "score_min": min_score,
        # "score_max": max_score,
        }

        retrieved_objects = [obj for obj, _ in scored_objects[:top_k_scene]]
        doc = self.format_scene_objects_for_llm(retrieved_objects)
        # print(colored(f"docs for scene graph search: \n, {doc}", "yellow"))
        return doc



def similarity_search_with_score_by_vector(
        pos_db,
        embedding: List[float],
        k: int = 4,
        param: Optional[dict] = None,
        expr: Optional[str] = None,
        timeout: Optional[float] = None,
        **kwargs: Any,
    ) -> List[Tuple[Document, float]]:
        """Perform a search on a query string and return results with score.

        For more information about the search parameters, take a look at the pymilvus
        documentation found here:
        https://milvus.io/api-reference/pymilvus/v2.2.6/Collection/search().md

        Args:
            embedding (List[float]): The embedding vector being searched.
            k (int, optional): The amount of results to return. Defaults to 4.
            param (dict): The search params for the specified index.
                Defaults to None.
            expr (str, optional): Filtering expression. Defaults to None.
            timeout (float, optional): How long to wait before timeout error.
                Defaults to None.
            kwargs: Collection.search() keyword arguments.

        Returns:
            List[Tuple[Document, float]]: Result doc and score.
        """
        if pos_db.col is None:
            print("No existing collection to search.")
            return []

        if param is None:
            param = pos_db.search_params

        # Determine result metadata fields with PK.
        output_fields = pos_db.fields[:]
        # output_fields.remove(pos_db._vector_field) # NOTE: Only thing removed
        timeout = pos_db.timeout or timeout
        # Perform the search.
        res = pos_db.col.search(
            data=[embedding],
            anns_field=pos_db._vector_field,
            param=param,
            limit=k,
            expr=expr,
            output_fields=output_fields,
            timeout=timeout,
            **kwargs,
        )
        # Organize results.
        ret = []
        for result in res[0]:
            data = {x: result.entity.get(x) for x in output_fields}
            doc = pos_db._parse_document(data)
            pair = (doc, result.score)
            ret.append(pair)

        return ret #[doc for doc, _ in ret]
