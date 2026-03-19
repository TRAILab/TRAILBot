import os
import torch
import numpy as np
import distinctipy
import open3d as o3d
import networkx as nx
import matplotlib.pyplot as plt

from agentic_rag.star.utils.utils import print_to_cot_log

def cluster_task_scores(objects, clusters, task_features):
    """
    objects: list of obj dicts with 'ft' (L2-normalized np/tensor)
    clusters: list[list[int]]
    task_features: (M, D) L2-normalized numpy array (SBERT)
    returns:
      scores: list[float] — max cosine(task, cluster_mean)
      winners: list[int] — argmax task index per cluster
      cluster_means: list[np.ndarray] — (D,) mean embedding per cluster
    """
    scores, winners, cluster_means = [], [], []
    for idxs in clusters:
        vecs = []
        for k in idxs:
            ft = objects[k]['ft']
            ft = ft.detach().cpu().numpy() if hasattr(ft, 'device') else np.asarray(ft)
            vecs.append(ft)
        if len(vecs) == 0:
            scores.append(0.0); winners.append(-1); cluster_means.append(None); continue
        mean = np.mean(np.stack(vecs), axis=0)
        mean = mean / (np.linalg.norm(mean) + 1e-12)
        sims = task_features @ mean  # (M,)
        j = int(np.argmax(sims))
        scores.append(float(sims[j]))
        winners.append(j)
        cluster_means.append(mean)
    return scores, winners, cluster_means

def select_relevant_clusters(scores, thr=0.35):
    """
    scores: list[float]
    thr: clusters with score >= thr are considered relevant
    returns: set of indices to highlight
    """
    return {i for i, s in enumerate(scores) if s >= thr}

def obb_to_lineset(obb, color=(0,0,0)):
    pts = np.asarray(obb.get_box_points())
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

def visualize_highlighted_clusters_open3d(
    objects, clusters, highlight_idxs, winners, task_texts,
    dim_alpha=0.05, save_dir=None, show=True, task_tf=None
):
    """
    - highlight_idxs: set of cluster indices to highlight
    - winners: per-cluster task index (from cluster_task_scores)
    - task_texts: list[str], used for console labels
    - dim_alpha: how much to dim non-relevant points (0..1)
    """
    geoms = []
    colors = distinctipy.get_colors(len(clusters))

    for ci, idxs in enumerate(clusters):
        is_hot = ci in highlight_idxs
        base_color = np.array(colors[ci])
        pcolor = base_color if is_hot else base_color * dim_alpha
        lcolor = base_color if is_hot else base_color * dim_alpha
        # print highlighted cluster
        if is_hot:
            print(f"[HIGHLIGHT] Cluster {ci} with {len(idxs)} objects, task: {task_texts[winners[ci]] if (winners[ci]>=0) else 'N/A'}")
        merged = o3d.geometry.PointCloud()
        for k in idxs:
            pcd = objects[k]['pcd']
            # print caption of each object in cluster
            caption = objects[k].get('caption', 'N/A')
            if is_hot:
                print(f"Object {k} has caption: {caption}")
            # calculate the score of each object relative to the task
            # obj_ft = objects[k]['ft']
            '''
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

            scored_objects.sort(key=lambda x: -x[1])'''

            pts = np.asarray(pcd.points)
            if pts.size == 0:
                continue
            col = np.tile(pcolor, (pts.shape[0], 1))
            pcd_col = o3d.geometry.PointCloud(pcd)  # copy
            pcd_col.colors = o3d.utility.Vector3dVector(col)
            geoms.append(pcd_col)
            merged += pcd

            # object bbox
            # geoms.append(obb_to_lineset(objects[k]['bbox'], color=lcolor))

        # cluster bbox
        # if len(merged.points) > 0:
        #     big = merged.get_oriented_bounding_box()
        #     big.color = lcolor
        #     geoms.append(big)

    # --- Add robot location as red dot ---
    robot_pos = None #(368.89, 39.6, 11.03)
    if robot_pos is not None:
        sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.5)  # small radius
        sphere.paint_uniform_color([1, 0, 0])  # red
        sphere.translate(robot_pos)
        geoms.append(sphere)

    if save_dir:
        os.makedirs(save_dir, exist_ok=True)
        for ci, idxs in enumerate(clusters):
            merged = o3d.geometry.PointCloud()
            for k in idxs:
                merged += objects[k]['pcd']
            o3d.io.write_point_cloud(os.path.join(save_dir, f"cluster_{ci:03d}.ply"), merged)

    if show:
        o3d.visualization.draw_geometries(geoms)




def visualize_graph_highlight(G_nx, clusters, highlight_idxs, out_png=None, show=True):
    pos = nx.spring_layout(G_nx, seed=42, k=0.25)
    hot_nodes, dim_nodes = [], []
    for ci, idxs in enumerate(clusters):
        (hot_nodes if ci in highlight_idxs else dim_nodes).extend(idxs)

    plt.figure(figsize=(10,10))
    nx.draw_networkx_edges(G_nx, pos, alpha=0.25, width=0.5)
    nx.draw_networkx_nodes(G_nx, pos, nodelist=dim_nodes, node_size=18, node_color="#BBBBBB", alpha=0.6)
    nx.draw_networkx_nodes(G_nx, pos, nodelist=hot_nodes, node_size=28, node_color="#3A86FF", alpha=0.95)
    plt.axis('off')
    if out_png:
        plt.savefig(out_png, bbox_inches='tight', dpi=200)
        plt.close()
    elif show:
        plt.show()





def objects_to_aabb_corners(objects, device='cpu'):
    pts = []
    for obj in objects:
        aabb = obj['bbox'].get_axis_aligned_bounding_box()
        pts.append(np.asarray(aabb.get_box_points(), dtype=np.float32))
    return torch.from_numpy(np.stack(pts)).to(device)  # (N,8,3)

def aabb_minmax_from_corners(corners):
    # corners: (N,8,3)
    mn, _ = corners.min(dim=1)  # (N,3)
    mx, _ = corners.max(dim=1)  # (N,3)
    return mn, mx

def pairwise_center_distance(centers):
    return torch.cdist(centers, centers, p=2)  # (N,N)

def compute_iou_batch(b1, b2):
    # Your batch IoU (AABB) function: b1=(M,8,3), b2=(N,8,3) -> (M,N)
    b1_min, b1_max = b1.min(1).values, b1.max(1).values
    b2_min, b2_max = b2.min(1).values, b2.max(1).values
    b1_min = b1_min[:,None,:]; b1_max = b1_max[:,None,:]
    b2_min = b2_min[None,:,:]; b2_max = b2_max[None,:,:]
    inter_min = torch.maximum(b1_min, b2_min)
    inter_max = torch.minimum(b1_max, b2_max)
    inter_vol = torch.prod(torch.clamp(inter_max - inter_min, min=0), dim=2)
    vol1 = torch.prod(b1_max - b1_min, dim=2)
    vol2 = torch.prod(b2_max - b2_min, dim=2)
    union = vol1 + vol2 - inter_vol + 1e-10
    return inter_vol / union  # (M,N)

def build_object_graph_smart(
    objects,
    region_features,
    *,
    device='cpu',
    dist_radius=3.0,           # meters (tight)
    z_overlap=False,            # require z-interval overlap
    z_slack=0.25,              # meters of slack on z if not using strict overlap
    iou_thresh=0.02,           # small but >0 to avoid “touching” via floor
    covis_min=2,               # co-visibility frames threshold; set 0 to disable
    knn=6,                     # sparsify to k nearest neighbors among gated pairs
    ignore_ground=True,        # don’t create edges from a ground node
    ground_height_thresh=0.05, # detect ground-ish by small thickness + low z
):
    """
    Builds a sparse, well-gated adjacency graph.

    objects[i] needs: 'bbox' (Open3D OBB), 'image_idx' (list of frame IDs), possibly 'caption'/'ft'
    region_features: (N,D) numpy used as node attr
    """
    N = len(objects)
    G = nx.Graph()
    for i, obj in enumerate(objects):
        G.add_node(
            i,
            position=np.asarray(obj['bbox'].center),
            semantic_feature=region_features[i].reshape(-1, 1),
            bounding_box=obj['bbox'],
        )
    if N <= 1: return G

    # Prepare geometry
    corners = objects_to_aabb_corners(objects, device=device)          # (N,8,3)
    mn, mx = aabb_minmax_from_corners(corners)                         # (N,3),(N,3)
    centers = (mn + mx) * 0.5
    extents = (mx - mn)                                                # (N,3)

    # Optional: label ground-ish nodes (thin in z and close to floor)
    is_ground = torch.zeros(N, dtype=torch.bool, device=device)
    if ignore_ground:
        z_thickness = extents[:, 2]
        z_base = mn[:, 2]
        is_ground = (z_thickness < ground_height_thresh) | (z_base < ground_height_thresh)

    # 1) distance gate
    D = pairwise_center_distance(centers)                              # (N,N)
    dist_mask = (D <= dist_radius)

    # 2) vertical criterion
    if z_overlap:
        # Overlap on z intervals
        zmin = mn[:, 2][:, None]; zmax = mx[:, 2][:, None]
        zmin2 = mn[:, 2][None, :]; zmax2 = mx[:, 2][None, :]
        inter_z = torch.minimum(zmax, zmax2) - torch.maximum(zmin, zmin2)
        z_mask = inter_z > 0
    else:
        # allow |z centers| within slack
        zc = centers[:, 2][:, None]
        zc2 = centers[:, 2][None, :]
        z_mask = (torch.abs(zc - zc2) <= z_slack)

    # 3) IoU gate (AABB)
    iou = compute_iou_batch(corners, corners)                          # (N,N)
    iou = torch.triu(iou, diagonal=1)                                  # keep upper triangle

    iou_mask = iou > iou_thresh

    # 4) co-visibility gate
    if covis_min > 0:
        # Build a small boolean matrix by set intersection counts
        covis = torch.zeros((N, N), dtype=torch.bool, device=device)
        img_sets = [set(objects[i].get('image_idx', [])) for i in range(N)]
        for i in range(N):
            Si = img_sets[i]
            # you can restrict j to neighborhood by dist_mask[i] to speed up:
            for j in range(i + 1, N):
                if len(Si.intersection(img_sets[j])) >= covis_min:
                    covis[i, j] = True
        covis_mask = covis
    else:
        covis_mask = torch.ones((N, N), dtype=torch.bool, device=device)

    # 5) ground hygiene: no edges from ground-ish nodes
    if ignore_ground:
        # build a mask that zeros rows and cols for ground nodes
        not_ground = ~is_ground
        ng_row = not_ground[:, None].expand(N, N)
        ng_col = not_ground[None, :].expand(N, N)
        ground_mask = ng_row & ng_col
    else:
        ground_mask = torch.ones((N, N), dtype=torch.bool, device=device)

    # Combine gates
    gate = dist_mask & z_mask & iou_mask & covis_mask & ground_mask    # (N,N), upper triangle only
    gate = torch.triu(gate, diagonal=1)

    # 6) sparsify by kNN on distance among the gated pairs
    if knn is not None and knn > 0:
        edges = []
        for i in range(N):
            # candidates j where gated and j>i
            mask_row = gate[i]
            js = torch.nonzero(mask_row, as_tuple=False).flatten()
            if js.numel() == 0:
                continue
            dij = D[i, js]
            # take k smallest distances
            k = min(knn, js.numel())
            topk = torch.topk(-dij, k).indices  # negative to get smallest
            chosen = js[topk]
            edges.extend([(i, int(j)) for j in chosen])
        G.add_edges_from(edges)
    else:
        ii, jj = torch.where(gate)
        G.add_edges_from([(int(i), int(j)) for i, j in zip(ii, jj)])

    return G
