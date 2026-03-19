import re
from datetime import datetime
from time import mktime, strptime

import matplotlib.pyplot as plt
import matplotlib.dates as mdates
from termcolor import colored
import numpy as np

def get_caption(caption, method='longest'):
    caption_string = caption.strip()
    captions = caption_string.split(".,")
    captions = [c.strip() for c in captions if c.strip()]
    if method == 'longest':
        caption = max(captions, key=lambda x: len(x))
    else:# take the last one
        caption = captions[-1]
    return caption    

def assign_frame_indices(out, all_start_times, all_end_times, duration=3.0, fps=10):
    """
    Assign frame indices to each caption based on start and end times.

    Args:
        out (list of dict): List of caption dictionaries containing 'file_start' and 'file_end' keys.
        frame_interval (float): Time interval between frames, default is 1.0 second.

    Returns:
        list of dict: Updated list where each dict has an additional 'frame_idx' key.
    """

    # Global start and end times
    global_start_time = all_start_times[0]
    global_end_time = all_end_times[-1]
    # print("global_start_time:", global_start_time, "global_end_time:", global_end_time, "time duration:", global_end_time - global_start_time)
    # Assign frame indices to each caption
    for i in range(0, len(out)):
        start_time = all_start_times[i]-global_start_time#all_start_times[i+1]-global_start_time
        #end_time = all_start_times[i+1]-global_start_time#all_end_times[i+1] - global_start_time
        # print("start_time:", start_time, "end_time:", end_time)      
        frame_indices = list(range(np.ceil(start_time).astype(int), np.ceil(start_time+duration).astype(int)))
        # print("frame_list:", frame_indices)
        out[i]["frame_idx"] = frame_indices
    return out

def assign_object_ids(out, objects):
    """
    Assign object IDs to each caption in 'out' based on the frames it covers.

    Args:
        out (list of dict): List of caption dictionaries containing 'frame_idx' key.
        objects (list of dict): List of object dictionaries, each with 'obj_id' and 'image_idx' key.

    Returns:
        list of dict: Updated 'out' with an additional 'object_id' key for each caption.
    """
    from collections import defaultdict
    # Step 1: Build a frame-to-object mapping
    frame_to_objects = defaultdict(set)

    for obj in objects:
        obj_id = obj['obj_id']
        frames = obj.get('image_idx', [])  # Use 'image_idx' field for frames
        for frame in frames:
            frame_to_objects[frame].add(obj_id)
    # Step 2: Assign object IDs to each caption based on covered frames
    for i, x in enumerate(out):
        covered_obj_ids = set()
        for frame in x['frame_idx']:
            covered_obj_ids.update(frame_to_objects.get(frame, []))
        # Save the collected object IDs as a list
        x['object_id'] = sorted(list(covered_obj_ids))  # Optional: sorted for consistent order
    return out

def extract_gt_times(qa_instance: dict) -> dict:
    """
    Extract ground truth time information from a QA instance.

    Args:
        qa_instance (dict): The QA item containing question, type, context, and answers.

    Returns:
        dict: A dictionary with:
            - 'gt_type': 'single', 'range', or 'none'
            - 'timestamps': list of timestamps (e.g., [t1] or [t1, t2]) in epoch format
    """
    q_type = qa_instance.get('type', '')
    context = qa_instance.get('context', '')

    # Define a regex pattern to extract time strings like '2023-01-16 10:56:33'
    time_pattern = r'At time=([\d\-: ]+),'

    # Find all time mentions in the context
    matched_times = re.findall(time_pattern, context)

    if q_type != 'duration': #'time' or q_type == 'position':
        if matched_times:
            # Take the first timestamp found as the GT
            time_str = matched_times[0]
            # Convert to epoch timestamp
            epoch_time = mktime(strptime(time_str, '%Y-%m-%d %H:%M:%S'))
            return {'gt_type': 'single', 'timestamps': [epoch_time]}
        else:
            return {'gt_type': 'none', 'timestamps': []}

    elif q_type == 'duration':
        if len(matched_times) >= 2:
            # Take first and last timestamp as the range
            start_str = matched_times[0]
            end_str = matched_times[-1]
            start_epoch = mktime(strptime(start_str, '%Y-%m-%d %H:%M:%S'))
            end_epoch = mktime(strptime(end_str, '%Y-%m-%d %H:%M:%S'))
            return {'gt_type': 'range', 'timestamps': [start_epoch, end_epoch]}
        else:
            return {'gt_type': 'none', 'timestamps': []}

    else:
        return {'gt_type': 'none', 'timestamps': []}
    
def plot_multi_method_scores(plot_data_list, k=6, gt_info=None, title="Retrieved Video Captions", save_path=None):
    """
    Plot multiple retrieval method scores over time, with top-k highlighted.

    Args:
        plot_data_list (list of dict): 
            Each dict: {
                'method_name': str,
                'data': [{'time': float or array, 'score': float}, ...]
            }
        k (int): Number of top results to highlight (default 5)
        gt_info (dict, optional): {
            'gt_type': 'single' or 'range',
            'timestamps': [t1] or [t1, t2]
        }
        title (str): Plot title.
    """
    shape_styles = ['o', 's', '^', 'D', 'P', '*', 'X']  # different marker shapes

    # Plot
    fig, ax = plt.subplots(figsize=(14, 4))

    for idx, plot_data in enumerate(plot_data_list):
        method = plot_data['method_name']
        data = plot_data['data']

        if len(data) == 0:
            print(colored(f"Warning: No data for method '{method}', skipping.", "grey"))
            # print(f"Warning: No data for method '{method}', skipping.")
            continue

        # Extract times and scores
        times = []
        scores = []
        for item in data:
            t = item['time']
            if isinstance(t, (list, np.ndarray)):
                t = t[0]  # take the first value if array
            times.append(t)
            scores.append(item['score'])
        times = np.array(times)
        scores = np.array(scores)

        # Normalize: higher = better
        min_score = 0.0 #np.min(scores)
        max_score = 400 # np.max(scores)
        max_score = max(400, np.max(scores))  # Avoid division by zero
        if np.max(scores) > 400:
            print(colored(f"Warning: max score {np.max(scores)} is greater than 400, setting to 400!!!!!", "red"))
        print(f"min_score: {min_score}, max_score: {max_score}")
        norm_scores = 1 - (scores - min_score) / (max_score - min_score + 1e-8)

        # Get top-k indices (most relevant)
        top_k_indices = np.argsort(scores)[:k]

        # Convert to datetime objects
        times_dt = [datetime.fromtimestamp(ts) for ts in times]

        # Plot non-top-k (blue)
        mask = np.ones_like(scores, dtype=bool)
        mask[top_k_indices] = False
        plt.scatter(np.array(times_dt)[mask], norm_scores[mask],
                    marker=shape_styles[idx % len(shape_styles)],
                    color='blue', alpha=0.6, label=f"{method} (other)")

        # Plot top-k (red)
        plt.scatter(np.array(times_dt)[top_k_indices], norm_scores[top_k_indices],
                    marker=shape_styles[idx % len(shape_styles)],
                    color='red', alpha=0.9, label=f"{method} (top {k})")

    # Plot ground truth
    if gt_info:
        if gt_info['gt_type'] == 'single':
            for ts in gt_info['timestamps']:
                plt.axvline(datetime.fromtimestamp(ts), color='green', linestyle='--', label='GT Time')
        elif gt_info['gt_type'] == 'range':
            start_ts, end_ts = gt_info['timestamps']
            plt.axvline(datetime.fromtimestamp(start_ts), color='green', linestyle='--', label='GT Start')
            plt.axvline(datetime.fromtimestamp(end_ts), color='orange', linestyle='--', label='GT End')

    # Format x-axis to HH:MM:SS
    ax = plt.gca()
    ax.xaxis.set_major_formatter(mdates.DateFormatter('%H:%M:%S'))
    locator = mdates.SecondLocator(interval=30)  # Try changing to 10, 20, 60 as needed
    ax.xaxis.set_major_locator(locator)
    # ax.xaxis.set_major_locator(mdates.AutoDateLocator(maxticks=10))

    plt.xlabel("Time (HH:MM:SS)")
    plt.ylabel("Normalized Similarity (L2 Distance)")
    plt.title(title)
    plt.legend()
    plt.grid(True)
    # Improve x-axis readability
    fig.autofmt_xdate()
    plt.tight_layout()
    # plt.show()
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"Plot saved to {save_path}")
    else:
        plt.show()

def plot_scenegraph_scores(sg_data, gt_info=None, title="Scene Graph Search: object Scores vs Time", save_path=None):
    if not sg_data:
        print("No sg_data provided. Skipping plot.")
        return

    fig, ax = plt.subplots(figsize=(14, 5))

    markers = ['o', 's', '^', 'v', 'D', 'P', '*', 'X', 'h', '+']
    color_cycle = plt.rcParams['axes.prop_cycle'].by_key()['color']

    for sg_idx, entry in enumerate(sg_data):
        method_name = entry.get("method_name", f"SG_{sg_idx}")
        data = entry.get("data", [])

        if not data:
            continue

        for obj_idx, obj_data in enumerate(data):
            obj_id = obj_data['object_id']
            times = obj_data['times']
            score = obj_data['score']

            if not times:
                continue

            time_dt = [datetime.fromtimestamp(t) for t in times]
            scores = [score] * len(times)

            label = f"{method_name}: {obj_id}"
            ax.scatter(
                time_dt,
                scores,
                label=label,
                marker=markers[obj_idx % len(markers)],
                color=color_cycle[sg_idx % len(color_cycle)],
                s=80
            )

    # ✅ Plot GT info if available
    if gt_info:
        timestamps = gt_info.get('timestamps', [])
        gt_type = gt_info.get('gt_type', '')

        if gt_type == 'single':
            for ts in timestamps:
                ax.axvline(datetime.fromtimestamp(ts), color='green', linestyle='--', label='GT Time')
        elif gt_type == 'range' and len(timestamps) == 2:
            ax.axvline(datetime.fromtimestamp(timestamps[0]), color='green', linestyle='--', label='GT Start')
            ax.axvline(datetime.fromtimestamp(timestamps[1]), color='orange', linestyle='--', label='GT End')
        else:
            print("GT info provided but format is not recognized.")

    if not ax.collections:
        print("No valid points were plotted. Skipping figure.")
        plt.close(fig)
        return

    ax.xaxis.set_major_formatter(mdates.DateFormatter('%H:%M:%S'))
    locator = mdates.SecondLocator(interval=60)
    ax.xaxis.set_major_locator(locator)

    ax.set_xlabel('Time')
    ax.set_ylabel('Similarity Score')
    ax.set_title(title)

    ax.legend(
        title='Query + Object ID',
        bbox_to_anchor=(0.5, -0.25),
        loc='upper center',
        ncol=6,
        fontsize=9
    )

    plt.grid(True)
    fig.autofmt_xdate()
    plt.tight_layout()

    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"Plot saved to {save_path}")
    else:
        plt.show()