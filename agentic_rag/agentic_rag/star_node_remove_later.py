#!/usr/bin/env python3
"""
ROS2 QA Node (paste-ready)

Subscribes:
  - /qa/query   (std_msgs/msg/String)
  - /front_cam  (sensor_msgs/msg/Image)   # only used when use_visual:=true

Publishes (optional, but useful):
  - /qa/answer  (std_msgs/msg/String)  # JSON string of model response
"""

import os
import sys
import json
import time
import glob
import gzip
import pickle
import traceback
from termcolor import colored
from dataclasses import asdict
from typing import Optional, Any, Dict

import numpy as np
import torch

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String as StringMsg
from sensor_msgs.msg import Image as ImageMsg

# --- Optional: image conversion (only needed when use_visual==True) ---
try:
    from cv_bridge import CvBridge
    import cv2
    from PIL import Image as PILImage
    _HAS_CV_BRIDGE = True
except Exception:
    CvBridge = None
    cv2 = None
    PILImage = None
    _HAS_CV_BRIDGE = False


# ---------------------------------------------------------------------
# Your project imports (must exist in your repo)
# ---------------------------------------------------------------------
# If this file is placed under your package, you typically don't need sys.path hacks.
# If you do, uncomment and adjust as needed:
# parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
# if parent_dir not in sys.path:
#     sys.path.insert(0, parent_dir)

from agentic_rag.star.some_class.map_calss import MapObjectList  # your class

from langchain_huggingface import HuggingFaceEmbeddings
from sentence_transformers import SentenceTransformer

from agentic_rag.star.memory.memory import MemoryItem
from agentic_rag.star.memory.text_memory import TextMemory
from agentic_rag.star.memory.video_memory import VideoMemory, ImageMemoryItem
from agentic_rag.star.memory.milvus_memory_isaacsim import MilvusMemory

from agentic_rag.star.agents.non_agent import NonAgent
from agentic_rag.star.agents.vlm_non_agent import VLMNonAgent
from agentic_rag.star.agents.remembr_agent_aib import ReMEmbRAgent_AIB
from agentic_rag.star.agents.remembr_agent_vanila import ReMEmbRAgent_VANILA
from agentic_rag.star.agents.remembr_agent_sg import ReMEmbRAgent_SG

from agentic_rag.star.utils.util_isaacsim import (
    get_caption,
    assign_object_ids,
)

from agentic_rag.star.utils.utils import print_to_cot_log


# ---------------------------------------------------------------------
# Small utilities copied from your script (minimal set)
# ---------------------------------------------------------------------
def write_goal_to_file(pred_pos, path="/tmp/goal_pose.json"):
    goal_data = {"x": float(pred_pos[0]), "y": float(pred_pos[1]), "z": float(pred_pos[2])}
    with open(path, "w") as f:
        json.dump(goal_data, f)
    print(f"[QA_NODE] Saved predicted goal to {path}")


def evaluate_output(qa_instance, predicted, scenegrpahh=None, manual_qa=False):
    """
    Keep your original evaluator behavior. In ROS mode we often run manual_qa=True,
    but we leave this for compatibility.
    """
    out_error = {}
    if manual_qa or qa_instance is None:
        out_error["position_error"] = 0.0
        return out_error

    q_type = qa_instance["type"]
    if "position" in q_type:
        answer = np.array(qa_instance["answers"]["position"])

        if isinstance(predicted.get("position", None), str):
            predicted["position"] = eval(predicted["position"])

        try:
            if isinstance(predicted.get("object_id", None), str):
                predicted["object_id"] = eval(predicted["object_id"])

            if predicted.get("object_id", None) is not None:
                pred_pos = np.array(scenegrpahh[int(predicted["object_id"])]["bbox"].center)
            else:
                pred_pos = np.array(predicted["position"])
        except Exception:
            pred_pos = np.array(predicted.get("position", [0, 0, 0]))

        try:
            dist = np.linalg.norm(answer - pred_pos)
        except Exception:
            dist = 20.0

        out_error["position_error"] = float(dist)
        return out_error

    # Other types omitted for brevity; keep if needed.
    return out_error


def answer_squad_question(model, question, qa_instance=None, scenegrpahh=None, manual_qa=False):
    """
    Same structure as your version, but without infinite retry loops that can hang a ROS callback forever.
    """
    parsed = None
    start_time = time.time()

    response = model.query(question)  # your agent interface
    elapsed = time.time() - start_time

    parsed = asdict(response)  # dict like {"position":..., "object_id":..., ...}

    out_error = evaluate_output(qa_instance, parsed, scenegrpahh, manual_qa)

    return_dict = {"response": parsed}
    return_dict.update(parsed)
    return_dict["error"] = out_error
    return_dict["elapsed"] = float(elapsed)
    return return_dict


def load_SG_data(data_dir: str, sequence_id: int, scenegraph_file: str, caption_file: str, fps: int = 10):
    """
    Minimal port of your load_SG_data (no plotting).
    """
    scene_graph_path = os.path.join(data_dir, str(sequence_id), "pcd", f"{scenegraph_file}.pkl.gz")
    captions_path = os.path.join(data_dir, str(sequence_id), "caption", f"{caption_file}.json")
    print("scene_graph_path\n", scene_graph_path)
    print("captions_path\n", captions_path)

    with open(captions_path, "r") as f:
        out = json.load(f)
    all_start_times = np.array([float(x["file_start"][:-4]) for x in out])
    dataset_start_timestamp = all_start_times[0]

    with gzip.open(scene_graph_path, "rb") as f:
        results = pickle.load(f)

    if isinstance(results, dict):
        objects = MapObjectList()
        objects.load_serializable(results["objects"])
    elif isinstance(results, list):
        objects = MapObjectList()
        objects.load_serializable(results)
    else:
        raise ValueError(f"Unknown scenegraph results type: {type(results)}")

    for i, obj in enumerate(objects):
        objects[i]["caption"] = get_caption(obj["caption"], method="majority")
        objects[i]["image_idx"] = sorted(set(obj.get("image_idx", [])))
        objects[i]["obj_id"] = i
        objects[i]["time"] = [dataset_start_timestamp + (idx * 10 / fps) for idx in objects[i]["image_idx"]]

    objects_all = objects.copy()
    timestamps = np.array(results["timestamps"]) if isinstance(results, dict) and "timestamps" in results else None
    print(colored(f"{len(objects)} has been loaded!", "white", attrs=["dark"]))
    return objects, objects_all, timestamps


def load_memory(
    *,
    data_dir: str,
    coda_dir: str,
    sequence_id: int,
    caption_file: str,
    model_name: str,
    use_milvus: bool,
    db_ip: str,
    embedder,
    objects,
    objects_all,
    all_mem: bool = True,
):
    """
    Minimal port of your load_memory() for ROS runtime.
    Loads captions and inserts MemoryItems into your MilvusMemory/TextMemory.
    """
    captions_path = os.path.join(data_dir, str(sequence_id), "caption", f"{caption_file}.json")
    with open(captions_path, "r") as f:
        out = json.load(f)

    all_start_times = np.array([float(x["file_start"][:-4]) for x in out])
    all_end_times = np.array([float(x["file_end"][:-4]) for x in out])

    start_time = all_start_times[0]
    end_time = all_end_times[-1]

    if use_milvus:
        memory = MilvusMemory(f"eval_memory_{sequence_id}", db_ip=db_ip, time_offset=start_time, embedder=embedder, args=None)
    elif "vlm" in model_name:
        memory = VideoMemory()
    else:
        memory = TextMemory()

    memory.reset()

    # assign object ids into caption entries
    out = assign_object_ids(out, objects)

    start_idx = 0
    end_idx = len(out) - 1

    pkl_files = glob.glob(os.path.join(coda_dir, str(sequence_id), "*.pkl"))
    pkl_files.sort(key=lambda x: float(os.path.basename(x)[:-4]))

    outputs = []

    for i in range(start_idx, end_idx + 1):
        item = out[i]
        obj_id = item.get("object_id", None)
        if isinstance(obj_id, list):
            object_id = ",".join(map(str, obj_id))
        elif isinstance(obj_id, int):
            object_id = str(obj_id)
        else:
            object_id = ""

        entity_dict = {
            "position": item["position"],
            "theta": item["rotation"],
            "time": item["times"],
            "caption": item["caption"][:3000],
            "object_id": object_id,
        }
        outputs.append(entity_dict)

        if isinstance(memory, VideoMemory):
            # Keep as-is if you really use VideoMemory mode
            entity = ImageMemoryItem.from_dict(entity_dict)
        else:
            entity = MemoryItem.from_dict(entity_dict)

        if use_milvus:
            memory.insert(entity, text_embedding=item.get("text_embedding", None))
            if all_mem:
                memory.set_scene_graph(objects_all)
            else:
                memory.set_scene_graph(objects)
        else:
            memory.insert(entity)
    print(colored(f"Video caption length: {len(outputs)}", "white", attrs=["dark"]))

    return memory, outputs, float(all_start_times[0]), objects


# ---------------------------------------------------------------------
# ROS2 Node
# ---------------------------------------------------------------------
class QAROSNode(Node):
    def __init__(self):
        super().__init__("remembr_qa_node")
        self.cb_group = ReentrantCallbackGroup()

        # -------------------------
        # Parameters (set via launch or CLI)
        # -------------------------
        self.declare_parameter("data_dir", "/workspace/results/")
        self.declare_parameter("coda_dir", "./coda_data/")
        self.declare_parameter("sequence_id", 2)
        self.declare_parameter("caption_file", "captions_NVILA-8B")
        self.declare_parameter("scenegraph_file", "full_pcd")

        self.declare_parameter("model", "remembr+gpt-4.1")  # [vanila remembr opengraph][gpt-oss:20b gpt-4.1]
        self.declare_parameter("num_ctx", 8192 * 4)
        self.declare_parameter("temperature", 0.0)
        self.declare_parameter("window_size", 2)

        self.declare_parameter("use_milvus", True)
        self.declare_parameter("db_ip", "127.0.0.1")
        self.declare_parameter("all_mem", True)

        self.declare_parameter("use_visual", False)  # if True, uses latest /front_cam image in prompt
        self.declare_parameter("goal_json_path", "/tmp/goal_pose.json")

        # Topic names
        self.declare_parameter("query_topic", "/qa/query")
        self.declare_parameter("image_topic", "/front_cam")
        self.declare_parameter("answer_topic", "/qa/answer")

        # Read params
        self.data_dir = self.get_parameter("data_dir").get_parameter_value().string_value
        self.coda_dir = self.get_parameter("coda_dir").get_parameter_value().string_value
        self.sequence_id = self.get_parameter("sequence_id").get_parameter_value().integer_value
        self.caption_file = self.get_parameter("caption_file").get_parameter_value().string_value
        self.scenegraph_file = self.get_parameter("scenegraph_file").get_parameter_value().string_value

        self.model = self.get_parameter("model").get_parameter_value().string_value
        self.num_ctx = int(self.get_parameter("num_ctx").get_parameter_value().integer_value)
        self.temperature = float(self.get_parameter("temperature").get_parameter_value().double_value)
        self.window_size = int(self.get_parameter("window_size").get_parameter_value().integer_value)

        self.use_milvus = bool(self.get_parameter("use_milvus").get_parameter_value().bool_value)
        self.db_ip = self.get_parameter("db_ip").get_parameter_value().string_value
        self.all_mem = bool(self.get_parameter("all_mem").get_parameter_value().bool_value)

        self.use_visual = bool(self.get_parameter("use_visual").get_parameter_value().bool_value)
        self.goal_json_path = self.get_parameter("goal_json_path").get_parameter_value().string_value

        self.query_topic = self.get_parameter("query_topic").get_parameter_value().string_value
        self.image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        self.answer_topic = self.get_parameter("answer_topic").get_parameter_value().string_value

        # -------------------------
        # ROS interfaces
        # -------------------------
        self.query_sub = self.create_subscription(
            StringMsg, self.query_topic, self.on_query, 10, callback_group=self.cb_group
        )

        self.image_sub = self.create_subscription(
            ImageMsg, self.image_topic, self.on_image, 10, callback_group=self.cb_group
        )

        self.answer_pub = self.create_publisher(StringMsg, self.answer_topic, 10)

        # -------------------------
        # Image buffer
        # -------------------------
        self._bridge = CvBridge() if _HAS_CV_BRIDGE else None
        self._latest_pil: Optional[Any] = None
        self._latest_img_stamp: Optional[float] = None

        # -------------------------
        # Heavy initialization (models, memory, agent)
        # -------------------------
        self.get_logger().info("Loading SBERT + embedder + scene graph + memory + agent ...")

        # Embedder (Milvus text embeddings)
        self.embedder = HuggingFaceEmbeddings(
            model_name="mixedbread-ai/mxbai-embed-large-v1",
            model_kwargs={"device": "cuda:0"},
        )

        # SBERT reranker model used by your ReMEmbR agent
        self.sbert_model = SentenceTransformer("sentence-transformers/all-MiniLM-L6-v2").to("cuda")

        # Load scene graph
        self.objects, self.objects_all, self.timestamp_list = load_SG_data(
            data_dir=self.data_dir,
            sequence_id=self.sequence_id,
            scenegraph_file=self.scenegraph_file,
            caption_file=self.caption_file,
            fps=10,
        )

        # Build memory once
        self.memory, self.instance_captions, self.global_starttime, self.scene_graph = load_memory(
            data_dir=self.data_dir,
            coda_dir=self.coda_dir,
            sequence_id=self.sequence_id,
            caption_file=self.caption_file,
            model_name=self.model,
            use_milvus=self.use_milvus,
            db_ip=self.db_ip,
            embedder=[self.embedder, self.sbert_model],
            objects=self.objects,
            objects_all=self.objects_all,
            all_mem=self.all_mem,
        )

        # Build agent once
        self.agent = self._build_agent(self.model)

        # Attach memory to agent once
        self._attach_memory_to_agent()

        self.get_logger().info("✅ QA ROS node initialized and ready.")

    def _build_agent(self, model: str):
        if "remembr" in model:
            base_llm = model.split("+")[-1]
            agent = ReMEmbRAgent_AIB(llm_type=base_llm, num_ctx=self.num_ctx, temperature=self.temperature)
            self.get_logger().info("Using ReMEmbRAgent_AIB")
            return agent
        if "vanila" in model:
            base_llm = model.split("+")[-1]
            agent = ReMEmbRAgent_VANILA(llm_type=base_llm, num_ctx=self.num_ctx, temperature=self.temperature)
            self.get_logger().info("Using ReMEmbRAgent_VANILA")
            return agent
        if "opengraph" in model:
            base_llm = model.split("+")[-1]
            agent = ReMEmbRAgent_SG(llm_type=base_llm, num_ctx=self.num_ctx, temperature=self.temperature)
            self.get_logger().info("Using ReMEmbRAgent_SG")
            return agent
        if "vlm" in model:
            agent = VLMNonAgent(llm_type="gpt-4o")
            self.get_logger().info("Using VLMNonAgent")
            return agent

        agent = NonAgent(llm_type=model, num_ctx=self.num_ctx, temperature=self.temperature)
        self.get_logger().info("Using NonAgent")
        return agent

    def _attach_memory_to_agent(self):
        # Mirrors your evaluation loop’s set_memory()
        if isinstance(self.agent, (ReMEmbRAgent_AIB, ReMEmbRAgent_VANILA, ReMEmbRAgent_SG)):
            self.agent.set_memory(
                self.memory,
                scene_graph=self.scene_graph,
                dataset_start_timestamp=self.global_starttime,
                sbert_model=self.sbert_model,
                test_num=0,
                args=None,
                timestamp_list=self.timestamp_list,
            )
        else:
            self.agent.set_memory(self.memory)

    def on_image(self, msg: ImageMsg):
        if not self.use_visual:
            return

        if not _HAS_CV_BRIDGE:
            self.get_logger().warn("use_visual=True but cv_bridge is not available. Ignoring /front_cam.")
            return

        try:
            cv_img = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            rgb = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
            self._latest_pil = PILImage.fromarray(rgb)
            self._latest_img_stamp = self.get_clock().now().nanoseconds * 1e-9
        except Exception as e:
            self.get_logger().error(f"Failed to convert /front_cam image: {e}")

    def _build_visual_prompt(self, question: str) -> str:
        """
        You said: "subscribe /front_cam when use_visual is true".
        Your original code called GPT-4V to rewrite the question based on the image.
        Here we do a practical ROS-friendly version:

        - If you have a VLM pipeline already (your LLT / models.get_chatgpt_output), call it here.
        - Otherwise, we just append a small tag that an image is available,
          and let your downstream agent decide how to use it (if it supports images).
        """
        if not self.use_visual:
            return question
        
        if self._latest_pil is None:
            return question + "\n\n(Note: use_visual=True but no camera image received yet.)"

        # If your agent supports images directly, you should pass the image through your agent interface.
        # Since your current ReMEmbR agents look text-only in this script, we keep it text-compatible:
        return question + "\n\n(Visual context available from /front_cam at runtime.)"

    def on_query(self, msg: StringMsg):
        question = msg.data.strip()
        if not question:
            return

        # Run heavy LLM call in-place (MultiThreadedExecutor recommended)
        self.get_logger().info(f"Received query: {question}")

        try:
            q = question #self._build_visual_prompt(question)

            # Reset search logs if your memory uses them
            if hasattr(self.agent, "memory") and hasattr(self.agent.memory, "search_text"):
                self.agent.memory.search_text = {}

            out_dict = answer_squad_question(
                self.agent,
                q,
                qa_instance=None,          # no dataset QA instance in ROS runtime
                scenegrpahh=self.scene_graph,
                manual_qa=True,            # ROS mode
            )

            agent_response = out_dict.get("response", {})
            pos = agent_response.get("position", None)

            if isinstance(pos, str):
                pos = eval(pos)
            if pos is None:
                pos = [0.0, 0.0, 0.0]

            pred_pos = np.array(pos, dtype=np.float32)

            # If object_id returned, use object bbox center (same as your code)
            obj_id = agent_response.get("object_id", None)
            if obj_id is not None and obj_id != "" and self.scene_graph is not None:
                try:
                    if isinstance(obj_id, str):
                        obj_id_int = int(eval(obj_id)) if obj_id.strip().isdigit() is False else int(obj_id)
                    else:
                        obj_id_int = int(obj_id)
                    pred_pos = np.array(self.scene_graph[obj_id_int]["bbox"].center, dtype=np.float32)
                except Exception:
                    pass

            # Save goal json
            write_goal_to_file(pred_pos, path=self.goal_json_path)

            # Publish full response as JSON string (debug)
            self.answer_pub.publish(StringMsg(data=json.dumps(out_dict, ensure_ascii=False)))

            self.get_logger().info(f"Answer published. Predicted goal: {pred_pos.tolist()}")

        except Exception as e:
            self.get_logger().error(f"Failed to answer query: {e}")
            traceback.print_exc()


def main():
    rclpy.init()
    node = QAROSNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
