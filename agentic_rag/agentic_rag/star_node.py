#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Minimal ROS2 wrapper for your working eval pipeline.

Goal:
- Keep your original logic intact.
- Only wrap it into a ROS2 Node so you can later subscribe to topics.
- Avoid removing "saving/logging" glue that your agents/memory depend on.

Subscribes:
  - /qa/query   (std_msgs/msg/String)

Publishes:
  - /qa/answer  (std_msgs/msg/String)   # JSON string: out_dict returned by answer_squad_question()

Side-effect:
  - writes /tmp/goal_pose.json (or configured path)
"""

import os
import sys
import json
import time
import glob
import gzip
import pickle
import traceback
from dataclasses import asdict
from types import SimpleNamespace
from typing import Optional, Any

import numpy as np
import torch
from termcolor import colored

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String as StringMsg

# ---------------------------------------------------------------------
# Your project imports (must exist in your repo)
# ---------------------------------------------------------------------
from agentic_rag.star.some_class.map_calss import MapObjectList

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
# Keep your original small utilities (unchanged behavior)
# ---------------------------------------------------------------------
def write_goal_to_file(pred_pos, path="/tmp/goal_pose.json"):
    goal_data = {"x": float(pred_pos[0]), "y": float(pred_pos[1]), "z": float(pred_pos[2])}
    with open(path, "w") as f:
        json.dump(goal_data, f)
    print(f"Saved predicted goal to {path}")


def evaluate_output(qa_instance, predicted, scenegrpahh=None, manual_qa=False):
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
                print(colored(f"Using predicted object id {predicted['object_id']} with position {pred_pos}", "yellow"))
            else:
                pred_pos = np.array(predicted["position"])
                print(colored(f"Using predicted position {pred_pos}", "yellow"))
        except Exception as e:
            print(e)
            pred_pos = np.array(predicted.get("position", [0, 0, 0]))
            print(colored(f"Using predicted position {pred_pos}", "yellow"))

        try:
            dist = np.linalg.norm(answer - pred_pos)
        except Exception as e:
            print(e)
            dist = 20.0

        out_error["position_error"] = float(dist)

    return out_error


def answer_squad_question(model, question, qa_instance=None, scenegrpahh=None, manual_qa=False):
    print(colored(f"Question: {question}", "green", attrs=["bold"]))

    start_time = time.time()
    response = model.query(question)  # key part
    elapsed = time.time() - start_time

    parsed = asdict(response)
    print(colored(f"Raw response: {parsed}", "blue"))

    out_error = evaluate_output(qa_instance, parsed, scenegrpahh, manual_qa)

    return_dict = {"response": parsed}
    return_dict.update(parsed)
    return_dict["error"] = out_error
    return_dict["elapsed"] = float(elapsed)
    return return_dict


def load_SG_data(args, fps=10):
    # unchanged logic from your script (minimal)
    scene_graph_path = os.path.join(args.data_dir, str(args.sequence_id), "pcd", f"{args.scenegraph_file}.pkl.gz")
    print("scene_graph_path\n", scene_graph_path)

    captions_path = os.path.join(args.data_dir, str(args.sequence_id), "caption", f"{args.caption_file}.json")
    with open(captions_path, "r") as f:
        out = json.load(f)
    all_start_times = np.array([float(x["file_start"][:-4]) for x in out])
    dataset_start_timestamp = all_start_times[0]

    with gzip.open(scene_graph_path, "rb") as f:
        results = pickle.load(f)

    if isinstance(results, dict):
        objects = MapObjectList()
        objects.load_serializable(results["objects"])

        if results.get("bg_objects", None) is None:
            _bg_objects = None
        else:
            _bg_objects = MapObjectList()
            _bg_objects.load_serializable(results["bg_objects"])

    elif isinstance(results, list):
        objects = MapObjectList()
        objects.load_serializable(results)
    else:
        raise ValueError("Unknown results type: ", type(results))

    for i, obj in enumerate(objects):
        objects[i]["caption"] = get_caption(obj["caption"], method="majority")
        objects[i]["image_idx"] = sorted(set(obj["image_idx"]))
        objects[i]["obj_id"] = i
        objects[i]["time"] = [dataset_start_timestamp + (idx * 10 / fps) for idx in obj["image_idx"]]

    objects_all = objects.copy()

    timestamps = None
    if isinstance(results, dict) and "timestamps" in results:
        timestamps = np.array(results["timestamps"])

    return objects, objects_all, timestamps


def load_memory(args, qa_instance=None, use_milvus=True, use_optimal_context=False, ip_address="127.0.0.1",
                embedder=None, objects=None, objects_all=None):
    captions_path = os.path.join(args.data_dir, str(args.sequence_id), "caption", f"{args.caption_file}.json")
    with open(captions_path, "r") as f:
        out = json.load(f)

    all_start_times = np.array([float(x["file_start"][:-4]) for x in out])
    all_end_times = np.array([float(x["file_end"][:-4]) for x in out])

    if args.all_mem:
        start_time = all_start_times[0]
        end_time = all_end_times[-1]
    else:
        # keep original behavior if you later use qa_instance
        start_time = np.float64(qa_instance["start_time"])
        end_time = qa_instance["end_time"]

    if use_milvus:
        memory = MilvusMemory(
            f"eval_memory_{args.sequence_id}",
            db_ip=ip_address,
            time_offset=start_time,
            embedder=embedder,
            args=args,                       # IMPORTANT: do NOT pass None
        )
    elif "vlm" in args.model:
        memory = VideoMemory()
    else:
        memory = TextMemory()

    memory.reset()

    outputs = []

    diff = all_start_times - start_time
    start_idx = int(np.argmin(np.abs(diff)))
    diff = all_end_times - end_time
    end_idx = int(np.argmin(np.abs(diff)))

    # these pkls used only for VideoMemory mode (keep intact)
    pkl_files = glob.glob(os.path.join(args.coda_dir, str(args.sequence_id), "*.pkl"))
    pkl_files.sort(key=lambda x: float(x.split("/")[-1][:-4]))

    # assign object ids
    out = assign_object_ids(out, objects)

    for i in range(start_idx, end_idx + 1):
        item = out[i]
        obj_id = item.get("object_id", None)
        if isinstance(obj_id, list):
            object_id = ",".join(map(str, obj_id))
        elif isinstance(obj_id, int):
            object_id = str(obj_id)
        elif obj_id is None:
            object_id = ""
        else:
            object_id = str(obj_id)

        entity = {
            "position": item["position"],
            "theta": item["rotation"],
            "time": item["times"],
            "caption": item["caption"][:3000],
            "object_id": object_id,
        }
        outputs.append(entity)

        if isinstance(memory, VideoMemory):
            entity = ImageMemoryItem.from_dict(entity)
        else:
            entity = MemoryItem.from_dict(entity)

        if use_milvus:
            memory.insert(entity, text_embedding=item.get("text_embedding", None))
            if args.all_mem:
                memory.set_scene_graph(objects_all)
            else:
                memory.set_scene_graph(objects)
        else:
            memory.insert(entity)

    return memory, outputs, float(all_start_times[0]), objects


# ---------------------------------------------------------------------
# ROS2 Node (minimal wrapper)
# ---------------------------------------------------------------------
class QAROSNode(Node):
    def __init__(self):
        super().__init__("remembr_qa_node")
        self.cb_group = ReentrantCallbackGroup()

        # Parameters (match your original defaults as much as possible)
        self.declare_parameter("base_dir", "/workspace/results/")
        self.declare_parameter("data_dir", "/workspace/results/")
        self.declare_parameter("coda_dir", "./coda_data/")
        self.declare_parameter("sequence_id", 2)

        self.declare_parameter("qa_file", "human_qa")
        self.declare_parameter("log_file", "test_log")
        self.declare_parameter("caption_file", "captions_NVILA-8B")
        self.declare_parameter("scenegraph_file", "full_pcd")

        self.declare_parameter("VDB", "search_DB")
        self.declare_parameter("SG", "search_SG")

        self.declare_parameter("model", "remembr+gpt-4.1")
        self.declare_parameter("postfix", "OpenMem_LONG_V19")
        self.declare_parameter("temperature", 0.0)
        self.declare_parameter("num_ctx", 8192 * 4)
        self.declare_parameter("window_size", 2)

        self.declare_parameter("db_ip", "127.0.0.1")
        self.declare_parameter("use_milvus", True)
        self.declare_parameter("all_mem", True)

        self.declare_parameter("goal_json_path", "/tmp/goal_pose.json")

        self.declare_parameter("query_topic", "/qa/query")
        self.declare_parameter("answer_topic", "/qa/answer")

        # Build an args-like object so your downstream code sees the same fields
        self.args = SimpleNamespace()
        self.args.base_dir = self.get_parameter("base_dir").value
        self.args.data_dir = self.get_parameter("data_dir").value
        self.args.coda_dir = self.get_parameter("coda_dir").value
        self.args.sequence_id = int(self.get_parameter("sequence_id").value)

        self.args.qa_file = self.get_parameter("qa_file").value
        self.args.log_file = self.get_parameter("log_file").value
        self.args.caption_file = self.get_parameter("caption_file").value
        self.args.scenegraph_file = self.get_parameter("scenegraph_file").value

        self.args.VDB = self.get_parameter("VDB").value
        self.args.SG = self.get_parameter("SG").value

        self.args.model = self.get_parameter("model").value
        self.args.postfix = self.get_parameter("postfix").value
        self.args.temperature = float(self.get_parameter("temperature").value)
        self.args.num_ctx = int(self.get_parameter("num_ctx").value)
        self.args.window_size = int(self.get_parameter("window_size").value)

        self.args.db_ip = self.get_parameter("db_ip").value
        self.args.use_milvus = bool(self.get_parameter("use_milvus").value)
        self.args.all_mem = bool(self.get_parameter("all_mem").value)

        self.goal_json_path = self.get_parameter("goal_json_path").value

        self.query_topic = self.get_parameter("query_topic").value
        self.answer_topic = self.get_parameter("answer_topic").value

        # Keep these paths (your original code uses them; even if you don’t right now,
        # some agent logic may rely on base_dir layout)
        self.latest_idx_path = os.path.join(self.args.base_dir, "latest_frame.txt")
        self.instruction_path = os.path.join(self.args.base_dir, "instructions", "instruction.json")

        # ROS I/O
        self.query_sub = self.create_subscription(
            StringMsg, self.query_topic, self.on_query, 10, callback_group=self.cb_group
        )
        self.answer_pub = self.create_publisher(StringMsg, self.answer_topic, 10)

        # Heavy init
        self.get_logger().info("Loading SBERT + embedder + scene graph + memory + agent ...")

        self.embedder = HuggingFaceEmbeddings(
            model_name="mixedbread-ai/mxbai-embed-large-v1",
            model_kwargs={"device": "cuda:0"},
        )
        self.sbert_model = SentenceTransformer("sentence-transformers/all-MiniLM-L6-v2").to("cuda")

        self.objects, self.objects_all, self.timestamp_list = load_SG_data(self.args, fps=10)

        # Build memory exactly like your eval pipeline expects (args is NOT None)
        self.memory, self.instance_captions, self.global_starttime, self.scene_graph = load_memory(
            self.args,
            qa_instance=None,
            use_milvus=self.args.use_milvus,
            use_optimal_context=False,
            ip_address=self.args.db_ip,
            embedder=[self.embedder, self.sbert_model],
            objects=self.objects,
            objects_all=self.objects_all,
        )

        # Build agent
        self.agent = self._build_agent(self.args.model)

        # Attach memory to agent in the same way as your while-loop
        self._attach_memory_to_agent(test_num=0)

        self.get_logger().info("✅ QA ROS node initialized and ready.")

    def _build_agent(self, model: str):
        if "remembr" in model:
            base_llm = model.split("+")[-1]
            return ReMEmbRAgent_AIB(llm_type=base_llm, num_ctx=self.args.num_ctx, temperature=self.args.temperature)
        if "vanila" in model:
            base_llm = model.split("+")[-1]
            return ReMEmbRAgent_VANILA(llm_type=base_llm, num_ctx=self.args.num_ctx, temperature=self.args.temperature)
        if "opengraph" in model:
            base_llm = model.split("+")[-1]
            return ReMEmbRAgent_SG(llm_type=base_llm, num_ctx=self.args.num_ctx, temperature=self.args.temperature)
        if "vlm" in model:
            return VLMNonAgent(llm_type="gpt-4o")
        return NonAgent(llm_type=model, num_ctx=self.args.num_ctx, temperature=self.args.temperature)

    def _attach_memory_to_agent(self, test_num: int):
        # replicate your original set_memory() usage
        if isinstance(self.agent, (ReMEmbRAgent_AIB, ReMEmbRAgent_VANILA, ReMEmbRAgent_SG)):
            self.agent.set_memory(
                self.memory,
                scene_graph=self.scene_graph,
                dataset_start_timestamp=self.global_starttime,
                sbert_model=self.sbert_model,
                test_num=test_num,
                args=self.args,                      # IMPORTANT
                timestamp_list=self.timestamp_list,
            )
        else:
            self.agent.set_memory(self.memory)

    def _prepare_cot_log(self, test_num: int) -> str:
        cot_dir = os.path.join(self.args.base_dir, "cot_log")
        os.makedirs(cot_dir, exist_ok=True)
        cot_log_file = os.path.join(cot_dir, f"cot_log_{test_num}.txt")
        with open(cot_log_file, "w") as f:
            f.write("Chain of Thought Log\n")
        return cot_log_file

    def on_query(self, msg: StringMsg):
        question = msg.data.strip()
        if not question:
            return

        self.get_logger().info(f"Received query: {question}")

        try:
            # Make this look like one step of your eval loop
            test_num = 0
            if hasattr(self.agent, "test_num"):
                self.agent.test_num = test_num

            cot_log_file = self._prepare_cot_log(test_num)
            if hasattr(self.agent, "cot_log_file"):
                self.agent.cot_log_file = cot_log_file
            if hasattr(self.agent, "memory") and hasattr(self.agent.memory, "cot_log_file"):
                self.agent.memory.cot_log_file = cot_log_file

            print_to_cot_log(message=f"Question received: {question}", log_file=cot_log_file)

            # Reset search logs if your plotting/saving relies on these keys existing
            if hasattr(self.agent, "memory"):
                if hasattr(self.agent.memory, "search_text"):
                    self.agent.memory.search_text = {}
                if hasattr(self.agent.memory, "search_SG"):
                    self.agent.memory.search_SG = {}
                if hasattr(self.agent.memory, "search_time"):
                    self.agent.memory.search_time = {}
                if hasattr(self.agent.memory, "search_position"):
                    self.agent.memory.search_position = {}

            # Query
            out_dict = answer_squad_question(
                self.agent,
                question,
                qa_instance=None,
                scenegrpahh=self.scene_graph,
                manual_qa=True,
            )

            agent_response = out_dict.get("response", {})
            pos = agent_response.get("position", None)

            if isinstance(pos, str):
                pos = eval(pos)
            if pos is None:
                pos = [0.0, 0.0, 0.0]

            pred_pos = np.array(pos, dtype=np.float32)

            # object_id override (same as your original)
            obj_id = agent_response.get("object_id", None)
            if obj_id is not None and obj_id != "" and self.scene_graph is not None:
                try:
                    if isinstance(obj_id, str):
                        # keep behavior close to your original code (eval-able)
                        obj_id_eval = eval(obj_id)
                        obj_id_int = int(obj_id_eval) if not isinstance(obj_id_eval, (list, tuple)) else int(obj_id_eval[0])
                    else:
                        obj_id_int = int(obj_id)
                    pred_pos = np.array(self.scene_graph[obj_id_int]["bbox"].center, dtype=np.float32)
                except Exception:
                    pass

            # Save goal json
            write_goal_to_file(pred_pos, path=self.goal_json_path)

            # Publish JSON answer
            self.answer_pub.publish(StringMsg(data=json.dumps(out_dict, ensure_ascii=False)))
            self.get_logger().info(f"Published answer. Predicted goal: {pred_pos.tolist()}")

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
