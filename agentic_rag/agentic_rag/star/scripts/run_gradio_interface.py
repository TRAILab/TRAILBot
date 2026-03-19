import re
import sys
sys.path.append(sys.path[0] + '/..')
import glob
import shutil
import yaml
from threading import Event
from string import Template

from uuid import uuid4
import gradio as gr
import hydra
import os
import json
from functools import partial
from PIL import Image, ImageDraw

# def load_config_with_variables(file_path):
#     with open(file_path, 'r') as f:
#         content = f.read()
    
#     # Get initial config without variable substitution
#     initial_config = yaml.safe_load(content)
#     # Create template and substitute
#     template = Template(content)
#     substituted_content = template.safe_substitute(initial_config)
#     # Load final config with substituted variables
#     final_config = yaml.safe_load(substituted_content)

#     return final_config

# def load_config_with_variables(file_path: str, max_iterations:int = 10) -> dict:
#     with open(file_path, 'r') as f:
#         content: dict = f.read()

#     initial_config = yaml.safe_load(content)
    
#     current_content: dict = content
#     previous_content: dict = None
#     iteration: int = 0
#     while current_content != previous_content and iteration < max_iterations:
#         previous_content = current_content
#         template = Template(current_content)
#         current_content = template.safe_substitute(initial_config)
#         iteration += 1

#     final_config: dict = yaml.safe_load(current_content)
    # return final_config

# === CONFIG ===
# cfg = load_config_with_variables("./configs/config.yaml")
# cfg = cfg['defaults']
# print(cfg)
# for i, kv in enumerate(cfg):
#     if 'inference' in kv:
#         cfg = kv
#     break 

# print("Loaded config:", cfg)

# POSTFIX= cfg["postfix"] # "STaR_LONG_V19"
# BASE_PATH = cfg["base_path"] # "/home/trailbot/RAG/results/103" # "/home/trailbot/Documents/data_process_Kitti/results/04"
# INSTRUCTION_PATH = cfg["instruction_path"] # os.path.join(BASE_PATH, "instructions", "instruction.json")
# LATEST_IDX_PATH = cfg["latest_idx_path"] # os.path.join(BASE_PATH, "latest_frame.txt")
# RGB_FOLDER = cfg["rgb_folder"] # os.path.join(BASE_PATH, "capture")

# LABEL_FOLDER = cfg["label_folder"] # os.path.join(BASE_PATH, "labels")  # Make sure to save labels as labels_{idx}.txt

# RETRIEVE_FOLDER = cfg["retrieve_folder"] # os.path.join(BASE_PATH, "search_DB", POSTFIX)
# RETRIEVE_ANNOTATED_FOLDER = cfg["retrieve_annotated_folder"] # os.path.join(BASE_PATH, "images", POSTFIX)
# COT_LOG_FOLDER = cfg["cot_log_folder"] # os.path.join(BASE_PATH, "cot_log")  # now a folder

# SEMANTIC_FOLDER = cfg["semantic_folder"] # os.path.join(BASE_PATH, "seg_map")
# OCCUPANCY_FOLDER = cfg["occupancy_folder"] # os.path.join(BASE_PATH, "occ_map")
# COST_FOLDER = cfg["cost_folder"] # os.path.join(BASE_PATH, "cost_map")
# FALLBACK_IMAGE_PATH = cfg["fallback_image_path"] # os.path.join(BASE_PATH, "fallback_image.png")  # Fallback image if no data exists
# UPDATE_FLAG_PATH = cfg["update_flag_path"] # os.path.join(BASE_PATH, "update", "update.txt")

def retrieve_fallback_image(fallback_image_path):
    if os.path.exists(fallback_image_path):
        return Image.open(fallback_image_path)
    else:
        # Dynamically create a fallback image with text
        get_null_image()

def get_null_image():
    # Dynamically create a fallback image with text
    img = Image.new("RGB", (256, 256), color="gray")
    draw = ImageDraw.Draw(img)
    draw.text((70, 120), "No Data", fill="white")
    return img

def change_entity_name(idx, cfg):
    uuid = uuid4().hex
    retrieve_db_path = os.path.join(cfg["retrieve_folder"], f"retrieval_DB_{idx}_{cfg['postfix']}.png")
    retrieve_annotated_path = os.path.join(cfg["retrieve_annotated_folder"], f"{idx}")
    cot_log_path = os.path.join(cfg["cot_log_folder"], f"cot_log_{idx}.txt")

    change_list = [retrieve_db_path, retrieve_annotated_path, cot_log_path]
    for path in change_list:
        if os.path.exists(path):
            change_name_by_idx(path, uuid)
        # else:
        #     print(f"Path does not exist, skipping rename: {path}")

def change_name_by_idx(path, uuid):
    base, ext = os.path.splitext(path)
    new_path = f"{base}_{uuid}{ext}"
    os.rename(path, new_path)

def init_folders(cfg):
    folder_list = [
        cfg["rgb_folder"],
        cfg["retrieve_annotated_folder"],
        cfg["retrieve_folder"],
        cfg["cot_log_folder"],
    ]

    for folder in folder_list:
        os.makedirs(folder, exist_ok=True)          
        delete_files(folder)          

    os.makedirs(os.path.dirname(cfg["instruction_path"]), exist_ok=True)
    with open(cfg["instruction_path"], "w") as f:
        json.dump({"trigger": False, "command": ""}, f)

    os.makedirs(os.path.dirname(cfg["latest_idx_path"]), exist_ok=True)
    with open(cfg["latest_idx_path"], "w") as f:
        f.write("-1")

    # Create Updated

def delete_files(folder_path):
    for filename in os.listdir(folder_path):
        file_path = os.path.join(folder_path, filename)

        if os.path.isfile(file_path):
            os.remove(file_path)
        elif os.path.isdir(file_path):
            shutil.rmtree(file_path)


def trigger_update_flag(update_flag_path):
    os.makedirs(os.path.dirname(update_flag_path), exist_ok=True)
    with open(update_flag_path, "w") as f:
        print("Update triggered!")
        json.dump({"trigger": True}, f)
    return "🔄 Update Triggered"


def load_map_image(folder, idx, prefix, cfg):
    path = os.path.join(folder, f"{prefix}_map_{idx}.png")
    if os.path.exists(path):
        return Image.open(path)
    elif os.path.exists(cfg["fallback_image_path"]):
        return Image.open(cfg["fallback_image_path"])
    else:
        # Dynamically create a fallback image with text
        img = Image.new("RGB", (256, 256), color="gray")
        draw = ImageDraw.Draw(img)
        draw.text((70, 120), "No Data", fill="white")
        return img

def load_retrieve_annotated_image(image_list, idx, fallback_image_path):
    print("Loading annotated image idx:", idx)
    if idx > len(image_list) - 1:
        return retrieve_fallback_image(fallback_image_path)

        # # Dynamically create a fallback image with text
        # img = Image.new("RGB", (256, 256), color="gray")
        # draw = ImageDraw.Draw(img)
        # draw.text((70, 120), "No Data", fill="white")
        # return img

    print("Path:", fallback_image_path)
    print(os.path.exists(fallback_image_path))
    path = image_list[idx]
    if os.path.exists(path):
        return Image.open(path)
    return retrieve_fallback_image(fallback_image_path)

    # elif os.path.exists(FALLBACK_IMAGE_PATH):
    #     return Image.open(FALLBACK_IMAGE_PATH)
    # else:
    #     # Dynamically create a fallback image with text
    #     img = Image.new("RGB", (256, 256), color="gray")
    #     draw = ImageDraw.Draw(img)
    #     draw.text((70, 120), "No Data", fill="white")
    #     return img

def get_latest_frame_idx(latest_idx_path):
    try:
        print("Reading latest idx from:", latest_idx_path)
        with open(latest_idx_path, "r") as f:
            return int(f.read().strip())
    except:
        return -1

# === Function to get latest image ===
def get_latest_som_image(idx, rgb_folder, fallback_image_path):
    image_path = os.path.join(rgb_folder, f"captured_rgb_{idx}.png")
    if os.path.exists(image_path):
        return Image.open(image_path)
    return retrieve_fallback_image(fallback_image_path)

    # img = Image.new("RGB", (256, 256), color="gray")
    # draw = ImageDraw.Draw(img)
    # draw.text((70, 120), "No Data", fill="white")
    # return img

# === Function to get label text ===
def get_label_text(idx, cfg):
    label_path = os.path.join(cfg["label_folder"], f"labels_{idx}.txt")
    if os.path.exists(label_path):
        with open(label_path, "r") as f:
            return f.read()
    return "No object labels found for this frame."

# === Function to get retrieve image ===
def get_retrieve_image(idx, retrieve_folder, postfix, fallback_image_path):
    image_path = os.path.join(retrieve_folder, f"retrieval_DB_{idx}_{postfix}.png")
    print("Retrieving from:", image_path)
    if os.path.exists(image_path):
        return Image.open(image_path)
    return retrieve_fallback_image(fallback_image_path)
    # img = Image.new("RGB", (256, 256), color="gray")
    # draw = ImageDraw.Draw(img)
    # draw.text((70, 120), "No Data", fill="white")
    # return img

# === Submit instruction from user ===
def submit_command(command, instruction_path):
    os.makedirs(os.path.dirname(instruction_path), exist_ok=True)
    with open(instruction_path, "w") as f:
        print(f"Submitting command: {command}")
        json.dump({"trigger": True, "command": command}, f)

    # if event:
    #     event.set()

    return f"✅ Instruction Sent" #: {command}"

# === Main updater function (image + labels) ===
# def update_interface():
#     idx = get_latest_frame_idx()
#     image = get_latest_som_image(idx)
#     label_text = get_label_text(idx)
#     return image, label_text,

def get_latest_image_from_listener(listener):
    print("Update triggered!")
    image = listener.get_latest_frame()
    return image

def update_interface(
    latest_idx_path, 
    retrieve_folder,
    postfix,
    fallback_image_path,
    cot_log_folder,
    retrieve_annotated_folder,
    rgb_folder
):
    idx = get_latest_frame_idx(latest_idx_path)
    image = get_latest_som_image(0, rgb_folder, fallback_image_path)
    # label_text = get_label_text(idx)

    retrieve_image = get_retrieve_image(idx, retrieve_folder, postfix, fallback_image_path)
    cot_messages = load_cot_chat(idx, cot_log_folder)  # this returns list of [role, content]

    # semantic_map = load_map_image(SEMANTIC_FOLDER, idx, "sem")
    # occupancy_map = load_map_image(OCCUPANCY_FOLDER, idx, "occ")
    # cost_map = load_map_image(COST_FOLDER, idx, "cost")
    image_list = glob.glob(f"{retrieve_annotated_folder}/{idx}/*.png")
    image_list.sort(key=os.path.getmtime)

    retrieve_1 = load_retrieve_annotated_image(image_list, 0, fallback_image_path)
    retrieve_2 = load_retrieve_annotated_image(image_list, 2, fallback_image_path)
    retrieve_3 = load_retrieve_annotated_image(image_list, 4, fallback_image_path)
    retrieve_4 = load_retrieve_annotated_image(image_list, 6, fallback_image_path)

    print("Latest idx:", idx)
    return image, retrieve_image, cot_messages, retrieve_1, retrieve_2, retrieve_3, retrieve_4 # , retrieve_2, retrieve_3

def load_cot_chat(idx, cot_log_folder):
    messages = []
    cot_path = os.path.join(cot_log_folder, f"cot_log_{idx}.txt")
    print("Loading CoT from:", cot_path)

    role = None
    buffer = []

    try:
        with open(cot_path, "r") as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue

                # Skip label-related lines
                if line.startswith("label-") or "3D bbox" in line or "extent" in line:
                    continue

                if line.startswith("User:"):
                    if role and buffer:
                        messages.append([role, "\n".join(buffer).strip()])
                        buffer = []
                    role = "User"
                    buffer.append(line[len("User:"):].strip())
                elif line.startswith("Assistant:") or line.startswith("System:"):
                    if role and buffer:
                        messages.append([role, "\n".join(buffer).strip()])
                        buffer = []
                    role = "Assistant"
                    buffer.append(line.split(":", 1)[1].strip())
                else:
                    buffer.append(line)

            # Add last block
            if buffer: # and role:
                messages.append([role, "\n".join(buffer).strip()])

    except Exception as e:
        messages.append(["System", "⌛ Waiting for CoT reasoning..."])

    if not messages:
        messages.append(["System", "⌛ Waiting for CoT reasoning..."])

    return messages


def load_cot_chat11(cfg):
    messages = []
    idx = get_latest_frame_idx(cfg)
    cot_path = os.path.join(cfg["cot_log_folder"], cfg["postfix"], f"cot_log_{idx}.txt")

    role = None
    buffer = []

    try:
        with open(cot_path, "r") as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                elif line.startswith("User:"):
                    if role and buffer:
                        messages.append([role, "\n".join(buffer).strip()])
                        buffer = []
                    role = "User"
                    buffer.append(line[len("User:"):].strip())
                elif line.startswith("Assistant:") or line.startswith("System:"):
                    if role and buffer:
                        messages.append([role, "\n".join(buffer).strip()])
                        buffer = []
                    role = "Assistant"
                    buffer.append(line.split(":", 1)[1].strip())
                else:
                    buffer.append(line)

            # Add last block
            if role and buffer:
                messages.append([role, "\n".join(buffer).strip()])

    except Exception as e:
        messages.append(["System", f"❌ Error reading CoT: {e}"])

    if not messages:
        messages.append(["System", "⌛ Waiting for CoT reasoning..."])

    return messages

# === Gradio Interface ===
def build_demo(cfg):
    event = Event()
    with gr.Blocks(title="STaR") as demo:
        gr.Markdown("## 🧠 STaR: Live Demo")
        with gr.Row():
            with gr.Column(scale=1):
                with gr.Row():
                    command_box = gr.Textbox(
                        label="Enter Command",
                        lines=1,
                        placeholder="e.g., Find me a yellow hand truck",
                        show_label=True,
                        scale=8
                    )
                    submit_btn = gr.Button("Submit", min_width=10)
                    update_btn = gr.Button("Update", min_width=10)
                with gr.Row():
                    som_img = gr.Image(label="Captured RGB Image", height=250, width=920)
                    # status_box = gr.Textbox(label="Status", lines=1, interactive=False, max_lines=1, scale=1)
                    status_box = gr.Textbox(
                        label="Status",
                        lines=2, max_lines=1, interactive=False,
                        show_label=False,          # save vertical space
                        container=False,           # remove the card padding
                        scale=0,                   # don’t grow with the row
                        min_width=100              # choose the width you want
                    )
                # label_textbox = gr.Textbox(label="Object Labels", lines=11, max_lines=11, interactive=False)
            with gr.Column(scale=1):
            # 🟩 Status ABOVE CoT
                cot_chat = gr.Chatbot(label="🧠 Live Chain-of-Thought Reasoning", height=300)
        with gr.Row():
            retrieve_img = gr.Image(label="Retrieve status", height=200)
        with gr.Row():
            retrieve_1 = gr.Image(label="Retrieve Result 1", height=180, width=520)
            retrieve_2 = gr.Image(label="Retrieve Result 2", height=180, width=520)
        with gr.Row():
            retrieve_3 = gr.Image(label="Retrieve Result 3", height=180, width=520)
            retrieve_4 = gr.Image(label="Retrieve Result 4", height=180, width=520)

        submit_btn.click(fn=partial(submit_command, instruction_path=cfg["instruction_path"]), inputs=[command_box], outputs=status_box)
        update_btn.click(fn=partial(trigger_update_flag, update_flag_path=cfg['update_flag_path']), inputs=[], outputs=status_box) #
        demo.load(
            fn=partial(
                update_interface,
                latest_idx_path=cfg['latest_idx_path'], 
                postfix=cfg['postfix'], 
                fallback_image_path=cfg['fallback_image_path'], 
                cot_log_folder=cfg['cot_log_folder'], 
                retrieve_folder=cfg['retrieve_folder'], 
                retrieve_annotated_folder=cfg['retrieve_annotated_folder'],
                rgb_folder=cfg['rgb_folder']
            ), inputs=[], 
            outputs=[som_img, retrieve_img, cot_chat, retrieve_1, retrieve_2, retrieve_3, retrieve_4], 
            every=0.5
        )

    return demo

@hydra.main(version_base=None, config_path="../configs", config_name="config")
def main(cfg):
    inference_cfg = cfg.inference

    demo = build_demo(inference_cfg)
    if will_init_folders := True:
        init_folders(inference_cfg)
        print("Initialized folders and cleared old data.")
    demo.queue()
    demo.launch(share=True)


if __name__ == "__main__":
    main()