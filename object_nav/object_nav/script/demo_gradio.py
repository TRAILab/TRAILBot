import gradio as gr
import os
import json
from PIL import Image, ImageDraw


# === CONFIG ===
BASE_PATH = "/home/trailbot/Documents/data_process_Kitti/results/04"
INSTRUCTION_PATH = os.path.join(BASE_PATH, "instructions", "instruction.json")
LATEST_IDX_PATH = os.path.join(BASE_PATH, "latest_frame.txt")
RGB_FOLDER = os.path.join(BASE_PATH, "annotated_rgb")
LABEL_FOLDER = os.path.join(BASE_PATH, "labels")  # Make sure to save labels as labels_{idx}.txt
COT_LOG_FOLDER = os.path.join(BASE_PATH, "cot_log")  # now a folder

SEMANTIC_FOLDER = os.path.join(BASE_PATH, "seg_map")
OCCUPANCY_FOLDER = os.path.join(BASE_PATH, "occ_map")
COST_FOLDER = os.path.join(BASE_PATH, "cost_map")
FALLBACK_IMAGE_PATH = os.path.join(BASE_PATH, "fallback_image.png")  # Fallback image if no data exists
UPDATE_FLAG_PATH = os.path.join(BASE_PATH, "update", "update.txt")


def trigger_update_flag():
    os.makedirs(os.path.dirname(UPDATE_FLAG_PATH), exist_ok=True)
    with open(UPDATE_FLAG_PATH, "w") as f:
        json.dump({"trigger": True}, f)
    return "🔄 Update Triggered"


def load_map_image(folder, idx, prefix):
    path = os.path.join(folder, f"{prefix}_map_{idx}.png")
    if os.path.exists(path):
        return Image.open(path)
    elif os.path.exists(FALLBACK_IMAGE_PATH):
        return Image.open(FALLBACK_IMAGE_PATH)
    else:
        # Dynamically create a fallback image with text
        img = Image.new("RGB", (256, 256), color="gray")
        draw = ImageDraw.Draw(img)
        draw.text((70, 120), "No Data", fill="white")
        return img

def get_latest_frame_idx():
    try:
        with open(LATEST_IDX_PATH, "r") as f:
            return int(f.read().strip())
    except:
        return -1

# === Function to get latest image ===
def get_latest_som_image(idx):
    image_path = os.path.join(RGB_FOLDER, f"annotated_rgb_{idx}.png")
    if os.path.exists(image_path):
        return Image.open(image_path)
    return None

# === Function to get label text ===
def get_label_text(idx):
    label_path = os.path.join(LABEL_FOLDER, f"labels_{idx}.txt")
    if os.path.exists(label_path):
        with open(label_path, "r") as f:
            return f.read()
    return "No object labels found for this frame."

# === Submit instruction from user ===
def submit_command(command):
    os.makedirs(os.path.dirname(INSTRUCTION_PATH), exist_ok=True)
    with open(INSTRUCTION_PATH, "w") as f:
        json.dump({"trigger": True, "command": command}, f)
    return f"✅ Instruction Sent"#: {command}"

# === Main updater function (image + labels) ===
# def update_interface():
#     idx = get_latest_frame_idx()
#     image = get_latest_som_image(idx)
#     label_text = get_label_text(idx)
#     return image, label_text, 

def update_interface():
    idx = get_latest_frame_idx()
    image = get_latest_som_image(idx)
    label_text = get_label_text(idx)
    cot_messages = load_cot_chat(idx)  # this returns list of [role, content]

    semantic_map = load_map_image(SEMANTIC_FOLDER, idx, "sem")
    occupancy_map = load_map_image(OCCUPANCY_FOLDER, idx, "occ")
    cost_map = load_map_image(COST_FOLDER, idx, "cost")
    return image, label_text, cot_messages, semantic_map, occupancy_map, cost_map

def load_cot_chat(idx):
    messages = []
    cot_path = os.path.join(COT_LOG_FOLDER, f"cot_log_{idx}.txt")

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
            if role and buffer:
                messages.append([role, "\n".join(buffer).strip()])

    except Exception as e:
        messages.append(["System", "⌛ Waiting for CoT reasoning..."])

    if not messages:
        messages.append(["System", "⌛ Waiting for CoT reasoning..."])
    
    return messages


def load_cot_chat11():
    messages = []
    idx = get_latest_frame_idx()
    cot_path = os.path.join(COT_LOG_FOLDER, f"cot_log_{idx}.txt")

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

with gr.Blocks(title="OpenNav") as demo:
    gr.Markdown("## 🧠 OpenNav: Instruction Following Live Demo")
    with gr.Row():
        with gr.Column(scale=1):
            som_img = gr.Image(label="Annotated RGB Image", height=400, width=720)
            label_textbox = gr.Textbox(label="Object Labels", lines=11, max_lines=11, interactive=False)
            
        with gr.Column(scale=1):
            with gr.Row():
                command_box = gr.Textbox(
                    label="Enter Command", 
                    lines=1,
                    placeholder="e.g., go to the ladder",
                    show_label=True,
                    scale=8
                )
                # with gr.Column(scale=1):
                submit_btn = gr.Button("Submit", min_width=40)
                update_btn = gr.Button("Update", min_width=40)
                # submit_btn = gr.Button("Submit", scale=1, min_width=80)
                status_box = gr.Textbox(label="Status", lines=1, interactive=False, max_lines=1, scale=3)
        # 🟩 Status ABOVE CoT
            cot_chat = gr.Chatbot(label="🧠 Live Chain-of-Thought Reasoning", height=590)

    with gr.Row():
        sem_map = gr.Image(label="Semantic Map", height=260, width=520)
        occ_map = gr.Image(label="Occupancy Map", height=260, width=520)
        cost_map = gr.Image(label="Cost Map", height=260, width=520)

    submit_btn.click(fn=submit_command, inputs=command_box, outputs=status_box)
    update_btn.click(fn=trigger_update_flag, inputs=[], outputs=status_box)
    

    demo.load(fn=update_interface, inputs=[], outputs=[som_img, label_textbox, cot_chat, sem_map, occ_map, cost_map], every=0.5)


if __name__ == "__main__":
    demo.launch(share=True)



