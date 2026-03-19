import gradio as gr
import os, json
from PIL import Image, ImageDraw

# === CONFIG ===
BASE_PATH = "/home/trailbot/Documents/data_process_Kitti/results/04"
INSTRUCTION_PATH = os.path.join(BASE_PATH, "instructions", "instruction.json")
LATEST_IDX_PATH  = os.path.join(BASE_PATH, "latest_frame.txt")
RGB_FOLDER       = os.path.join(BASE_PATH, "annotated_rgb")
LABEL_FOLDER     = os.path.join(BASE_PATH, "labels")
COT_LOG_FOLDER   = os.path.join(BASE_PATH, "cot_log")
SEMANTIC_FOLDER  = os.path.join(BASE_PATH, "seg_map")
OCCUPANCY_FOLDER = os.path.join(BASE_PATH, "occ_map")
COST_FOLDER      = os.path.join(BASE_PATH, "cost_map")
FALLBACK_IMAGE_PATH = os.path.join(BASE_PATH, "fallback_image.png")
UPDATE_FLAG_PATH = os.path.join(BASE_PATH, "update", "update.txt")

# ---------- utilities ----------
def trigger_update_flag():
    os.makedirs(os.path.dirname(UPDATE_FLAG_PATH), exist_ok=True)
    with open(UPDATE_FLAG_PATH, "w") as f:
        json.dump({"trigger": True}, f)
    return "🔄 Update triggered"

def load_map_image(folder, idx, prefix):
    path = os.path.join(folder, f"{prefix}_map_{idx}.png")
    if os.path.exists(path):
        return Image.open(path)
    if os.path.exists(FALLBACK_IMAGE_PATH):
        return Image.open(FALLBACK_IMAGE_PATH)
    img = Image.new("RGB", (256, 256), "gray")
    ImageDraw.Draw(img).text((70, 120), "No Data", fill="white")
    return img

def get_latest_frame_idx():
    try:
        with open(LATEST_IDX_PATH) as f:
            return int(f.read().strip())
    except Exception:
        return -1

def get_latest_som_image(idx):
    img_path = os.path.join(RGB_FOLDER, f"annotated_rgb_{idx}.png")
    return Image.open(img_path) if os.path.exists(img_path) else None

def get_label_text(idx):
    label_path = os.path.join(LABEL_FOLDER, f"labels_{idx}.txt")
    if os.path.exists(label_path):
        with open(label_path) as f:
            return f.read()
    return "No object labels found for this frame."

# NEW message-format loader
def load_cot_chat(idx: int) -> list[dict]:
    messages, role, buffer = [], None, []
    cot_path = os.path.join(COT_LOG_FOLDER, f"cot_log_{idx}.txt")

    try:
        with open(cot_path) as f:
            for raw in f:
                line = raw.strip()
                if not line or line.startswith("label-") or "3D bbox" in line or "extent" in line:
                    continue
                if line.startswith("User:"):
                    if role and buffer:
                        messages.append({"role": role, "content": "\n".join(buffer)})
                    role, buffer = "user", [line[5:].strip()]
                elif line.startswith(("Assistant:", "System:")):
                    if role and buffer:
                        messages.append({"role": role, "content": "\n".join(buffer)})
                    role = "assistant"
                    buffer = [line.split(":", 1)[1].strip()]
                else:
                    buffer.append(line)
        if role and buffer:
            messages.append({"role": role, "content": "\n".join(buffer)})
    except FileNotFoundError:
        pass

    return messages or [{"role": "system", "content": "⌛ Waiting for CoT reasoning…"}]

def submit_command(command):
    os.makedirs(os.path.dirname(INSTRUCTION_PATH), exist_ok=True)
    with open(INSTRUCTION_PATH, "w") as f:
        json.dump({"trigger": True, "command": command}, f)
    return "✅ Instruction sent"

def update_interface():
    idx          = get_latest_frame_idx()
    image        = get_latest_som_image(idx)
    label_text   = get_label_text(idx)
    cot_messages = load_cot_chat(idx)
    semantic_map  = load_map_image(SEMANTIC_FOLDER, idx, "sem")
    occupancy_map = load_map_image(OCCUPANCY_FOLDER, idx, "occ")
    cost_map      = load_map_image(COST_FOLDER, idx, "cost")
    return image, label_text, cot_messages, semantic_map, occupancy_map, cost_map

# ---------- UI ----------
with gr.Blocks(title="OpenNav") as demo:
    gr.Markdown("## 🧠 OpenNav: Instruction-Following Live Demo")
    with gr.Row():
        with gr.Column(scale=1):
            som_img       = gr.Image(label="Annotated RGB Image", height=400, width=720)
            label_textbox = gr.Textbox(label="Object Labels", lines=11, interactive=False)
        with gr.Column(scale=1):
            with gr.Row():
                command_box = gr.Textbox(
                    label="Enter Command",
                    placeholder="e.g., go to the ladder",
                    scale=8
                )
                submit_btn = gr.Button("Submit", min_width=40)
                update_btn = gr.Button("Update", min_width=40)
                status_box = gr.Textbox(label="Status", interactive=False, scale=3)
            cot_chat = gr.Chatbot(
                label="🧠 Live Chain-of-Thought Reasoning",
                height=590,
                type="messages"               # NEW!
            )
    with gr.Row():
        sem_map = gr.Image(label="Semantic Map",  height=260, width=520)
        occ_map = gr.Image(label="Occupancy Map", height=260, width=520)
        cost_map= gr.Image(label="Cost Map",      height=260, width=520)

    submit_btn.click(fn=submit_command, inputs=command_box, outputs=status_box)
    update_btn.click(fn=trigger_update_flag, inputs=None, outputs=status_box)
    demo.load(fn=update_interface, inputs=None,
              outputs=[som_img, label_textbox, cot_chat, sem_map, occ_map, cost_map])

if __name__ == "__main__":
    demo.launch(share=True)
