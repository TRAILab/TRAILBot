import gradio as gr
import os
from PIL import Image

def load_visualization(frame_index: int, base_path: str = "/home/trailbot/Documents/data_process_Kitti/results/04"):

    rgb_path = os.path.join(base_path, "annotated_rgb", f"annotated_rgb_{frame_index}.png")
    cost_path = os.path.join(base_path, "cost_map", f"cost_map_{frame_index}.png")

    message_folder = os.path.join(base_path, "messages")  # match configs.message_path
    reasoning_file = None
    if os.path.exists(message_folder):
        for f in os.listdir(message_folder):
            if f.endswith(f"{frame_index}.txt"):
                reasoning_file = os.path.join(message_folder, f)
                break

    # Load images
    seg_img = Image.open(rgb_path) if os.path.exists(rgb_path) else None
    cost_img = Image.open(cost_path) if os.path.exists(cost_path) else None

    # Load text
    reasoning = "No reasoning file found."
    if reasoning_file and os.path.exists(reasoning_file):
        with open(reasoning_file, "r") as f:
            reasoning = f.read()

    return seg_img, cost_img, reasoning


demo = gr.Interface(
    fn=load_visualization,
    inputs=[
        gr.Number(label="Frame Index (e.g., 20)"),
        gr.Textbox(value="/home/trailbot/Documents/data_process_Kitti/results/04", label="Base Path")
    ],
    outputs=[
        gr.Image(label="Annotated RGB Image"),
        gr.Image(label="Cost Map"),
        gr.Textbox(label="Chain of Thought Reasoning", lines=20)
    ],
    title="VLN Visualization Viewer",
    description="Displays segmentation, cost map, and reasoning for a selected frame."
)

if __name__ == "__main__":
    demo.launch(share=False)
