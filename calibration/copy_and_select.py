import os
import shutil

txt_path = "/workspace/trail_ws/calibration_2025/slow/match.txt"
input_root = "/workspace/trail_ws/calibration_2025/slow/img"
output_folder = "/workspace/trail_ws/calibration_2025/slow/output"
os.makedirs(output_folder, exist_ok=True)

pc_list = []
img_index = 1

with open(txt_path, "r") as f:
    lines = f.readlines()

for line in lines:
    if ":" not in line:
        continue
    
    parts = line.strip().split(":")
    pc_label = parts[0].strip()
    img_id = parts[1].strip()
    # import pdb; pdb.set_trace()
    if not img_id.isdigit():
        continue

    try:
        pc_id = int(pc_label.split()[-1])
    except:
        continue

    img_filename = f"output_{img_id}.jpg"
    src_img_path = os.path.join(input_root, img_filename)
    if os.path.isfile(src_img_path):
        dst_img_path = os.path.join(output_folder, f"set_{img_index}.png")
        shutil.copy(src_img_path, dst_img_path)
        pc_list.append(pc_id)
        img_index += 1

print("Copied images from matched point clouds:", pc_list)
