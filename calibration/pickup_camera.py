import os

folder = "top300_images"

files = sorted(os.listdir(folder))

for i, f in enumerate(files, start=1):
    old_path = os.path.join(folder, f)
    new_name = f"set_{i}.jpg"
    new_path = os.path.join(folder, new_name)

    os.rename(old_path, new_path)