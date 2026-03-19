import json
import os


path_file = "/home/trailbot/Documents/data_process_Kitti/results/04/instructions/instruction.json"

# check if the file exists
if not os.path.exists(path_file):
    print(f"❌ File not found: {path_file}")
    exit(1)

# Read JSON
with open(path_file, 'r') as f:
    try:
        data = json.load(f)
    except json.JSONDecodeError as e:
        print(f"❌ Failed to parse JSON: {e}")
        exit(1)

# 修改 trigger
data['trigger'] = True # False True
data['command'] = "Go to the board that says 'Caution' without a small traffic cone at its base."
# help find something that can be used to block the path
# data['command'] =  "Go to the board that says 'Caution' — there is a small traffic cone at its base."
# data['command'] = "go to the person"
#"Go to the real car parked behind the boards."

# Outdoor tasks
#"help me find something that can be used for BBQ"
#"go to the yellow bollard with some wooden pallets near it, do not step on the sign on the ground."
#"Go tho the trash can that has cylindrical." 

with open(path_file, 'w') as f:
    json.dump(data, f, indent=4)

print(f"✅ Trigger has been set to {data['trigger']}.")
