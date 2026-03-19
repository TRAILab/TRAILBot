# Project Setup and Usage Guide

This README consists of two parts:

- **Part 1: Installation**
- **Part 2: How to Use**

---

## Part 1: Installation

You can set up this project in one of two ways:

---

### ✅ Option 1: Install via Docker Image (Highly recommanded)

```bash
# Step 1: Download the pre-built Docker image then 
# For example for our case: 
cd /home/trailbot/trail_ws/src/TRAILBot/object_nav/object_nav/docker_elems
docker load -i graph_rag_docker_image.tar

# Step 2: Use the provided bash script to run the container
bash docker_container.sh # this create a container called ours_docker
```


### ✅ Option 2: Manual Setup (Conda + Pip) 

**⚠️ You MUST follow these steps in order due to dependency conflicts.**

Step 1: Create and activate a new Conda environment.

```bash
conda create -n <your_conda_name> python=3.10.13 -y
conda activate <your_conda_name>
````

Step 2: Install PyTorch, torchvision, and torchaudio with CUDA 11.8.

```bash
pip install torch==2.3.0 torchvision==0.18.0 torchaudio==2.3.0 --index-url https://download.pytorch.org/whl/cu118
```

Step 3: Clone the GroundingDINO repository and install it.

```bash
git clone <grounding_dino_repo_url>
cd GroundingDINO
pip install -e .
```

Step 4: Update the environment using the provided YAML file.

```bash
conda env update -n <your_conda_name> -f my_env.yml
```

Step 5: Clone and install third-party packages.
Repeat the following commands for each package: `tokenize-anything`, `s2wrapper`, `vila`, `recognize-anything`.

```bash
git clone <repo_url>
cd <folder_name>
pip install -e .
cd ..
```

Step 6: Fix DeepSpeed and Pydantic conflicts.

First, uninstall existing Pydantic and install the correct DeepSpeed version.

```bash
pip uninstall pydantic
pip install deepspeed==0.9.5
```

Remove the conflicting DeepSpeed installation.

```bash
rm -rf /root/anaconda3/envs/ragnav/lib/python3.10/site-packages/deepspeed/
```

Copy your modified DeepSpeed into the site-packages.

```bash
cp -r ../deepspeed/ /root/anaconda3/envs/ragnav/lib/python3.10/site-packages/
```

Reinstall a compatible version of Pydantic.

```bash
pip install pydantic==2.11.4
```

Step 7: Source your ROS 2 environment and workspace.

```bash
source /opt/ros/<ros_distro>/setup.bash
source /path/to/your/ros2_ws/install/setup.bash
```



## Part 2: How to Use

You can run the project depending on your chosen installation method:

---

### ✅ If you used Option 1 (Docker Image):

First, enter the container:

```bash
docker exec -it <container_name> bash
````

Then activate the Conda environment and run the script:

```bash
conda activate ragnav
cd /path/to/our/repo    # for example in our case "/home/trailbot/trail_ws/src/TRAILBot/object_nav/object_nav"
python script/scene_graph_with_nvila.py
```

---

### ✅ If you used Option 2 (Manual Setup):

Simply navigate to the project directory and run the script:

```bash
cd /path/to/our/repo
python script/scene_graph_with_nvila.py
```
