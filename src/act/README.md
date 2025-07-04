# Customized:
## Custom:
### Train & Inference
- ``check_cam_serial.py``: Checking the camera serial
- ``custom_constants.py``: Define custom Task, DT, JOINT_NAMES, START_ARM_POSE
- ``custom_real_env.py``: Define custm_env(Recorder, ImageRecorder) and custom methods
- ``test_custom_robot_utils.py``: Define ImageRecorder, Recorder class. Also, define observation & action methods.
- `custom_imitate_episodes.py`: Train and Inference
### Data
- ``record_hdf5_act_form.py``: Get demonstration dataset(Using Padding and Episode Length is defined)
- ``read_view_hdf5.py``: Checking a hdf5 file
- ``read_hdf5_shape.py``: Checking a data shape

## Modified Scripts:
- ``detr_vae.py``: state(qpos) & action=14 -> state(qpos) & action = 7
- ``utils.py``: float64 -> float32
- ``constans.py``@aloha_scripts: Add task and put my data dir
- ``imitate_episodes.py``: state_dim = 14 -> 7

# Using Manual
## Collect Data
### Gripper: 
1. 
    PICO
    roslaunch onrobot_rg_control bringup.launch gripper:=rg2 ip:=192.168.1.1
2. 
    PICO
    rosrun onrobot_rg_control potentiometer_combined.py
### VR + Manipulator
1. 
    roslaunch vive_ros vive.launch
2. 
    rosrun robotory_rb10_rt servo_vr.py
### Record data
1. Activate conda:
    conda activate aloha
2. Run:
    rosrun custom_act record_hdf5_act_form.py

## Train
1. Move the dataset to <DATA_DIR>/<TASK_NAME>
2. Make 'checkpoint' directory at act directory.
3. Activate conda: 
    conda activate aloha
4. Run: 
    rosrun custom_act custom_imitate_episodes.py \
    --task_name rb_transfer_can \
    --ckpt_dir /home/vision/catkin_ws/src/custom_act/src/act/checkpoint/rb_transfer_can_scripted_5000 \
    --policy_class ACT --kl_weight 10 --chunk_size 100 --hidden_dim 512 --batch_size 8 --dim_feedforward 3200 \
    --num_epochs 5000  --lr 1e-5 \
    --seed 0
    ```

## Inference
### Gripper
    PICO
    roslaunch onrobot_rg_control bringup.launch gripper:=rg2 ip:=192.168.1.1

# Run
    conda activate aloha
    rosrun custom_act custom_imitate_episodes.py \
    --eval \
    --temporal_agg \
    --task_name rb_transfer_can \
    --ckpt_dir /home/vision/catkin_ws/src/custom_act/src/act/checkpoint/rb_transfer_can_scripted \
    --policy_class ACT --kl_weight 10 --chunk_size 100 --hidden_dim 512 --batch_size 8 --dim_feedforward 3200 \
    --num_epochs 2000  --lr 1e-5 \
    --seed 0

---
# ACT: Action Chunking with Transformers

### *New*: [ACT tuning tips](https://docs.google.com/document/d/1FVIZfoALXg_ZkYKaYVh-qOlaXveq5CtvJHXkY25eYhs/edit?usp=sharing)
TL;DR: if your ACT policy is jerky or pauses in the middle of an episode, just train for longer! Success rate and smoothness can improve way after loss plateaus.

#### Project Website: https://tonyzhaozh.github.io/aloha/

This repo contains the implementation of ACT, together with 2 simulated environments:
Transfer Cube and Bimanual Insertion. You can train and evaluate ACT in sim or real.
For real, you would also need to install [ALOHA](https://github.com/tonyzhaozh/aloha).

### Updates:
You can find all scripted/human demo for simulated environments [here](https://drive.google.com/drive/folders/1gPR03v05S1xiInoVJn7G7VJ9pDCnxq9O?usp=share_link).


### Repo Structure
- ``imitate_episodes.py`` Train and Evaluate ACT
- ``policy.py`` An adaptor for ACT policy
- ``detr`` Model definitions of ACT, modified from DETR
- ``sim_env.py`` Mujoco + DM_Control environments with joint space control
- ``ee_sim_env.py`` Mujoco + DM_Control environments with EE space control
- ``scripted_policy.py`` Scripted policies for sim environments
- ``constants.py`` Constants shared across files
- ``utils.py`` Utils such as data loading and helper functions
- ``visualize_episodes.py`` Save videos from a .hdf5 dataset


### Installation

    conda create -n aloha python=3.8.10
    conda activate aloha
    pip install torchvision
    pip install torch
    pip install pyquaternion
    pip install pyyaml
    pip install rospkg
    pip install pexpect
    pip install mujoco==2.3.7
    pip install dm_control==1.0.14
    pip install opencv-python
    pip install matplotlib
    pip install einops
    pip install packaging
    pip install h5py
    pip install ipython
    cd act/detr && pip install -e .

### Example Usages

To set up a new terminal, run:

    conda activate aloha
    cd <path to act repo>

### Simulated experiments

We use ``sim_transfer_cube_scripted`` task in the examples below. Another option is ``sim_insertion_scripted``.
To generated 50 episodes of scripted data, run:

    python3 record_sim_episodes.py \
    --task_name sim_transfer_cube_scripted \
    --dataset_dir <data save dir> \
    --num_episodes 50

To can add the flag ``--onscreen_render`` to see real-time rendering.
To visualize the episode after it is collected, run

    python3 visualize_episodes.py --dataset_dir <data save dir> --episode_idx 0

To train ACT:
    
    # Transfer Cube task
    python3 imitate_episodes.py \
    --task_name sim_transfer_cube_scripted \
    --ckpt_dir <ckpt dir> \
    --policy_class ACT --kl_weight 10 --chunk_size 100 --hidden_dim 512 --batch_size 8 --dim_feedforward 3200 \
    --num_epochs 2000  --lr 1e-5 \
    --seed 0


To evaluate the policy, run the same command but add ``--eval``. This loads the best validation checkpoint.
The success rate should be around 90% for transfer cube, and around 50% for insertion.
To enable temporal ensembling, add flag ``--temporal_agg``.
Videos will be saved to ``<ckpt_dir>`` for each rollout.
You can also add ``--onscreen_render`` to see real-time rendering during evaluation.

For real-world data where things can be harder to model, train for at least 5000 epochs or 3-4 times the length after the loss has plateaued.
Please refer to [tuning tips](https://docs.google.com/document/d/1FVIZfoALXg_ZkYKaYVh-qOlaXveq5CtvJHXkY25eYhs/edit?usp=sharing) for more info.

