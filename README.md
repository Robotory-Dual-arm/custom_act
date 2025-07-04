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
    PICO
    roslaunch onrobot_rg_control bringup.launch gripper:=rg2 ip:=192.168.1.1

    PICO
    rosrun onrobot_rg_control potentiometer_combined.py
### VR + Manipulator 
    roslaunch vive_ros vive.launch

    rosrun robotory_rb10_rt servo_vr.py
### Record data

    conda activate aloha
    rosrun custom_act record_hdf5_act_form.py

## Train
1. Move the dataset to <DATA_DIR>/<TASK_NAME>
2. Make 'checkpoint' directory at act directory.

    conda activate aloha
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