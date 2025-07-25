### Task parameters

# Author: Chemin Ahn (chemx3937@gmail.com)
# Use of this source code is governed by the MIT, see LICENSE

# Data 취득 및 Train시 episode_len을 짧게 설정
# Imference 시 episode_len을 길게 설정


# 개인 PC

# DATA_DIR= '/home/chem/vision_custom_act/src/act/data'

# vision PC
DATA_DIR = '/home/vision/catkin_ws/src/custom_act/src/act/data'
TASK_CONFIGS = {
    # 'aloha_wear_shoe':{
    #     'dataset_dir': DATA_DIR + '/aloha_wear_shoe',
    #     'num_episodes': 50,
    #     'episode_len': 1000,
    #     'camera_names': ['cam_high', 'cam_low', 'cam_left_wrist', 'cam_right_wrist']
    # },
    'rb_transfer_can':{
        'dataset_dir': DATA_DIR + '/rb_transfer_can',
        'num_episodes': 100,
        'episode_len': 600,
        'camera_names': ['cam_high', 'cam_low']
    },
    'rb_push_toolbox':{
        'dataset_dir': DATA_DIR + '/rb_push_toolbox',
        'num_episodes': 50,
        # Data 취득 및 Train 시에는 episode len 길게
        # 'episode_len': 350,

        # Inference 시에는 episode len 길게
        'episode_len': 5000,
        'camera_names': ['cam_high', 'cam_low']
    },
}

### ALOHA fixed constants
# DT = 0.02 # 50Hz(ACT 원본)
DT = 0.05 # 20Hz (수정)

# 원본
# JOINT_NAMES = ["waist", "shoulder", "elbow", "forearm_roll", "wrist_angle", "wrist_rotate"]
# START_ARM_POSE = [0, -0.96, 1.16, 0, -0.3, 0, 0.02239, -0.02239,  0, -0.96, 1.16, 0, -0.3, 0, 0.02239, -0.02239]

# Custom
JOINT_NAMES = ["base", "shoulder", "elbow", "wrist1", "wrist2", "wrist3"]
# START_ARM_POSE = [1.570796, 0, -1.570796, 0, -1.570796, 0, 1] # 초기 위치 정해서 rad으로 입력 필요
# [65 -13 -100 30 270 40 1]

# rb_transfer_can_start_pose
# [60 -26 -100 37 275 53 1]
# START_ARM_POSE = [1.0472, -0.4537856, -1.72876, 0.645772, 4.799655, 0.925, 0] # 초기 위치 정해서 rad으로 입력 필요

# push tool box_start_pose
import math
base = 86.19*math.pi/180  # 81.6 degree to radian
shoulder = -6.6*math.pi/180  # -3.48 degree to radian
elbow = -117.63*math.pi/180  # -123.4 degree to radian
wrist1 = 34.23*math.pi/180  # 51.43 degree to radian
wrist2 = 270.00*math.pi/180  # 269.10 degree to radian
wrist3 = -88.96*math.pi/180  # -87.22 degree to radian
START_ARM_POSE = [base, shoulder, elbow, wrist1, wrist2, wrist3, 0] # 초기 위치 정해서 rad으로 입력 필요

# START_ARM_POSE = [base", "shoulder", "elbow", "wrist1", "wrist2", "wrist3, "gripper"]

# Left finger position limits (qpos[7]), right_finger = -1 * left_finger
# MASTER_GRIPPER_POSITION_OPEN = 0.02417
# MASTER_GRIPPER_POSITION_CLOSE = 0.01244
# PUPPET_GRIPPER_POSITION_OPEN = 0.05800
# PUPPET_GRIPPER_POSITION_CLOSE = 0.01844

# # Gripper joint limits (qpos[6])
# MASTER_GRIPPER_JOINT_OPEN = 0.3083
# MASTER_GRIPPER_JOINT_CLOSE = -0.6842
# PUPPET_GRIPPER_JOINT_OPEN = 1.4910
# PUPPET_GRIPPER_JOINT_CLOSE = -0.6213

# ############################ Helper functions ############################

# MASTER_GRIPPER_POSITION_NORMALIZE_FN = lambda x: (x - MASTER_GRIPPER_POSITION_CLOSE) / (MASTER_GRIPPER_POSITION_OPEN - MASTER_GRIPPER_POSITION_CLOSE)
# PUPPET_GRIPPER_POSITION_NORMALIZE_FN = lambda x: (x - PUPPET_GRIPPER_POSITION_CLOSE) / (PUPPET_GRIPPER_POSITION_OPEN - PUPPET_GRIPPER_POSITION_CLOSE)
# MASTER_GRIPPER_POSITION_UNNORMALIZE_FN = lambda x: x * (MASTER_GRIPPER_POSITION_OPEN - MASTER_GRIPPER_POSITION_CLOSE) + MASTER_GRIPPER_POSITION_CLOSE
# PUPPET_GRIPPER_POSITION_UNNORMALIZE_FN = lambda x: x * (PUPPET_GRIPPER_POSITION_OPEN - PUPPET_GRIPPER_POSITION_CLOSE) + PUPPET_GRIPPER_POSITION_CLOSE
# MASTER2PUPPET_POSITION_FN = lambda x: PUPPET_GRIPPER_POSITION_UNNORMALIZE_FN(MASTER_GRIPPER_POSITION_NORMALIZE_FN(x))

# MASTER_GRIPPER_JOINT_NORMALIZE_FN = lambda x: (x - MASTER_GRIPPER_JOINT_CLOSE) / (MASTER_GRIPPER_JOINT_OPEN - MASTER_GRIPPER_JOINT_CLOSE)
# PUPPET_GRIPPER_JOINT_NORMALIZE_FN = lambda x: (x - PUPPET_GRIPPER_JOINT_CLOSE) / (PUPPET_GRIPPER_JOINT_OPEN - PUPPET_GRIPPER_JOINT_CLOSE)
# MASTER_GRIPPER_JOINT_UNNORMALIZE_FN = lambda x: x * (MASTER_GRIPPER_JOINT_OPEN - MASTER_GRIPPER_JOINT_CLOSE) + MASTER_GRIPPER_JOINT_CLOSE
# PUPPET_GRIPPER_JOINT_UNNORMALIZE_FN = lambda x: x * (PUPPET_GRIPPER_JOINT_OPEN - PUPPET_GRIPPER_JOINT_CLOSE) + PUPPET_GRIPPER_JOINT_CLOSE
# MASTER2PUPPET_JOINT_FN = lambda x: PUPPET_GRIPPER_JOINT_UNNORMALIZE_FN(MASTER_GRIPPER_JOINT_NORMALIZE_FN(x))

# MASTER_GRIPPER_VELOCITY_NORMALIZE_FN = lambda x: x / (MASTER_GRIPPER_POSITION_OPEN - MASTER_GRIPPER_POSITION_CLOSE)
# PUPPET_GRIPPER_VELOCITY_NORMALIZE_FN = lambda x: x / (PUPPET_GRIPPER_POSITION_OPEN - PUPPET_GRIPPER_POSITION_CLOSE)

# MASTER_POS2JOINT = lambda x: MASTER_GRIPPER_POSITION_NORMALIZE_FN(x) * (MASTER_GRIPPER_JOINT_OPEN - MASTER_GRIPPER_JOINT_CLOSE) + MASTER_GRIPPER_JOINT_CLOSE
# MASTER_JOINT2POS = lambda x: MASTER_GRIPPER_POSITION_UNNORMALIZE_FN((x - MASTER_GRIPPER_JOINT_CLOSE) / (MASTER_GRIPPER_JOINT_OPEN - MASTER_GRIPPER_JOINT_CLOSE))
# PUPPET_POS2JOINT = lambda x: PUPPET_GRIPPER_POSITION_NORMALIZE_FN(x) * (PUPPET_GRIPPER_JOINT_OPEN - PUPPET_GRIPPER_JOINT_CLOSE) + PUPPET_GRIPPER_JOINT_CLOSE
# PUPPET_JOINT2POS = lambda x: PUPPET_GRIPPER_POSITION_UNNORMALIZE_FN((x - PUPPET_GRIPPER_JOINT_CLOSE) / (PUPPET_GRIPPER_JOINT_OPEN - PUPPET_GRIPPER_JOINT_CLOSE))

# MASTER_GRIPPER_JOINT_MID = (MASTER_GRIPPER_JOINT_OPEN + MASTER_GRIPPER_JOINT_CLOSE)/2
