# custom_act

## 🛠️ Install

### 📦 Package 설치

```bash
cd <ros1 workspace 경로>/src
git clone https://github.com/Robotory-Dual-arm/custom_act.git
```

### 🐍 Conda 환경 생성

```bash
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

cd <ros1 workspace 경로>/src/custom_act/src/act/detr
pip install -e .
```

---

## 🔧 Customized

- `custom_imitate_episodes.py` : Train 및 Inference
- `custom_constants.py` : DATA_DIR, Task, DT, START_ARM_POSE 정의
- `custom_real_env.py` : Inference를 위한 설정 및 요소 정의
- `custom_robot_utils.py` : Inference 시 Camera, Manipulator, Gripper 제어
- `record_hdf5_act_form.py` : DEMO_DATA 취득

### 🛠 Modified

- `detr_vae.py`: state(qpos) & action=14 → state(qpos) & action=7
- `utils.py`: float64 → float32

---

## ➕ Extension

### ✅ TASK 추가 시

- `custom_constants.py`: `TASK_CONFIGS` 정의
- `custom_imitate_episodes.py`: task 추가

### ✅ Camera 추가 시

- `custom_constants.py`: TASK_CONFIGS에 camera 정의
- `custom_real_env.py`: `RealEnv.__init__()`에서 camera serial 등록
- `custom_robot_utils.py`: `ImageRecorder`에 파라미터 추가
- `custom_imitate_episodes.py`: camera 및 `cv2.imshow()` 정의

### ✅ Robot State 확장 시

- `custom_constants.py`: `JOINT_NAMES`, `START_ARM_POSE` 정의
- `custom_robot_utils.py`: `Recorder`에서 확장된 state 정의
- `custom_imitate_episodes.py`: `state_dim` 수정
- `detr_vae.py`: state(qpos) & action=14 -> state(qpos) & action = 7
- `utils.py`: dtype 변경(float64 -> float32)

### ✅ qvel, qeffort 추가 시

- `custom_real_env.py`: `get_observation`에 정의
- `custom_imitate_episodes.py`: `forward_pass`에 반영

---

## 🎮 Collect DEMO_DATA

### ✅ 실행 전 확인 요소

#### `custom_constants.py`

1. `DATA_DIR` 설정
2. Task 정의:
   - `episode_len`이 충분한지
   - 사용 camera가 모두 정의되어 있는지

#### `custom_robot_utils.py`

1. 사용하는 카메라 수만큼 `ImageRecorder` 정의
2. `Recorder` 클래스에서 IP 설정
3. 로봇/그리퍼에 맞게 제어 명령 설정

### ▶ 실행 명령어

#### 그리퍼 연결

```bash
roslaunch onrobot_rg_control bringup.launch gripper:=rg2 ip:=192.168.1.1
```

#### 그리퍼 제어

```bash
rosrun onrobot_rg_control potentiometer_combined.py
```

#### VR 트래커 실행

```bash
roslaunch vive_ros vive.launch
```

#### Manipulator 제어

```bash
rosrun robotory_rb10_rt servo_vr.py
```

#### Record Data

```bash
conda activate aloha
rosrun custom_act record_hdf5_act_form.py
```

---

## 🧠 Train

### ✅ 실행 전 확인 사항

- `custom_constants.py`:
  1. `<DATA_DIR>/<TASK_NAME>`에 DEMO_DATA 존재 확인
  2. `num_episodes`와 데이터 수 일치 확인

### ▶ 실행 명령어

```bash
conda activate aloha 
rosrun custom_act custom_imitate_episodes.py \
--task_name rb_transfer_can \
--ckpt_dir /home/vision/catkin_ws/src/custom_act/src/act/checkpoint/rb_transfer_can_scripted_5000 \
--policy_class ACT --kl_weight 10 --chunk_size 100 --hidden_dim 512 \
--batch_size 8 --dim_feedforward 3200 --num_epochs 5000 --lr 1e-5 --seed 0
```

---

## 🚀 Inference

### ✅ 실행 전 확인 사항

- `episode_len`이 커지면 rollout 길이도 늘어남
- `Collect DEMO_DATA` 시 episode_len과 달라도 됨

### ▶ 실행 명령어

#### 그리퍼 연결

```bash
roslaunch onrobot_rg_control bringup.launch gripper:=rg2 ip:=192.168.1.1
```

#### 추론 실행

```bash
conda activate aloha
rosrun custom_act custom_imitate_episodes.py \
--eval \
--temporal_agg \
--task_name rb_transfer_can \
--ckpt_dir /home/vision/catkin_ws/src/custom_act/src/act/checkpoint/rb_transfer_can_scripted \
--policy_class ACT --kl_weight 10 --chunk_size 100 --hidden_dim 512 \
--batch_size 8 --dim_feedforward 3200 --num_epochs 2000 --lr 1e-5 --seed 0
```

---

## 🧰 환경

- **Camera**: D435, D405  
- **Manipulator**: RB10  
- **Gripper**: OnRobot RG2
