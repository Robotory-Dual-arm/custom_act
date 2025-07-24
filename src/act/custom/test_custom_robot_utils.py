#!/home/vision/anaconda3/envs/aloha/bin/python

# Author: Chemin Ahn (chemx3937@gmail.com)
# Use of this source code is governed by the MIT, see LICENSE

#  send_joint_command()와 send_gripper_command() 함수 구현 (실제 API 또는 통신 방식) <- 완
#  초기화 및 리셋 함수 (move_arms, move_grippers)도 환경에 맞게 구성 <- 완
#  inference에서 get_qpos() → policy → set_joint_positions, set_gripper_pose()로 연결되는 흐름 구성 <- 완

# get_images()로 비동기 방식으로 계속 camera data 갖고옴

# cam_low: D405 - serials[0]
# cam_high: D435 - serials[1]

import pyrealsense2 as rs
import numpy as np

# DT = 0.05 # 20Hz (수정)
# DT = 0.02 # 50Hz(ACT 원본)


def get_device_serials():
    ctx = rs.context()
    serials = []
    for device in ctx.query_devices():
        serials.append(device.get_info(rs.camera_info.serial_number))
    if len(serials) < 2:
        raise RuntimeError("2개 이상의 Realsense 카메라가 연결되어 있어야 합니다.")
    print("Detected serials:", serials)
    return serials

# serials = get_device_serials()
# serial_d405 = serials[0]
# serial_d435 = serials[1]

import threading
import time

import pyrealsense2 as rs
import numpy as np
import threading
import time

class ImageRecorder:
    def __init__(self, serial_d405, serial_d435):
        print("[DEBUG] Initializing ImageRecorder")
        print(f"[DEBUG] serial_d405 = {serial_d405}, serial_d435 = {serial_d435}")

        self.cam_high_frame = None
        self.cam_low_frame = None
        self.running = True

        # D405
        self.pipeline0 = rs.pipeline()
        config0 = rs.config()
        config0.enable_device(serial_d405)
        config0.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        time.sleep(5.0)

        try:
            self.pipeline0.start(config0)
        except RuntimeError:
            print("Retrying D405 pipeline start after reconfig...")
            time.sleep(1.0)
            self.pipeline0.stop()
            self.pipeline0 = rs.pipeline()
            self.pipeline0.start(config0)
        time.sleep(1.0)

        for _ in range(30):
            try:
                self.pipeline0.wait_for_frames(timeout_ms=100)
            except:
                continue

        # D435
        self.pipeline1 = rs.pipeline()
        config1 = rs.config()
        config1.enable_device(serial_d435)
        config1.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline1.start(config1)
        time.sleep(1.0)

        print("[INFO] Waiting for camera streams to stabilize...")
        for _ in range(30):
            try:
                self.pipeline0.wait_for_frames(timeout_ms=1000)
                self.pipeline1.wait_for_frames(timeout_ms=1000)
            except Exception as e:
                print(f"[WARN] Skipping unstable frame: {e}")
            time.sleep(0.05)
        print("[INFO] Camera warm-up done.")

        # Start background thread
        self.thread = threading.Thread(target=self._update_frames, daemon=True)
        self.thread.start()

    def _update_frames(self):
        while self.running:
            try:
                frames0 = self.pipeline0.wait_for_frames(timeout_ms=1000)
                frames1 = self.pipeline1.wait_for_frames(timeout_ms=1000)
                color0 = frames0.get_color_frame()
                color1 = frames1.get_color_frame()
                if color0 and color1:
                    self.cam_low_frame = np.asanyarray(color0.get_data())
                    self.cam_high_frame = np.asanyarray(color1.get_data())
            except Exception as e:
                print(f"[WARN] Background frame update failed: {e}")
            time.sleep(0.01)  # 100Hz update rate

    def get_images(self):
        if self.cam_high_frame is None or self.cam_low_frame is None:
            print("[WARN] One or both frames not available yet.")
            return {'cam_high': None, 'cam_low': None}
        return {
            'cam_high': self.cam_high_frame.copy(),
            'cam_low': self.cam_low_frame.copy()
        }

    def shutdown(self):
        print("[INFO] Shutting down ImageRecorder...")
        self.running = False
        self.thread.join(timeout=2.0)
        self.pipeline0.stop()
        self.pipeline1.stop()
        print("[INFO] ImageRecorder shutdown complete.")


# class ImageRecorder:
#     def __init__(self, serial_d405, serial_d435):
#         print("[DEBUG] Initializing ImageRecorder")
#         print(f"[DEBUG] serial_d405 = {serial_d405}, serial_d435 = {serial_d435}")

#         # D405
#         self.pipeline0 = rs.pipeline()
#         config0 = rs.config()
#         config0.enable_device(serial_d405)
#         config0.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
#         time.sleep(5.0)
#         # self.pipeline0.start(config0)

#         try:
#             self.pipeline0.start(config0)
#         except RuntimeError:
#             print("Retrying D405 pipeline start after reconfig...")
#             time.sleep(1.0)
#             self.pipeline0.stop()
#             self.pipeline0 = rs.pipeline()
#             self.pipeline0.start(config0)
#         time.sleep(1.0)

#         # 추가적으로 warm-up
#         for _ in range(30):
#             try:
#                 self.pipeline0.wait_for_frames(timeout_ms=100)
#             except:
#                 continue
        
#         # D435
#         self.pipeline1 = rs.pipeline()
#         config1 = rs.config()
#         config1.enable_device(serial_d435)
#         config1.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
#         self.pipeline1.start(config1)
#         time.sleep(1.0)

#         # 안정화 대기 추가
#         print("[INFO] Waiting for camera streams to stabilize...")
#         for _ in range(30):
#             try:
#                 self.pipeline0.wait_for_frames(timeout_ms=1000)
#                 self.pipeline1.wait_for_frames(timeout_ms=1000)
#             except Exception as e:
#                 print(f"[WARN] Skipping unstable frame: {e}")
#             time.sleep(0.05)
#         print("[INFO] Camera warm-up done.")
    
#     # 디버깅 용도
#     def get_images(self):
#         max_retries = 10
#         for retry in range(max_retries):
#             try:
#                 print(f"[DEBUG] Trying to fetch frames (attempt {retry+1})")
#                 frames0 = self.pipeline0.wait_for_frames(timeout_ms=2000)
#                 frames1 = self.pipeline1.wait_for_frames(timeout_ms=2000)

#                 if not frames0 or not frames1:
#                     print("[DEBUG] One or more frames is None")
#                     continue

#                 color0 = frames0.get_color_frame()
#                 color1 = frames1.get_color_frame()
#                 if color0 is not None and color1 is not None:
#                     return {
#                         'cam_high': np.asanyarray(color0.get_data()),
#                         'cam_low': np.asanyarray(color1.get_data()),
#                     }

#             except Exception as e:
#                 print(f"[WARN] Frame fetch failed (attempt {retry+1}): {e}")
#                 time.sleep(0.1)

#         print("[ERROR] Failed to get images after retries")
#         return {'cam_high': None, 'cam_low': None}

    # def get_images(self):
    #     for retry in range(3):
    #         try:
    #             # 
    #             print("[DEBUG] Trying to fetch frames")
    #             # 
    #             frames0 = self.pipeline0.wait_for_frames(timeout_ms=2000)
    #             frames1 = self.pipeline1.wait_for_frames(timeout_ms=2000)
                
    #             # 
    #             if not frames0 or not frames1:
    #                 print("[DEBUG] Frame(s) is None")
    #                 return {'cam_high': None, 'cam_low': None}
    #             # 
    #             color0 = frames0.get_color_frame()
    #             color1 = frames1.get_color_frame()
    #             if color0 and color1:
    #                 return {
    #                     'cam_high': np.asanyarray(color0.get_data()),
    #                     'cam_low': np.asanyarray(color1.get_data()),
    #                 }
    #         except Exception as e:
    #             print(f"[WARN] Frame fetch failed: {e}")
    #             time.sleep(0.1)
    #     print("[ERROR] Failed to get images after retries")
    #     return {'cam_high': None, 'cam_low': None}
    
    # def get_images(self):
    #     print("[DEBUG] waiting for frame0")
    #     try:
    #         frames0 = self.pipeline0.wait_for_frames(timeout_ms=2000)
    #         print("[DEBUG] frame0 received")
    #     except Exception as e:
    #         print(f"[ERROR] frame0 failed: {e}")
    #         frames0 = None

    #     print("[DEBUG] waiting for frame1")
    #     try:
    #         frames1 = self.pipeline1.wait_for_frames(timeout_ms=2000)
    #         print("[DEBUG] frame1 received")
    #     except Exception as e:
    #         print(f"[ERROR] frame1 failed: {e}")
    #         frames1 = None

    #     color0 = frames0.get_color_frame() if frames0 else None
    #     color1 = frames1.get_color_frame() if frames1 else None

    #     if not color0:
    #         print("[WARN] color0 is None")
    #     if not color1:
    #         print("[WARN] color1 is None")

    #     image0 = np.asanyarray(color0.get_data()) if color0 else None
    #     image1 = np.asanyarray(color1.get_data()) if color1 else None

    #     return {'cam_high': image0, 'cam_low': image1}


    # 실제 사용 코드
    # def get_images(self):
    #     try:
    #         frames0 = self.pipeline0.wait_for_frames(timeout_ms=20000)
    #         frames1 = self.pipeline1.wait_for_frames(timeout_ms=20000)
    #         color0 = frames0.get_color_frame()
    #         color1 = frames1.get_color_frame()

    #         image0 = np.asanyarray(color0.get_data()) if color0 else None
    #         image1 = np.asanyarray(color1.get_data()) if color1 else None

    #         return {'cam_high': image0, 'cam_low': image1}
    #     except Exception as e:
    #         rospy.logwarn(f"[Realsense] Frame timeout or error: {e}")
    #         return {'cam_high': None, 'cam_low': None}

    # def get_images(self):
    #     while True:
    #         try:
    #             frames0 = self.pipeline0.wait_for_frames(timeout_ms=5000)
    #             frames1 = self.pipeline1.wait_for_frames(timeout_ms=5000)

    #             color0 = frames0.get_color_frame()
    #             color1 = frames1.get_color_frame()

    #             image0 = np.asanyarray(color0.get_data()) if color0 else None
    #             image1 = np.asanyarray(color1.get_data()) if color1 else None

    #             if image0 is not None and image1 is not None:
    #                 return {'cam_high': image0, 'cam_low': image1}
    #             else:
    #                 print("[WARN] One or both images are None. Retrying...")
    #         except Exception as e:
    #             print(f"[ERROR] Realsense image fetch failed: {e}")
    #             time.sleep(0.1)




import numpy as np
import rospy
import time

# 둘중 어떤 거 사용하는지 확인 필요
# from teleop_data.msg import OnRobotRGOutput
# from custom_act.msg import OnRobotRGOutput
from robotory_rb10_rt.msg import OnRobotRGInput, OnRobotRGOutput
from custom.custom_constants import DT


import sys
sys.path.append('/home/vision/catkin_ws/src/robotory_rb10_rt/scripts')  # 필요시 조정
from api.cobot import GetCurrentSplitedJoint, SendCOMMAND, CMD_TYPE, ToCB, SetProgramMode, PG_MODE


MAX_GRIP = 1100.0


class Recorder:
    def __init__(self, init_node=True, is_debug=False):
        print("[DEBUG] Initializing Recorder")
        self.is_debug = is_debug
        self.qpos = None
        self.joint_deg = None
        self.curr_gripper = None       # gGWD: 현재 상태
        self.prev_gripper = None       # 이전 상태
        self.gripper_pub = None

        ToCB("192.168.111.50")  # ← 이 줄을 추가

        mode = input("real? (y/n): ")
        if mode == 'y':
            SetProgramMode(PG_MODE.REAL)
        else:
            SetProgramMode(PG_MODE.SIMULATION)

        if init_node:
            rospy.init_node("custom_recorder", anonymous=True)
        rospy.Subscriber("/OnRobotRGInput", OnRobotRGInput, self._gripper_cb)

        if self.is_debug:
            from collections import deque
            self.gripper_log = deque(maxlen=50)
        time.sleep(0.1)

    def _gripper_cb(self, msg):
        # self.curr_gripper = msg.rGWD
        self.curr_gripper = msg.gGWD
        if self.prev_gripper is None:
            self.prev_gripper = self.curr_gripper
        if self.is_debug:
            self.gripper_log.append(time.time())

    def update_joint(self):
        self.joint_deg = GetCurrentSplitedJoint()

    def get_qpos(self):
        """
        조인트 각도 (rad) + Δgripper 정규화된 값 반환
        """
        self.update_joint()
        joint_rad = np.array(self.joint_deg[:6]) * np.pi / 180.0

        if self.curr_gripper is None:
            raise RuntimeError("Gripper data not received yet")

        # MAX_GRIP = 1100.0
        if self.prev_gripper is None:
            delta_grip_norm = 0.0
        else:
            delta_grip_norm = (self.curr_gripper - self.prev_gripper) / MAX_GRIP
        self.prev_gripper = self.curr_gripper
        
        curr_qpos = np.concatenate([joint_rad, [delta_grip_norm]])
        print(f"[DEBUG] Current qpos: {curr_qpos}")
        return curr_qpos

    def set_joint_positions(self, joint_rad: np.ndarray, delta_theta_max=np.deg2rad(5.0)):
        """
        joint_rad (6,) → degree 변환 → cobot API 전송
        """
        if joint_rad.shape != (6,):
            raise ValueError("Expected shape (6,), got", joint_rad.shape)

        # 현재 조인트 상태
        current_joint_rad = np.array(GetCurrentSplitedJoint())[:6] * np.pi / 180.0

        # Δθ 계산 및 제한
        delta = joint_rad - current_joint_rad
        delta_clipped = np.clip(delta, -delta_theta_max, delta_theta_max)
        clipped_target_rad = current_joint_rad + delta_clipped

        # rad → deg 변환
        joint_deg = clipped_target_rad * 180.0 / np.pi

        # ServoJ(joint_deg, time1=0.002, time2=0.1, gain=0.02, lpf_gain=0.2)
        msg = f"move_servo_j(jnt[{','.join(f'{j:.3f}' for j in joint_deg)}],0.002,0.1,0.02,0.2)"
        SendCOMMAND(msg, CMD_TYPE.MOVE)

    def set_gripper_pose(self, delta_grip_norm: float):
        """
        Δgripper 정규화 값을 받아 이전 상태 기준으로 이동
        """

        if self.curr_gripper is None:
            raise RuntimeError("Gripper state not initialized")

        # 목표 gripper width 계산
        target_gripper = self.curr_gripper + delta_grip_norm * MAX_GRIP
        target_gripper = np.clip(target_gripper, 0, MAX_GRIP)

        # publisher가 없으면 생성
        if self.gripper_pub is None:
            self.gripper_pub = rospy.Publisher("OnRobotRGOutput", OnRobotRGOutput, queue_size=10)
            time.sleep(0.1)

        # 명령 메시지 생성
        cmd = OnRobotRGOutput()
        cmd.rGWD = int(target_gripper)  # 목표 폭
        cmd.rGFR = 400                 # 일정한 그립 force
        cmd.rCTR = 16                  # position control mode

        # publish
        self.gripper_pub.publish(cmd)

        # 상태 갱신
        self.prev_gripper = self.curr_gripper
        self.curr_gripper = target_gripper

    def print_diagnostics(self):
        if self.is_debug and len(self.gripper_log) > 1:
            diffs = np.diff(np.array(self.gripper_log))
            print(f"[Gripper ROS Hz] ~{1 / np.mean(diffs):.2f} Hz")
        else:
            print("No gripper diagnostics available.")

    # 추가해야 되는 것
    # reset 시킬때만 Joint 움직이게 하는 메소드
    def move_arm(self, target_pose, move_time=1.0, delta_theta_max=np.deg2rad(5.5)):
        # from custom_constants import DT

        """
        로봇 조인트를 target_pose로 천천히 이동 (reset용)
        :param target_pose: (6,) array, radian 단위
        """
        num_steps = int(move_time / DT)
        current_pose = self.get_qpos()[:6]
        traj = np.linspace(current_pose, target_pose, num_steps)
        for step_pose in traj:
            delta = step_pose - self.get_qpos()[:6]
            delta = np.clip(delta, -delta_theta_max, delta_theta_max)
            limited_pose = self.get_qpos()[:6] + delta
            self.set_joint_positions(limited_pose)
            print(f"[DEBUG] Moving arm to {limited_pose}")
            time.sleep(DT)

    def move_gripper(self, target_grip, move_time=1.0):
        # from custom_constants import DT
        """
        gripper를 target_grip (절대 width 값, rGWD 기준)으로 천천히 이동 (reset용)
        :param target_grip: int or float (절대 단위, 0 ~ MAX_GRIP)
        """
        num_steps = int(move_time / DT)
        if self.curr_gripper is None:
            rospy.logwarn("Gripper state not initialized; skipping move_gripper")
            return

        curr_grip = self.curr_gripper
        traj = np.linspace(curr_grip, target_grip, num_steps)
        for g in traj:
            delta_grip_norm = (g - self.curr_gripper) / MAX_GRIP
            self.set_gripper_pose(delta_grip_norm)
            print(f"[DEBUG] Moving gripper to {delta_grip_norm}")
            time.sleep(DT)

# Cam 두개 제대로 연결되는 지 확인하는 용도
if __name__ == "__main__":
    import cv2

    serials = get_device_serials()
    serial_d405 = serials[0]
    serial_d435 = serials[1]

    image_recorder = ImageRecorder(serial_d405, serial_d435)

    while True:
        images = image_recorder.get_images()
        cam_low = images['cam_low']
        cam_high = images['cam_high']

        if cam_low is not None:
            cv2.imshow('cam_low (D405)', cam_low)
        if cam_high is not None:
            cv2.imshow('cam_high (D435)', cam_high)

        key = cv2.waitKey(1)
        if key == ord('q'):
            break

    cv2.destroyAllWindows()
