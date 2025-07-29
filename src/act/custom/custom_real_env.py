# Author: Chemin Ahn (chemx3937@gmail.com)
# Use of this source code is governed by the MIT, see LICENSE


import time
import numpy as np
import collections
import matplotlib.pyplot as plt
import dm_env

from custom.custom_constants import DT, START_ARM_POSE
from custom.custom_robot_utils import get_device_serials, Recorder, ImageRecorder, MAX_GRIP



import IPython
e = IPython.embed

class RealEnv:
    """
    Environment for real robot single manipulation
    Action space:      [qpos (6),             # absolute joint position
                        gripper_positions (1),    # normalized gripper position (0: close, 1: open)]

    Observation space: {"qpos": Concat[ qpos (6),          # absolute joint position
                                        gripper_position (1)],  # normalized gripper position (0: close, 1: open)
                        "qvel": Concat[ zeros]  #사용 X
                        "images": {"cam_high": (480x640x3),        # h, w, c, dtype='uint8'
                                   "cam_low": (480x640x3)},         # h, w, c, dtype='uint8'
    """

    # def __init__(self, init_node, setup_robots=True):
    def __init__(self, init_node = True):
        self.recorder_arm = Recorder(init_node=init_node)
        serials = get_device_serials()
        serial_d405 = serials[0]
        serial_d435 = serials[1]

        # camera 추가 시 해줄 부분
        # serial_d435 = serials[2]
        # serial_d435 = serials[3]

        # camera 추가 시 해줄 부분
        self.image_recorder = ImageRecorder(serial_d405, serial_d435)
        time.sleep(1.0)  # wait for cameras to start

    def get_qpos(self):
        return self.recorder_arm.get_qpos()

    # def get_qvel(self):
    #     qvel_raw = self.recorder_arm.qvel
    #     follower_arm_qvel = qvel_raw[:6]
    #     follower_gripper_qvel = [PUPPET_GRIPPER_VELOCITY_NORMALIZE_FN(qvel_raw[7])]
    #     return np.concatenate([follower_arm_qvel, follower_gripper_qvel])

    # def get_effort(self):
    #     effort_raw = self.recorder_arm.effort
    #     follower_robot_effort = effort_raw[:7]
    #     return np.concatenate([follower_robot_effort])

    def get_images(self):
        return self.image_recorder.get_images()

    def _reset_joints(self):
        reset_position = START_ARM_POSE[:6]
        self.recorder_arm.move_arm(reset_position, move_time=1.0)
        # move_arms([self.follower_bot], [reset_position], move_time=1)

    def _reset_gripper(self):

        half_closed = MAX_GRIP/2
        fully_open = MAX_GRIP

        self.recorder_arm.move_gripper(half_closed, move_time=0.5)
        self.recorder_arm.move_gripper(fully_open, move_time=1.0)

    # qvel, qeffort 추가시 해줄 부분
    def get_observation(self):
        obs = collections.OrderedDict()
        obs['qpos'] = self.get_qpos()
        # obs['qvel'] = self.get_qvel()
        # obs['effort'] = self.get_effort()
        obs['images'] = self.get_images()
        return obs

    def get_reward(self):
        return 0

    def reset(self, fake=False):
        if not fake:
            # Reboot puppet robot gripper motors
            # self.follower_bot.dxl.robot_reboot_motors("single", "gripper", True)
            self._reset_joints()
            self._reset_gripper()
        return dm_env.TimeStep(
            step_type=dm_env.StepType.FIRST,
            reward=self.get_reward(),
            discount=None,
            observation=self.get_observation())

    # step별 action을 실행시키는 부분으로 로봇 움직이게 함
    def step(self, action):
        follower_action = action
        self.recorder_arm.set_joint_positions(follower_action[:6])
        self.recorder_arm.set_gripper_pose(follower_action[-1])
        time.sleep(DT)
        return dm_env.TimeStep(
            step_type=dm_env.StepType.MID,
            reward=self.get_reward(),
            discount=None,
            observation=self.get_observation())

# custom에서 사용 X
# def get_action(Leader_bot):
#     action = np.zeros(14) # 6 joint + 1 gripper, for two recorder_arms
#     # recorder_arm actions
#     action[:6] = Leader_bot.dxl.joint_states.position[:6]
#     # action[7:7+6] = master_bot_right.dxl.joint_states.position[:6]
#     # Gripper actions
#     action[6] = MASTER_GRIPPER_JOINT_NORMALIZE_FN(Leader_bot.dxl.joint_states.position[6])
#     # action[7+6] = MASTER_GRIPPER_JOINT_NORMALIZE_FN(master_bot_right.dxl.joint_states.position[6])

#     return action

def make_real_env(init_node):
    env = RealEnv(init_node)
    return env


# custom에서 사용 X
# def test_real_teleop():
#     """
#     Test bimanual teleoperation and show image observations onscreen.
#     It first reads joint poses from both master recorder_arms.
#     Then use it as actions to step the environment.
#     The environment returns full observations including images.

#     An alternative approach is to have separate scripts for teleoperation and observation recording.
#     This script will result in higher fidelity (obs, action) pairs
#     """

#     onscreen_render = True
#     render_cam = 'cam_left_wrist'

#     # source of data
#     Leader_bot = InterbotixManipulatorXS(robot_model="wx250s", group_name="recorder_arm", gripper_name="gripper",
#                                               robot_name=f'master_left', init_node=True)
#     # master_bot_right = InterbotixManipulatorXS(robot_model="wx250s", group_name="recorder_arm", gripper_name="gripper",
#     #                                            robot_name=f'master_right', init_node=False)
#     setup_master_bot(Leader_bot)
#     # setup_master_bot(master_bot_right)

#     # setup the environment
#     env = make_real_env(init_node=False)
#     ts = env.reset(fake=True)
#     episode = [ts]
#     # setup visualization
#     if onscreen_render:
#         ax = plt.subplot()
#         plt_img = ax.imshow(ts.observation['images'][render_cam])
#         plt.ion()

#     for t in range(1000):
#         # action = get_action(Leader_bot, master_bot_right)
#         action = get_action(Leader_bot)
#         ts = env.step(action)
#         episode.append(ts)

#         if onscreen_render:
#             plt_img.set_data(ts.observation['images'][render_cam])
#             plt.pause(DT)
#         else:
#             time.sleep(DT)


if __name__ == '__main__':
    # test_real_teleop()
    pass

