# 5.19记录。 相较于 robot_hand_inspire copy.py，主要增加了以下功能：使用PICO手柄扳机控制灵巧手开合
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize # dds
from unitree_sdk2py.idl.unitree_go.msg.dds_ import MotorCmds_, MotorStates_                           # idl
from unitree_sdk2py.idl.default import unitree_go_msg_dds__MotorCmd_
from teleop.robot_control.hand_retargeting import HandRetargeting, HandType
import numpy as np
from enum import IntEnum
import threading
import time
from multiprocessing import Process, Array

import logging_mp
logger_mp = logging_mp.getLogger(__name__)

Inspire_Num_Motors = 6
kTopicInspireDFXCommand = "rt/inspire/cmd"
kTopicInspireDFXState = "rt/inspire/state"

# [5.19新增]”direct_cmd_array = None“ 参数
# direct_cmd_array 是主进程（teleop_hand_and_arm.py）和手控子进程（control_process）之间的通信通道。它和 left_hand_array、right_hand_array 是同一类东西——都是 multiprocessing.Array，跨进程共享内存。
# 参数加了默认值 None，所以旧的调用方式（不传这个参数）仍然能正常工作，不会破坏兼容性。
class Inspire_Controller_DFX:
    def __init__(self, left_hand_array, right_hand_array, dual_hand_data_lock = None, dual_hand_state_array = None,
                       dual_hand_action_array = None, direct_cmd_array = None, fps = 100.0, Unit_Test = False, simulation_mode = False):
        logger_mp.info("Initialize Inspire_Controller_DFX...")
        self.fps = fps
        self.Unit_Test = Unit_Test
        self.simulation_mode = simulation_mode
        if not self.Unit_Test:
            self.hand_retargeting = HandRetargeting(HandType.INSPIRE_HAND)
        else:
            self.hand_retargeting = HandRetargeting(HandType.INSPIRE_HAND_Unit_Test)


        # initialize handcmd publisher and handstate subscriber
        self.HandCmb_publisher = ChannelPublisher(kTopicInspireDFXCommand, MotorCmds_)
        self.HandCmb_publisher.Init()

        self.HandState_subscriber = ChannelSubscriber(kTopicInspireDFXState, MotorStates_)
        self.HandState_subscriber.Init()

        # Shared Arrays for hand states
        self.left_hand_state_array  = Array('d', Inspire_Num_Motors, lock=True)  
        self.right_hand_state_array = Array('d', Inspire_Num_Motors, lock=True)

        # initialize subscribe thread
        self.subscribe_state_thread = threading.Thread(target=self._subscribe_hand_state)
        self.subscribe_state_thread.daemon = True
        self.subscribe_state_thread.start()

        while True:
            if any(self.right_hand_state_array): # any(self.left_hand_state_array) and 
                break
            time.sleep(0.01)
            logger_mp.warning("[Inspire_Controller_DFX] Waiting to subscribe dds...")
        logger_mp.info("[Inspire_Controller_DFX] Subscribe dds ok.")

        # [5.19新增] “direct_cmd_array”参数
        # Process(target=self.control_process, args=(...)) 创建了一个新的操作系统进程。args 元组里的每个元素会被传给 control_process 作为函数参数。
        # 这和普通函数调用不同——普通函数调用传的是引用（同一进程），而这里是把 multiprocessing.Array 对象从主进程传给子进程。Python 的 multiprocessing模块会用系统级的共享内存机制（mmap）来保证两个进程读写同一块物理内存。
        # direct_cmd_array 作为 __init__ 的参数，当前在 self 作用域里。但 control_process 运行在另一个进程里，不能访问 self。所以必须显式通过 args 传过去。
        hand_control_process = Process(target=self.control_process, args=(left_hand_array, right_hand_array,  self.left_hand_state_array, self.right_hand_state_array,
                                                                          dual_hand_data_lock, dual_hand_state_array, dual_hand_action_array,direct_cmd_array))
        hand_control_process.daemon = True
        hand_control_process.start()

        logger_mp.info("Initialize Inspire_Controller_DFX OK!")

    def _subscribe_hand_state(self):
        while True:
            hand_msg  = self.HandState_subscriber.Read()
            if hand_msg is not None:
                for idx, id in enumerate(Inspire_Left_Hand_JointIndex):
                    self.left_hand_state_array[idx] = hand_msg.states[id].q
                for idx, id in enumerate(Inspire_Right_Hand_JointIndex):
                    self.right_hand_state_array[idx] = hand_msg.states[id].q
            time.sleep(0.002)

    def ctrl_dual_hand(self, left_q_target, right_q_target):
        """
        Set current left, right hand motor state target q
        """
        for idx, id in enumerate(Inspire_Left_Hand_JointIndex):             
            self.hand_msg.cmds[id].q = left_q_target[idx]         
        for idx, id in enumerate(Inspire_Right_Hand_JointIndex):             
            self.hand_msg.cmds[id].q = right_q_target[idx] 

        self.HandCmb_publisher.Write(self.hand_msg)
        # logger_mp.debug("hand ctrl publish ok.")
    
    # [5.19新增] “direct_cmd_array = None”参数
    # 在init里调用的control_process的args里多传了“direct_cmd_array“参数，对应函数签名这里也应该新增，且顺序必须一致。
    def control_process(self, left_hand_array, right_hand_array, left_hand_state_array, right_hand_state_array,
                              dual_hand_data_lock = None, dual_hand_state_array = None, dual_hand_action_array = None, direct_cmd_array = None):
        self.running = True

        left_q_target  = np.full(Inspire_Num_Motors, 1000.0)
        right_q_target = np.full(Inspire_Num_Motors, 1000.0)

        # initialize inspire hand's cmd msg
        self.hand_msg  = MotorCmds_()
        self.hand_msg.cmds = [unitree_go_msg_dds__MotorCmd_() for _ in range(len(Inspire_Right_Hand_JointIndex) + len(Inspire_Left_Hand_JointIndex))]

        for idx, id in enumerate(Inspire_Left_Hand_JointIndex):
            self.hand_msg.cmds[id].q = 1000.0
        for idx, id in enumerate(Inspire_Right_Hand_JointIndex):
            self.hand_msg.cmds[id].q = 1000.0

        try:
            while self.running:
                start_time = time.time()

                # [5.19新增] 优先检查是否有手柄扳机直接命令（不走手势重定向）
                if direct_cmd_array is not None: #- 兼容性检查。如果旧代码没传这个参数（比如 IPC 模式），它就是 None，直接跳过整个分支走原来的手势重定向，不会出错。
                    with direct_cmd_array.get_lock(): #和主进程约定好的数据集约：direct_cmd_array 是一个 multiprocessing.Array('d', 12)，12 个 double：[0:6] 是左手 6 个手指目标角度，[6:12] 是右手。
                        direct_cmd = np.array(direct_cmd_array[:]).copy() #自己加锁读一次然后 .copy()，确保拿到一致的数据快照，后续计算不依赖锁。
                    if np.any(direct_cmd >= 0): #这是判断"是否有扳机命令"的关键。主进程不用扳机时，数组全部是 -1.0。np.any(direct_cmd >= 0) 对全 -1 的数组返回 False，走手势重定向；扳机一旦按下，至少有一个值 >= 0，返回 True，走直接命令。
                        left_q_target = direct_cmd[:6] #直接拿扳机值赋给目标变量，跳过整个手势重定向的计算。
                        right_q_target = direct_cmd[6:]
                        # 写状态和动作到共享数组，供录制使用
                        # 这段和原代码（下面的）做的事一样：把状态和动作写进共享数组，供 teleop_hand_and_arm.py 的主进程在录制时读取。因为 continue 跳过了原有的这段写入代码，所以这里必须补上。
                        state_data = np.concatenate((np.array(left_hand_state_array[:]), np.array(right_hand_state_array[:])))
                        action_data = np.concatenate((left_q_target, right_q_target))
                        if dual_hand_state_array and dual_hand_action_array:
                            with dual_hand_data_lock:
                                dual_hand_state_array[:] = state_data
                                dual_hand_action_array[:] = action_data
                        self.ctrl_dual_hand(left_q_target, right_q_target) # 和原有代码一样，把目标角度通过 DDS 下发给机器人硬件。
                        current_time = time.time()
                        time_elapsed = current_time - start_time
                        sleep_time = max(0, (1 / self.fps) - time_elapsed) # 维持 100Hz 的控制频率。因为 continue 跳过了原有的 sleep 代码（line 147-150），这里必须补上。
                        time.sleep(sleep_time) 
                        continue #跳回 while 开头，不执行 后面的手势重定向+normalize 逻辑。

                # get dual hand state
                with left_hand_array.get_lock():
                    left_hand_data  = np.array(left_hand_array[:]).reshape(25, 3).copy()
                with right_hand_array.get_lock():
                    right_hand_data = np.array(right_hand_array[:]).reshape(25, 3).copy()

                # Read left and right q_state from shared arrays
                state_data = np.concatenate((np.array(left_hand_state_array[:]), np.array(right_hand_state_array[:])))

                if not np.all(right_hand_data == 0.0) and not np.all(left_hand_data[4] == np.array([-1.13, 0.3, 0.15])): # if hand data has been initialized.
                    ref_left_value = left_hand_data[self.hand_retargeting.left_indices[1,:]] - left_hand_data[self.hand_retargeting.left_indices[0,:]]
                    ref_right_value = right_hand_data[self.hand_retargeting.right_indices[1,:]] - right_hand_data[self.hand_retargeting.right_indices[0,:]]

                    left_q_target  = self.hand_retargeting.left_retargeting.retarget(ref_left_value)[self.hand_retargeting.left_dex_retargeting_to_hardware]
                    right_q_target = self.hand_retargeting.right_retargeting.retarget(ref_right_value)[self.hand_retargeting.right_dex_retargeting_to_hardware]

                    # In website https://support.unitree.com/home/en/G1_developer/inspire_dfx_dexterous_hand, you can find
                    #     In the official document, the angles are in the range [0, 1] ==> 0.0: fully closed  1.0: fully open
                    # The q_target now is in radians, ranges:
                    #     - idx 0~3: 0~1.7 (1.7 = closed)
                    #     - idx 4:   0~0.5
                    #     - idx 5:  -0.1~1.3
                    # We normalize them using (max - value) / range, scaled to [0, 1000] for inspire_g1_new driver
                    def normalize(val, min_val, max_val):
                        return np.clip((max_val - val) / (max_val - min_val) * 1000.0, 0.0, 1000.0)

                    for idx in range(Inspire_Num_Motors):
                        if idx <= 3:
                            left_q_target[idx]  = normalize(left_q_target[idx], 0.0, 1.7)
                            right_q_target[idx] = normalize(right_q_target[idx], 0.0, 1.7)
                        elif idx == 4:
                            left_q_target[idx]  = normalize(left_q_target[idx], 0.0, 0.5)
                            right_q_target[idx] = normalize(right_q_target[idx], 0.0, 0.5)
                        elif idx == 5:
                            left_q_target[idx]  = normalize(left_q_target[idx], -0.1, 1.3)
                            right_q_target[idx] = normalize(right_q_target[idx], -0.1, 1.3)

                # get dual hand action
                action_data = np.concatenate((left_q_target, right_q_target))    
                if dual_hand_state_array and dual_hand_action_array:
                    with dual_hand_data_lock:
                        dual_hand_state_array[:] = state_data
                        dual_hand_action_array[:] = action_data

                self.ctrl_dual_hand(left_q_target, right_q_target)
                current_time = time.time()
                time_elapsed = current_time - start_time
                sleep_time = max(0, (1 / self.fps) - time_elapsed)
                time.sleep(sleep_time)
        finally:
            logger_mp.info("Inspire_Controller_DFX has been closed.")



kTopicInspireFTPLeftCommand   = "rt/inspire_hand/ctrl/l"
kTopicInspireFTPRightCommand  = "rt/inspire_hand/ctrl/r"
kTopicInspireFTPLeftState  = "rt/inspire_hand/state/l"
kTopicInspireFTPRightState = "rt/inspire_hand/state/r"

class Inspire_Controller_FTP:
    def __init__(self, left_hand_array, right_hand_array, dual_hand_data_lock = None, dual_hand_state_array = None,
                       dual_hand_action_array = None, fps = 100.0, Unit_Test = False, simulation_mode = False):
        logger_mp.info("Initialize Inspire_Controller_FTP...")
        from inspire_sdkpy import inspire_dds  # lazy import
        import inspire_sdkpy.inspire_hand_defaut as inspire_hand_default
        self.fps = fps
        self.Unit_Test = Unit_Test
        self.simulation_mode = simulation_mode
        if not self.Unit_Test:
            self.hand_retargeting = HandRetargeting(HandType.INSPIRE_HAND)
        else:
            self.hand_retargeting = HandRetargeting(HandType.INSPIRE_HAND_Unit_Test)


        # Initialize hand command publishers
        self.LeftHandCmd_publisher = ChannelPublisher(kTopicInspireFTPLeftCommand, inspire_dds.inspire_hand_ctrl)
        self.LeftHandCmd_publisher.Init()
        self.RightHandCmd_publisher = ChannelPublisher(kTopicInspireFTPRightCommand, inspire_dds.inspire_hand_ctrl)
        self.RightHandCmd_publisher.Init()

        # Initialize hand state subscribers
        self.LeftHandState_subscriber = ChannelSubscriber(kTopicInspireFTPLeftState, inspire_dds.inspire_hand_state)
        self.LeftHandState_subscriber.Init() # Consider using callback if preferred: Init(callback_func, period_ms)
        self.RightHandState_subscriber = ChannelSubscriber(kTopicInspireFTPRightState, inspire_dds.inspire_hand_state)
        self.RightHandState_subscriber.Init()

        # Shared Arrays for hand states ([0,1] normalized values)
        self.left_hand_state_array  = Array('d', Inspire_Num_Motors, lock=True)
        self.right_hand_state_array = Array('d', Inspire_Num_Motors, lock=True)

        # Initialize subscribe thread
        self.subscribe_state_thread = threading.Thread(target=self._subscribe_hand_state)
        self.subscribe_state_thread.daemon = True
        self.subscribe_state_thread.start()

        # Wait for initial DDS messages (optional, but good for ensuring connection)
        wait_count = 0
        while not (any(self.left_hand_state_array) or any(self.right_hand_state_array)):
            if wait_count % 100 == 0: # Print every second
                logger_mp.info(f"[Inspire_Controller_FTP] Waiting to subscribe to hand states from DDS (L: {any(self.left_hand_state_array)}, R: {any(self.right_hand_state_array)})...")
            time.sleep(0.01)
            wait_count += 1
            if wait_count > 500: # Timeout after 5 seconds
                logger_mp.warning("[Inspire_Controller_FTP] Warning: Timeout waiting for initial hand states. Proceeding anyway.")
                break
        logger_mp.info("[Inspire_Controller_FTP] Initial hand states received or timeout.")

        hand_control_process = Process(target=self.control_process, args=(left_hand_array, right_hand_array, self.left_hand_state_array, self.right_hand_state_array,
                                                                          dual_hand_data_lock, dual_hand_state_array, dual_hand_action_array))
        hand_control_process.daemon = True
        hand_control_process.start()

        logger_mp.info("Initialize Inspire_Controller_FTP OK!\n")

    def _subscribe_hand_state(self):
        logger_mp.info("[Inspire_Controller_FTP] Subscribe thread started.")
        while True:
            # Left Hand
            left_state_msg = self.LeftHandState_subscriber.Read()
            if left_state_msg is not None:
                if hasattr(left_state_msg, 'angle_act') and len(left_state_msg.angle_act) == Inspire_Num_Motors:
                    with self.left_hand_state_array.get_lock():
                        for i in range(Inspire_Num_Motors):
                            self.left_hand_state_array[i] = left_state_msg.angle_act[i] / 1000.0
                else:
                    logger_mp.warning(f"[Inspire_Controller_FTP] Received left_state_msg but attributes are missing or incorrect. Type: {type(left_state_msg)}, Content: {str(left_state_msg)[:100]}")
            # Right Hand
            right_state_msg = self.RightHandState_subscriber.Read()
            if right_state_msg is not None:
                if hasattr(right_state_msg, 'angle_act') and len(right_state_msg.angle_act) == Inspire_Num_Motors:
                    with self.right_hand_state_array.get_lock():
                        for i in range(Inspire_Num_Motors):
                            self.right_hand_state_array[i] = right_state_msg.angle_act[i] / 1000.0
                else:
                    logger_mp.warning(f"[Inspire_Controller_FTP] Received right_state_msg but attributes are missing or incorrect. Type: {type(right_state_msg)}, Content: {str(right_state_msg)[:100]}")

            time.sleep(0.002)

    def _send_hand_command(self, left_angle_cmd_scaled, right_angle_cmd_scaled):
        """
        Send scaled angle commands [0-1000] to both hands.
        """
        # Left Hand Command
        left_cmd_msg = inspire_hand_default.get_inspire_hand_ctrl()
        left_cmd_msg.angle_set = left_angle_cmd_scaled
        left_cmd_msg.mode = 0b0001 # Mode 1: Angle control
        self.LeftHandCmd_publisher.Write(left_cmd_msg)

        # Right Hand Command
        right_cmd_msg = inspire_hand_default.get_inspire_hand_ctrl()
        right_cmd_msg.angle_set = right_angle_cmd_scaled
        right_cmd_msg.mode = 0b0001 # Mode 1: Angle control
        self.RightHandCmd_publisher.Write(right_cmd_msg)

        # 临时打开前 N 次的 log
        if not hasattr(self, "_debug_count"):
            self._debug_count = 0
        if self._debug_count < 50:
            logger_mp.info(f"[Inspire_Controller_FTP] Publish cmd L={left_angle_cmd_scaled} R={right_angle_cmd_scaled} ")
            self._debug_count += 1


    def control_process(self, left_hand_array, right_hand_array, left_hand_state_array, right_hand_state_array,
                              dual_hand_data_lock = None, dual_hand_state_array = None, dual_hand_action_array = None):
        logger_mp.info("[Inspire_Controller_FTP] Control process started.")
        self.running = True

        left_q_target  = np.full(Inspire_Num_Motors, 1.0)
        right_q_target = np.full(Inspire_Num_Motors, 1.0)

        try:
            while self.running:
                start_time = time.time()
                # get dual hand state
                with left_hand_array.get_lock():
                    left_hand_data  = np.array(left_hand_array[:]).reshape(25, 3).copy()
                with right_hand_array.get_lock():
                    right_hand_data = np.array(right_hand_array[:]).reshape(25, 3).copy()

                # Read left and right q_state from shared arrays
                state_data = np.concatenate((np.array(left_hand_state_array[:]), np.array(right_hand_state_array[:])))

                if not np.all(right_hand_data == 0.0) and not np.all(left_hand_data[4] == np.array([-1.13, 0.3, 0.15])): # if hand data has been initialized.
                    ref_left_value = left_hand_data[self.hand_retargeting.left_indices[1,:]] - left_hand_data[self.hand_retargeting.left_indices[0,:]]
                    ref_right_value = right_hand_data[self.hand_retargeting.right_indices[1,:]] - right_hand_data[self.hand_retargeting.right_indices[0,:]]

                    left_q_target  = self.hand_retargeting.left_retargeting.retarget(ref_left_value)[self.hand_retargeting.left_dex_retargeting_to_hardware]
                    right_q_target = self.hand_retargeting.right_retargeting.retarget(ref_right_value)[self.hand_retargeting.right_dex_retargeting_to_hardware]

                    def normalize(val, min_val, max_val):
                        return np.clip((max_val - val) / (max_val - min_val), 0.0, 1.0)

                    for idx in range(Inspire_Num_Motors):
                        if idx <= 3:
                            left_q_target[idx]  = normalize(left_q_target[idx], 0.0, 1.7)
                            right_q_target[idx] = normalize(right_q_target[idx], 0.0, 1.7)
                        elif idx == 4:
                            left_q_target[idx]  = normalize(left_q_target[idx], 0.0, 0.5)
                            right_q_target[idx] = normalize(right_q_target[idx], 0.0, 0.5)
                        elif idx == 5:
                            left_q_target[idx]  = normalize(left_q_target[idx], -0.1, 1.3)
                            right_q_target[idx] = normalize(right_q_target[idx], -0.1, 1.3)

                scaled_left_cmd = [int(np.clip(val * 1000, 0, 1000)) for val in left_q_target]
                scaled_right_cmd = [int(np.clip(val * 1000, 0, 1000)) for val in right_q_target]

                # get dual hand action
                action_data = np.concatenate((left_q_target, right_q_target))
                if dual_hand_state_array and dual_hand_action_array:
                    with dual_hand_data_lock:
                        dual_hand_state_array[:] = state_data
                        dual_hand_action_array[:] = action_data

                self._send_hand_command(scaled_left_cmd, scaled_right_cmd)
                current_time = time.time()
                time_elapsed = current_time - start_time
                sleep_time = max(0, (1 / self.fps) - time_elapsed)
                time.sleep(sleep_time)
        finally:
            logger_mp.info("Inspire_Controller_FTP has been closed.")

# Update hand state, according to the official documentation:
# 1. https://support.unitree.com/home/en/G1_developer/inspire_dfx_dexterous_hand
# 2. https://support.unitree.com/home/en/G1_developer/inspire_ftp_dexterity_hand
# the state sequence is as shown in the table below
# ┌──────┬───────┬──────┬────────┬────────┬────────────┬────────────────┬───────┬──────┬────────┬────────┬────────────┬────────────────┐
# │ Id   │   0   │  1   │   2    │   3    │     4      │       5        │   6   │  7   │   8    │   9    │    10      │       11       │
# ├──────┼───────┼──────┼────────┼────────┼────────────┼────────────────┼───────┼──────┼────────┼────────┼────────────┼────────────────┤
# │      │                    Right Hand                                │                   Left Hand                                  │
# │Joint │ pinky │ ring │ middle │ index  │ thumb-bend │ thumb-rotation │ pinky │ ring │ middle │ index  │ thumb-bend │ thumb-rotation │
# └──────┴───────┴──────┴────────┴────────┴────────────┴────────────────┴───────┴──────┴────────┴────────┴────────────┴────────────────┘
class Inspire_Right_Hand_JointIndex(IntEnum):
    kRightHandPinky = 0
    kRightHandRing = 1
    kRightHandMiddle = 2
    kRightHandIndex = 3
    kRightHandThumbBend = 4
    kRightHandThumbRotation = 5

class Inspire_Left_Hand_JointIndex(IntEnum):
    kLeftHandPinky = 6
    kLeftHandRing = 7
    kLeftHandMiddle = 8
    kLeftHandIndex = 9
    kLeftHandThumbBend = 10
    kLeftHandThumbRotation = 11
