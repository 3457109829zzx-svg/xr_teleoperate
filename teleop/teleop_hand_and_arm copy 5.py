# 5.18记录。 相较于 teleop_hand_and_arm.py，主要增加了以下功能：使用PICO手柄的按钮来控制开启/停止遥操作系统和开始/结束录制； ctrl+F 搜索 "新增" 可以看到所有新增代码的位置。 
# 5.19记录。 主要增加了以下功能：使用PICO手柄扳机控制灵巧手开合
# 5.23记录。 解决启动后末端位置突变的问题
# 5.24记录。 Y 键改为暂停/恢复：第一次按Y->启动遥操作，再按一次暂停但不退出程序，再按一次继续遥操作。退出换成X。 以及暂停恢复时重新标定 IK 偏移，确保每次恢复遥操作时末端位置都不会突变。 ctrl+F 搜索 "5.24" 可以看到所有相关代码的位置。
#    改动           │                      说明                      │               
#   ├─────────────────────────┼────────────────────────────────────────────────┤
#   │ 外层 while not          │ 包住等待循环和遥操作主循环，暂停后回到外层顶部          │
#   │ STOP:（第 271 行，注意是相对于copy3版本）      │                              │
#   ├─────────────────────────┼────────────────────────────────────────────────┤
#   │ IK_OFFSET_CALIBRATED =  │                                                │
#   │ False                   │ 每次从等待进入遥操作都重新标定偏移                   │
#   │ 移到外层循环内部（第    │                                                    │
#   │ 273 行）                │                                                 │
#   ├─────────────────────────┼────────────────────────────────────────────────┤
#   │ 等待循环缩进 +4 空格    │ 现在在外层 while 内部                                │
#   ├─────────────────────────┼────────────────────────────────────────────────┤
#   │ if STOP: break（第      │ X 键退出时跳出外层循环                             │
#   │ 290-291 行）            │                                                │
#   ├─────────────────────────┼────────────────────────────────────────────────┤
#   │ while START and not     │ 条件改为同时检查 START 和 STOP                    │
#   │ STOP:（第 296 行）      │                                                 │
#   ├─────────────────────────┼────────────────────────────────────────────────┤
#   │ 主循环体缩进 +4 空格    │ 现在在 while START 内部                             │
#   ├─────────────────────────┼────────────────────────────────────────────────┤
#   │ 删除了原来的            │ 已移到外层循环顶部，避免重复                          │
#   │ IK_OFFSET_CALIBRATED 行 │         
# 5.25记录。优化代码：1)无法设定最大采集组数 2)没有方便的按钮删除失败 

import time
import argparse
from multiprocessing import Value, Array, Lock
import threading
import logging_mp
import numpy as np
logging_mp.basicConfig(level=logging_mp.INFO)
logger_mp = logging_mp.getLogger(__name__)

import os 
import sys
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)

from unitree_sdk2py.core.channel import ChannelFactoryInitialize # dds 
from televuer import TeleVuerWrapper
from teleop.robot_control.robot_arm import G1_29_ArmController, G1_23_ArmController, H1_2_ArmController, H1_ArmController
from teleop.robot_control.robot_arm_ik import G1_29_ArmIK, G1_23_ArmIK, H1_2_ArmIK, H1_ArmIK
from teleimager.image_client import ImageClient
from teleop.utils.episode_writer import EpisodeWriter
from teleop.utils.ipc import IPC_Server
from teleop.utils.motion_switcher import MotionSwitcher, LocoClientWrapper
from sshkeyboard import listen_keyboard, stop_listening

# for simulation
from unitree_sdk2py.core.channel import ChannelPublisher
from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_
def publish_reset_category(category: int, publisher): # Scene Reset signal
    msg = String_(data=str(category))
    publisher.Write(msg)
    logger_mp.info(f"published reset category: {category}")

# state transition
START          = False  # Enable to start robot following VR user motion
STOP           = False  # Enable to begin system exit procedure
READY          = False  # Ready to (1) enter START state, (2) enter RECORD_RUNNING state
RECORD_RUNNING = False  # True if [Recording]
RECORD_TOGGLE  = False  # Toggle recording state
#  -------        ---------                -----------                -----------            ---------
#   state          [Ready]      ==>        [Recording]     ==>         [AutoSave]     -->     [Ready]
#  -------        ---------      |         -----------      |         -----------      |     ---------
#   START           True         |manual      True          |manual      True          |        True
#   READY           True         |set         False         |set         False         |auto    True
#   RECORD_RUNNING  False        |to          True          |to          False         |        False
#                                ∨                          ∨                          ∨
#   RECORD_TOGGLE   False       True          False        True          False                  False
#  -------        ---------                -----------                 -----------            ---------
#  ==> manual: when READY is True, set RECORD_TOGGLE=True to transition.
#  --> auto  : Auto-transition after saving data.

# 【5.18新增】PICO手柄按钮消抖（上升沿检测）
_last_Y_btn = False
_last_B_btn = False
# 【5.24新增】X键消抖（上升沿检测）
_last_X_btn = False
# 【5.25新增】A键消抖（上升沿检测）
_last_A_btn = False
# 【5.23新增】
IK_OFFSET_CALIBRATED = False  # 是否已记录初始偏移，全局标记
ik_offset = np.zeros(14)      # 偏移量数组，存放14个关节的偏移量（左右臂各7）



def on_press(key):
    global STOP, START, RECORD_TOGGLE
    if key == 'r':
        START = True
    elif key == 'q':
        START = False
        STOP = True
    elif key == 's' and START == True:
        RECORD_TOGGLE = True
    else:
        logger_mp.warning(f"[on_press] {key} was pressed, but no action is defined for this key.")

def get_state() -> dict:
    """Return current heartbeat state"""
    global START, STOP, RECORD_RUNNING, READY
    return {
        "START": START,
        "STOP": STOP,
        "READY": READY,
        "RECORD_RUNNING": RECORD_RUNNING,
    }

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    # basic control parameters
    parser.add_argument('--frequency', type = float, default = 30.0, help = 'control and record \'s frequency')
    parser.add_argument('--input-mode', type=str, choices=['hand', 'controller'], default='hand', help='Select XR device input tracking source')
    parser.add_argument('--display-mode', type=str, choices=['immersive', 'ego', 'pass-through'], default='immersive', help='Select XR device display mode')
    parser.add_argument('--arm', type=str, choices=['G1_29', 'G1_23', 'H1_2', 'H1'], default='G1_29', help='Select arm controller')
    parser.add_argument('--ee', type=str, choices=['dex1', 'dex3', 'inspire_ftp', 'inspire_dfx', 'brainco'], help='Select end effector controller')
    parser.add_argument('--img-server-ip', type=str, default='192.168.123.164', help='IP address of image server, used by teleimager and televuer')
    parser.add_argument('--network-interface', type=str, default=None, help='Network interface for dds communication, e.g., eth0, wlan0. If None, use default interface.')
    # mode flags
    parser.add_argument('--motion', action = 'store_true', help = 'Enable motion control mode')
    parser.add_argument('--headless', action='store_true', help='Enable headless mode (no display)')
    parser.add_argument('--sim', action = 'store_true', help = 'Enable isaac simulation mode')
    parser.add_argument('--ipc', action = 'store_true', help = 'Enable IPC server to handle input; otherwise enable sshkeyboard')
    parser.add_argument('--affinity', action = 'store_true', help = 'Enable high priority and set CPU affinity mode')
    # record mode and task info
    parser.add_argument('--record', action = 'store_true', help = 'Enable data recording mode')
    parser.add_argument('--task-dir', type = str, default = './utils/data/', help = 'path to save data')
    parser.add_argument('--task-name', type = str, default = 'pick cube', help = 'task file name for recording')
    parser.add_argument('--task-goal', type = str, default = 'pick up cube.', help = 'task goal for recording at json file')
    parser.add_argument('--task-desc', type = str, default = 'task description', help = 'task description for recording at json file')
    parser.add_argument('--task-steps', type = str, default = 'step1: do this; step2: do that;', help = 'task steps for recording at json file')
    parser.add_argument('--max-episodes', type=int, default=-1, help='max number of episodes to record, -1 means unlimited') # [5.25新增]：限制录制的最大 episode 数，防止误操作导致数据爆炸。默认 -1 表示不限制，想限制的话改成具体数字比如 10。
    args = parser.parse_args()
    logger_mp.info(f"args: {args}")

    try:
        # setup dds communication domains id
        if args.sim:
            ChannelFactoryInitialize(1, networkInterface=args.network_interface)
        else:
            ChannelFactoryInitialize(0, networkInterface=args.network_interface)

        # ipc communication mode. client usage: see utils/ipc.py
        if args.ipc:
            ipc_server = IPC_Server(on_press=on_press,get_state=get_state)
            ipc_server.start()
        # sshkeyboard communication mode
        else:
            listen_keyboard_thread = threading.Thread(target=listen_keyboard, 
                                                      kwargs={"on_press": on_press, "until": None, "sequential": False,}, 
                                                      daemon=True)
            listen_keyboard_thread.start()

        # image client
        img_client = ImageClient(host=args.img_server_ip, request_bgr=True)
        camera_config = img_client.get_cam_config()
        logger_mp.debug(f"Camera config: {camera_config}")
        xr_need_local_img = not (args.display_mode == 'pass-through' or camera_config['head_camera']['enable_webrtc'])

        # televuer_wrapper: obtain hand pose data from the XR device and transmit the robot's head camera image to the XR device.
        tv_wrapper = TeleVuerWrapper(use_hand_tracking=args.input_mode == "hand", 
                                     binocular=camera_config['head_camera']['binocular'],
                                     img_shape=camera_config['head_camera']['image_shape'],
                                     # maybe should decrease fps for better performance?
                                     # https://github.com/unitreerobotics/xr_teleoperate/issues/172
                                     # display_fps=camera_config['head_camera']['fps'] ? args.frequency? 30.0?
                                     display_mode=args.display_mode,
                                     zmq=camera_config['head_camera']['enable_zmq'],
                                     webrtc=camera_config['head_camera']['enable_webrtc'],
                                     webrtc_url=f"https://{args.img_server_ip}:{camera_config['head_camera']['webrtc_port']}/offer",
                                     )
        
        # motion mode (G1: Regular mode R1+X, not Running mode R2+A)
        if args.motion:
            if args.input_mode == "controller":
                loco_wrapper = LocoClientWrapper()
        else:
            motion_switcher = MotionSwitcher()
            status, result = motion_switcher.Enter_Debug_Mode()
            logger_mp.info(f"Enter debug mode: {'Success' if status == 0 else 'Failed'}")

        # arm
        if args.arm == "G1_29":
            arm_ik = G1_29_ArmIK()
            arm_ctrl = G1_29_ArmController(motion_mode=args.motion, simulation_mode=args.sim)
        elif args.arm == "G1_23":
            arm_ik = G1_23_ArmIK()
            arm_ctrl = G1_23_ArmController(motion_mode=args.motion, simulation_mode=args.sim)
        elif args.arm == "H1_2":
            arm_ik = H1_2_ArmIK()
            arm_ctrl = H1_2_ArmController(motion_mode=args.motion, simulation_mode=args.sim)
        elif args.arm == "H1":
            arm_ik = H1_ArmIK()
            arm_ctrl = H1_ArmController(simulation_mode=args.sim)

        # end-effector
        if args.ee == "dex3":
            from teleop.robot_control.robot_hand_unitree import Dex3_1_Controller
            left_hand_pos_array = Array('d', 75, lock = True)      # [input]
            right_hand_pos_array = Array('d', 75, lock = True)     # [input]
            dual_hand_data_lock = Lock()
            dual_hand_state_array = Array('d', 14, lock = False)   # [output] current left, right hand state(14) data.
            dual_hand_action_array = Array('d', 14, lock = False)  # [output] current left, right hand action(14) data.
            hand_ctrl = Dex3_1_Controller(left_hand_pos_array, right_hand_pos_array, dual_hand_data_lock, 
                                          dual_hand_state_array, dual_hand_action_array, simulation_mode=args.sim)
        elif args.ee == "dex1":
            from teleop.robot_control.robot_hand_unitree import Dex1_1_Gripper_Controller
            left_gripper_value = Value('d', 0.0, lock=True)        # [input]
            right_gripper_value = Value('d', 0.0, lock=True)       # [input]
            dual_gripper_data_lock = Lock()
            dual_gripper_state_array = Array('d', 2, lock=False)   # current left, right gripper state(2) data.
            dual_gripper_action_array = Array('d', 2, lock=False)  # current left, right gripper action(2) data.
            gripper_ctrl = Dex1_1_Gripper_Controller(left_gripper_value, right_gripper_value, dual_gripper_data_lock, 
                                                     dual_gripper_state_array, dual_gripper_action_array, simulation_mode=args.sim)
        elif args.ee == "inspire_dfx":
            from teleop.robot_control.robot_hand_inspire import Inspire_Controller_DFX
            left_hand_pos_array = Array('d', 75, lock = True)      # [input]
            right_hand_pos_array = Array('d', 75, lock = True)     # [input]
            dual_hand_data_lock = Lock()
            dual_hand_state_array = Array('d', 12, lock = False)   # [output] current left, right hand state(12) data.
            dual_hand_action_array = Array('d', 12, lock = False)  # [output] current left, right hand action(12) data.
            direct_hand_cmd_array = Array('d', 12, lock = True)    # [5.19新增]：扳机直接命令通道 'd' 表示 C 语言的双精度浮点数（double），12 个元素：6 个左手 + 6 个右手。 因为有主进程写 + 子进程读的并发访问，必须加锁保证原子性。
            direct_hand_cmd_array[:] = [-1.0]*12                        # [5.19新增]：初始化为 -1，表示无扳机命令
            # 初始化为全 -1，表示"没有直接命令"。子进程的 np.any(direct_cmd >= 0) 会判断为 False，走手势重定向。
            # 这个初始化的时机很重要：它发生在 Inspire_Controller_DFX(...) 构造之前，确保子进程从第一帧起就能读到 -1，被 wait DDS 的 while 循环阻塞住也不会出问题。
            # [5.19]新增 "direct_cmd_array=direct_hand_cmd_array"参数 ：把数组传给 __init__，再经 args 传给 control_process。
            hand_ctrl = Inspire_Controller_DFX(left_hand_pos_array, right_hand_pos_array, dual_hand_data_lock, dual_hand_state_array, dual_hand_action_array,direct_cmd_array=direct_hand_cmd_array,  simulation_mode=args.sim)
        elif args.ee == "inspire_ftp":
            from teleop.robot_control.robot_hand_inspire import Inspire_Controller_FTP
            left_hand_pos_array = Array('d', 75, lock = True)      # [input]
            right_hand_pos_array = Array('d', 75, lock = True)     # [input]
            dual_hand_data_lock = Lock()
            dual_hand_state_array = Array('d', 12, lock = False)   # [output] current left, right hand state(12) data.
            dual_hand_action_array = Array('d', 12, lock = False)  # [output] current left, right hand action(12) data.
            hand_ctrl = Inspire_Controller_FTP(left_hand_pos_array, right_hand_pos_array, dual_hand_data_lock, dual_hand_state_array, dual_hand_action_array, simulation_mode=args.sim)
        elif args.ee == "brainco":
            from teleop.robot_control.robot_hand_brainco import Brainco_Controller
            left_hand_pos_array = Array('d', 75, lock = True)      # [input]
            right_hand_pos_array = Array('d', 75, lock = True)     # [input]
            dual_hand_data_lock = Lock()
            dual_hand_state_array = Array('d', 12, lock = False)   # [output] current left, right hand state(12) data.
            dual_hand_action_array = Array('d', 12, lock = False)  # [output] current left, right hand action(12) data.
            hand_ctrl = Brainco_Controller(left_hand_pos_array, right_hand_pos_array, dual_hand_data_lock, 
                                           dual_hand_state_array, dual_hand_action_array, simulation_mode=args.sim)
        else:
            pass
        
        # affinity mode (if you dont know what it is, then you probably don't need it)
        if args.affinity:
            import psutil
            p = psutil.Process(os.getpid())
            p.cpu_affinity([0,1,2,3]) # Set CPU affinity to cores 0-3
            try:
                p.nice(-20)           # Set highest priority
                logger_mp.info("Set high priority successfully.")
            except psutil.AccessDenied:
                logger_mp.warning("Failed to set high priority. Please run as root.")
                
            for child in p.children(recursive=True):
                try:
                    logger_mp.info(f"Child process {child.pid} name: {child.name()}")
                    child.cpu_affinity([5,6])
                    child.nice(-20)
                except psutil.AccessDenied:
                    pass

        # simulation mode
        if args.sim:
            reset_pose_publisher = ChannelPublisher("rt/reset_pose/cmd", String_)
            reset_pose_publisher.Init()
            from teleop.utils.sim_state_topic import start_sim_state_subscribe
            sim_state_subscriber = start_sim_state_subscribe()

        # record + headless / non-headless mode
        if args.record:
            recorder = EpisodeWriter(task_dir = os.path.join(args.task_dir, args.task_name),
                                     task_goal = args.task_goal,
                                     task_desc = args.task_desc,
                                     task_steps = args.task_steps,
                                     frequency = args.frequency, 
                                     rerun_log = not args.headless)

        logger_mp.info("----------------------------------------------------------------")
        logger_mp.info("🟢  Press [r] to start syncing the robot with your movements.")
        if args.record:
            logger_mp.info("🟡  Press [s] to START or SAVE recording (toggle cycle).")
        else:
            logger_mp.info("🔵  Recording is DISABLED (run with --record to enable).")
        logger_mp.info("🔴  Press [q] to stop and exit the program.")
        logger_mp.info("⚠️  IMPORTANT: Please keep your distance and stay safe.")

        # [5.24新增] 外层循环：Y键暂停遥操作(START=False)后回到等待态，可再按Y恢复
        while not STOP:
            READY = True                  # now ready to (1) enter START state
            IK_OFFSET_CALIBRATED = False  # [5.23新增] 每次进入遥操作前重置偏移标记，恢复时重新标定不跳变 [5.24新增] 每次从等待进入遥操作都要重新标定偏移，不管是一开始的首次进入，还是暂停后恢复。
            while not START and not STOP: # 等待进入 START 状态，或者直接按退出键 STOP 跳出循环
                time.sleep(0.033)
                if camera_config['head_camera']['enable_zmq'] and xr_need_local_img:
                    head_img = img_client.get_head_frame()
                    tv_wrapper.render_to_xr(head_img.bgr)

                #【5.18新增】PICO Y键(左手B): 按一下进入遥操作
                # 为什么放等待循环里： 程序启动后先在这个循环里等，此时 START=False，Y 键按下就设 START=True 进入主循环。逻辑和键盘按 r 一样，只是数据来源从 on_press 换成了 tele_data.left_ctrl_bButton。
                tele_data = tv_wrapper.get_tele_data()
                Y_btn = tele_data.left_ctrl_bButton
                if Y_btn and not _last_Y_btn: #上升沿检测
                    START = True
                    #[5.19新增] 进入遥操时清除残留的扳机命令 (如果你先退出了遥操再重新进入，上一轮的扳机残留值可能还在数组里。进入时清掉保证每次遥操开始都是从手势重定向起步。)
                    if args.ee == "inspire_dfx":
                        direct_hand_cmd_array[:] = [-1.0]*12
                _last_Y_btn = Y_btn

                #【5.24新增】PICO X键(左手A): 退出程序
                X_btn = tele_data.left_ctrl_aButton
                if X_btn and not _last_X_btn:
                    STOP = True
                _last_X_btn = X_btn

            if STOP: #【5.24新增】如果在等待 START 的过程中按了退出键（键盘 q 或者 PICO X），就直接跳出外层循环，走 finally 进行清理退出，而不是进入主循环。
                break

            logger_mp.info("---------------------🚀start Tracking🚀-------------------------")
            arm_ctrl.speed_gradual_max()
            # main loop. robot start to follow VR user's motion
            while START and not STOP: #【5.24新增】在主循环里，如果按了暂停键 Y，START 变 False，立刻跳出主循环，回到外层的等待循环，等你再按一次 Y 恢复。
                start_time = time.time()
                # get image
                if camera_config['head_camera']['enable_zmq']:
                    if args.record or xr_need_local_img:
                        head_img = img_client.get_head_frame()
                    if xr_need_local_img:
                        tv_wrapper.render_to_xr(head_img.bgr)
                if camera_config['left_wrist_camera']['enable_zmq']:
                    if args.record:
                        left_wrist_img = img_client.get_left_wrist_frame()
                if camera_config['right_wrist_camera']['enable_zmq']:
                    if args.record:
                        right_wrist_img = img_client.get_right_wrist_frame()
    
                # record mode
                if args.record and RECORD_TOGGLE:
                    RECORD_TOGGLE = False
                    if not RECORD_RUNNING:
                        # [5.25新增] 达到最大 episode 数限制后不再创建新 episode，也不再进入 RECORD_RUNNING 状态，直到用户重启程序。这样可以防止误操作导致数据爆炸，同时保留已录制的数据。
                        if args.max_episodes > 0 and recorder.episode_id >= args.max_episodes:
                            logger_mp.info(f"Reached max episodes limit ({args.max_episodes}), stoprecording.")
                            # 不再创建新 episode，也可以直接 break 或只打印提示
                        else:
                            if recorder.create_episode():
                                RECORD_RUNNING = True
                            else:
                                logger_mp.error("Failed to create episode. Recording not started.")
                    else:
                        RECORD_RUNNING = False
                        recorder.save_episode()
                        if args.sim:
                            publish_reset_category(1, reset_pose_publisher)
    
                # get xr's tele data
                tele_data = tv_wrapper.get_tele_data()
    
                # [5.19新增] inspire_dfx + controller模式: 扳机直接控制爪子开合
                if args.ee == "inspire_dfx" and args.input_mode == "controller":# - 只在 inspire DFX 手 + controller 模式时生效。hand 模式时仍然走手势重定向（原有逻辑在 line 291 已经处理）。
                        # triggerValue: 10.0=完全松开 → 0.0=完全按下
                        # inspire手范围: 0=全闭 → 1000=全开
                        # 所以: 板机按越深 → 值越小 → 手越闭合，直觉符合
                        left_cmd  = int(np.clip(tele_data.left_ctrl_triggerValue * 100.0, 0, 1000)) #left_ctrl_triggerValue * 100.0：triggerValue 在 tv_wrapper.py 里被处理成 0.0 ~ 10.0（10.0 表示扳机完全松开，0.0 表示完全按下）。乘 100 映射到 0 ~ 1000，刚好是 inspire DFX 驱动协议的数值范围（0 = 全闭，1000 = 全开）。
                        right_cmd = int(np.clip(tele_data.right_ctrl_triggerValue * 100.0, 0, 1000))#np.clip(..., 0, 1000) 防御性编程。理论上 triggerValue 不会超出 0~10，但如果 XR 数据有抖动，clip 保证不会发出越界命令。
                        with direct_hand_cmd_array.get_lock(): # 锁住整个写入过程。子进程在同一时刻要么读到完整的旧值（全 -1 或上一次扳机值），要么读到完整的新值，不会读到半新半旧的脏数据。
                            direct_hand_cmd_array[:6] = [left_cmd] * 6 #6 个手指（pinky/ring/middle/index/thumb-bend/thumb-rotation）统一用同一个开合值。扳机是一个维度的控制，把它平均映射到所有手指。如果你以后想精细控制每个手指，可以改成不同的映射。
                            direct_hand_cmd_array[6:] = [right_cmd] * 6
    
                # ---- 【5.18新增】PICO Y键(左手B): 按一下退出遥操作 ----
                # Y_btn = tele_data.left_ctrl_bButton
                # if Y_btn and not _last_Y_btn: #上升沿检测 翻转 START 和 STOP 的值，达到按一下 Y 键就能进入/退出遥操作的效果
                #     if START: #当前在遥操中，那就退出（START=False, STOP=True），主循环 while not STOP 结束，程序走 finally 清理退出
                #         START = False
                #         STOP = True
                #         # [5.19新增] 重置扳机命令，让子进程回到手势重定向
                #         if args.ee == "inspire_dfx":
                #             direct_hand_cmd_array[:] = [-1.0]*12
                #     else:
                #         START = True
                # _last_Y_btn = Y_btn

                # ---- 【5.18新增】PICO B键(右手B): 按一下切换录制状态 ----
                B_btn = tele_data.right_ctrl_bButton
                if B_btn and not _last_B_btn and START: #上升沿检测，并且只能在遥操作状态下切换录制状态，避免误操作导致的录制混乱
                    RECORD_TOGGLE = True #完全照搬现有状态机。Line 276-287 的代码会处理剩下的：第一次触发→开始录制，第二次触发→保存并停止录制，第三次→又开始录制……和键盘按 s 一模一样
                _last_B_btn = B_btn

                # ---- 【5.25新增】PICO A键(右手A): 删除上一个已保存的episode ----
                A_btn = tele_data.right_ctrl_aButton
                if A_btn and not _last_A_btn and START and not RECORD_RUNNING:#上升沿检测，并且只能在遥操作状态下、非录制状态下触发删除，避免误操作导致的录制混乱或误删正在录制的数据
                    recorder.delete_current_episode()
                _last_A_btn = A_btn

                # ---- 【5.24修改】PICO Y键(左手B): 暂停/恢复遥操作 ----
                Y_btn = tele_data.left_ctrl_bButton
                if Y_btn and not _last_Y_btn:
                    if START:
                        START = False  # 暂停，不退出程序，回到外层的等待循环
                _last_Y_btn = Y_btn

                # ---- 【5.24新增】PICO X键(左手A): 退出程序 ----
                X_btn = tele_data.left_ctrl_aButton
                if X_btn and not _last_X_btn:
                    START = False
                    STOP = True
                    # [5.19] 重置扳机命令 这里是针对使用inspire_dfx手的用户：当他们按 X 键退出程序时，顺便把扳机命令数组清零，避免残留命令对下一次使用造成干扰。对于其他手型，这个数组可能根本没用，不清零也无所谓。
                    if args.ee == "inspire_dfx":
                        direct_hand_cmd_array[:] = [-1.0]*12
                _last_X_btn = X_btn
    
                if (args.ee == "dex3" or args.ee == "inspire_dfx" or args.ee == "inspire_ftp" or args.ee == "brainco") and args.input_mode == "hand":
                    with left_hand_pos_array.get_lock():
                        left_hand_pos_array[:] = tele_data.left_hand_pos.flatten()
                    with right_hand_pos_array.get_lock():
                        right_hand_pos_array[:] = tele_data.right_hand_pos.flatten()
                elif args.ee == "dex1" and args.input_mode == "controller":
                    with left_gripper_value.get_lock():
                        left_gripper_value.value = tele_data.left_ctrl_triggerValue
                    with right_gripper_value.get_lock():
                        right_gripper_value.value = tele_data.right_ctrl_triggerValue
                elif args.ee == "dex1" and args.input_mode == "hand":
                    with left_gripper_value.get_lock():
                        left_gripper_value.value = tele_data.left_hand_pinchValue
                    with right_gripper_value.get_lock():
                        right_gripper_value.value = tele_data.right_hand_pinchValue
                else:
                    pass
                
                # high level control
                if args.input_mode == "controller" and args.motion:
                    # quit teleoperate
                    if tele_data.right_ctrl_aButton:
                        START = False
                        STOP = True
                    # command robot to enter damping mode. soft emergency stop function
                    if tele_data.left_ctrl_thumbstick and tele_data.right_ctrl_thumbstick:
                        loco_wrapper.Damp()
                    # https://github.com/unitreerobotics/xr_teleoperate/issues/135, control, limit velocity to within 0.3
                    loco_wrapper.Move(-tele_data.left_ctrl_thumbstickValue[1] * 0.3,
                                      -tele_data.left_ctrl_thumbstickValue[0] * 0.3,
                                      -tele_data.right_ctrl_thumbstickValue[0]* 0.3)
    
                # get current robot state data.
                current_lr_arm_q  = arm_ctrl.get_current_dual_arm_q()
                current_lr_arm_dq = arm_ctrl.get_current_dual_arm_dq()
    
                # solve ik using motor data and wrist pose, then use ik results to control arms.
                time_ik_start = time.time()
                sol_q, sol_tauff  = arm_ik.solve_ik(tele_data.left_wrist_pose, tele_data.right_wrist_pose, current_lr_arm_q, current_lr_arm_dq)
                
                # [5.23新增] IK 偏移量标定：初始时记录一次 IK 解算结果与当前机械臂状态的差值作为偏移量，后续每次解算结果都减去这个偏移量。这样可以消除初始时机械臂末端位置与 XR 手部位置的突变。
                if not IK_OFFSET_CALIBRATED:
                    ik_offset = sol_q - current_lr_arm_q
                    IK_OFFSET_CALIBRATED = True
                    logger_mp.info(f"IK 偏移量标定: {ik_offset}")
                sol_q = sol_q - ik_offset
                #  第一帧（你手柄没动）：
                # ik_offset = sol_q - current_lr_arm_q   # 记录差异
                # sol_q = sol_q - ik_offset = current_lr_arm_q   # 机械臂不动 ✓
    
                # 第二帧（你手往上抬了 5cm）：
                # 新的 sol_q = IK(新的手腕位姿)          # 比如原来0.5，现在变成0.7
                # sol_q = 0.7 - ik_offset               # 减去的是第一帧算出来的那个固定偏移
                #         = 0.7 - (0.5 - 0)               # 假设current_lr_arm_q≈0
                #         = 0.7 - 0.5 = 0.2               # 机械臂只跟着相对运动走了0.2
    
                time_ik_end = time.time()
                logger_mp.debug(f"ik:\t{round(time_ik_end - time_ik_start, 6)}")
                arm_ctrl.ctrl_dual_arm(sol_q, sol_tauff)
    
                # record data
                if args.record:
                    READY = recorder.is_ready() # now ready to (2) enter RECORD_RUNNING state
                    # dex hand or gripper
                    if args.ee == "dex3" and args.input_mode == "hand":
                        with dual_hand_data_lock:
                            left_ee_state = dual_hand_state_array[:7]
                            right_ee_state = dual_hand_state_array[-7:]
                            left_hand_action = dual_hand_action_array[:7]
                            right_hand_action = dual_hand_action_array[-7:]
                            current_body_state = []
                            current_body_action = []
                    elif args.ee == "dex1" and args.input_mode == "hand":
                        with dual_gripper_data_lock:
                            left_ee_state = [dual_gripper_state_array[0]]
                            right_ee_state = [dual_gripper_state_array[1]]
                            left_hand_action = [dual_gripper_action_array[0]]
                            right_hand_action = [dual_gripper_action_array[1]]
                            current_body_state = []
                            current_body_action = []
                    elif args.ee == "dex1" and args.input_mode == "controller":
                        with dual_gripper_data_lock:
                            left_ee_state = [dual_gripper_state_array[0]]
                            right_ee_state = [dual_gripper_state_array[1]]
                            left_hand_action = [dual_gripper_action_array[0]]
                            right_hand_action = [dual_gripper_action_array[1]]
                            current_body_state = arm_ctrl.get_current_motor_q().tolist()
                            current_body_action = [-tele_data.left_ctrl_thumbstickValue[1]  * 0.3,
                                                   -tele_data.left_ctrl_thumbstickValue[0]  * 0.3,
                                                   -tele_data.right_ctrl_thumbstickValue[0] * 0.3]
                    elif (args.ee == "inspire_dfx" or args.ee == "inspire_ftp" or args.ee == "brainco") and args.input_mode == "hand":
                        with dual_hand_data_lock:
                            left_ee_state = dual_hand_state_array[:6]
                            right_ee_state = dual_hand_state_array[-6:]
                            left_hand_action = dual_hand_action_array[:6]
                            right_hand_action = dual_hand_action_array[-6:]
                            current_body_state = []
                            current_body_action = []
                    else:
                        left_ee_state = []
                        right_ee_state = []
                        left_hand_action = []
                        right_hand_action = []
                        current_body_state = []
                        current_body_action = []
    
                    # arm state and action
                    left_arm_state  = current_lr_arm_q[:7]
                    right_arm_state = current_lr_arm_q[-7:]
                    left_arm_action = sol_q[:7]
                    right_arm_action = sol_q[-7:]
                    if RECORD_RUNNING:
                        colors = {}
                        depths = {}
                        if camera_config['head_camera']['binocular']:
                            if head_img is not None:
                                colors[f"color_{0}"] = head_img.bgr[:, :camera_config['head_camera']['image_shape'][1]//2]
                                colors[f"color_{1}"] = head_img.bgr[:, camera_config['head_camera']['image_shape'][1]//2:]
                            else:
                                logger_mp.warning("Head image is None!")
                            if camera_config['left_wrist_camera']['enable_zmq']:
                                if left_wrist_img is not None:
                                    colors[f"color_{2}"] = left_wrist_img.bgr
                                else:
                                    logger_mp.warning("Left wrist image is None!")
                            if camera_config['right_wrist_camera']['enable_zmq']:
                                if right_wrist_img is not None:
                                    colors[f"color_{3}"] = right_wrist_img.bgr
                                else:
                                    logger_mp.warning("Right wrist image is None!")
                        else:
                            if head_img is not None:
                                colors[f"color_{0}"] = head_img.bgr
                            else:
                                logger_mp.warning("Head image is None!")
                            if camera_config['left_wrist_camera']['enable_zmq']:
                                if left_wrist_img is not None:
                                    colors[f"color_{1}"] = left_wrist_img.bgr
                                else:
                                    logger_mp.warning("Left wrist image is None!")
                            if camera_config['right_wrist_camera']['enable_zmq']:
                                if right_wrist_img is not None:
                                    colors[f"color_{2}"] = right_wrist_img.bgr
                                else:
                                    logger_mp.warning("Right wrist image is None!")
                        states = {
                            "left_arm": {                                                                    
                                "qpos":   left_arm_state.tolist(),    # numpy.array -> list
                                "qvel":   [],                          
                                "torque": [],                        
                            }, 
                            "right_arm": {                                                                    
                                "qpos":   right_arm_state.tolist(),       
                                "qvel":   [],                          
                                "torque": [],                         
                            },                        
                            "left_ee": {                                                                    
                                "qpos":   left_ee_state,           
                                "qvel":   [],                           
                                "torque": [],                          
                            }, 
                            "right_ee": {                                                                    
                                "qpos":   right_ee_state,       
                                "qvel":   [],                           
                                "torque": [],  
                            }, 
                            "body": {
                                "qpos": current_body_state,
                            }, 
                        }
                        actions = {
                            "left_arm": {                                   
                                "qpos":   left_arm_action.tolist(),       
                                "qvel":   [],       
                                "torque": [],      
                            }, 
                            "right_arm": {                                   
                                "qpos":   right_arm_action.tolist(),       
                                "qvel":   [],       
                                "torque": [],       
                            },                         
                            "left_ee": {                                   
                                "qpos":   left_hand_action,       
                                "qvel":   [],       
                                "torque": [],       
                            }, 
                            "right_ee": {                                   
                                "qpos":   right_hand_action,       
                                "qvel":   [],       
                                "torque": [], 
                            }, 
                            "body": {
                                "qpos": current_body_action,
                            }, 
                        }
                        if args.sim:
                            sim_state = sim_state_subscriber.read_data()            
                            recorder.add_item(colors=colors, depths=depths, states=states, actions=actions, sim_state=sim_state)
                        else:
                            recorder.add_item(colors=colors, depths=depths, states=states, actions=actions)
    
                current_time = time.time()
                time_elapsed = current_time - start_time
                sleep_time = max(0, (1 / args.frequency) - time_elapsed)
                time.sleep(sleep_time)
                logger_mp.debug(f"main process sleep: {sleep_time}")

    except KeyboardInterrupt:
        logger_mp.info("⛔ KeyboardInterrupt, exiting program...")
    except Exception:
        import traceback
        logger_mp.error(traceback.format_exc())
    finally:
        try:
            arm_ctrl.ctrl_dual_arm_go_home()
        except Exception as e:
            logger_mp.error(f"Failed to ctrl_dual_arm_go_home: {e}")
        
        try:
            if args.ipc:
                ipc_server.stop()
            else:
                stop_listening()
                listen_keyboard_thread.join()
        except Exception as e:
            logger_mp.error(f"Failed to stop keyboard listener or ipc server: {e}")
        
        try:
            img_client.close()
        except Exception as e:
            logger_mp.error(f"Failed to close image client: {e}")

        try:
            tv_wrapper.close()
        except Exception as e:
            logger_mp.error(f"Failed to close televuer wrapper: {e}")

        try:
            if not args.motion:
                pass
                # status, result = motion_switcher.Exit_Debug_Mode()
                # logger_mp.info(f"Exit debug mode: {'Success' if status == 3104 else 'Failed'}")
        except Exception as e:
            logger_mp.error(f"Failed to exit debug mode: {e}")

        try:
            if args.sim:
                sim_state_subscriber.stop_subscribe()
        except Exception as e:
            logger_mp.error(f"Failed to stop sim state subscriber: {e}")
        
        try:
            if args.record:
                recorder.close()
        except Exception as e:
            logger_mp.error(f"Failed to close recorder: {e}")
        logger_mp.info("✅ Finally, exiting program.")
        exit(0)

