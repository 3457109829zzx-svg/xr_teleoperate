"""
Dex1-1 二指夹爪控制器

基于宇树官方 dex1_1_gripper_server（C++ 服务）封装，通过 DDS 通信控制夹爪开合。
运行在开发 PC 上，与机器人 PC2 上的 dex1_1_gripper_server 配合使用。

使用示例（作为模块导入）:
    controller = Dex1_1_Controller(network_interface="eth0")
    controller.set_gripper_targets(2.0, 2.0)  # 左右夹爪都开到 2.0 rad
    left_q, right_q = controller.get_current_dual_gripper_q()
    controller.close()

使用示例（在 g1_executor.py 中使用）:
    from robot_control.dex1_1_controller import Dex1_1_Controller

    # 替换原来的 InspireController，接口完全兼容
    self.gripper_controller = Dex1_1_Controller(skip_dds_init=True)

    # 获取状态 — 与原 InspireController 完全一致的 API
    left_q, right_q = self.gripper_controller.get_current_dual_hand_q()
    # left_q.shape → (1,), right_q.shape → (1,)

    # 执行动作 — ctrl_dual_hand 接受 ndarray，提取标量
    self.gripper_controller.ctrl_dual_hand(left_action[i], right_action[i])

    # 回初始位
    self.gripper_controller.ctrl_label("open")

    完整集成见文件末尾的 g1_executor 集成示例。

使用示例（键盘控制测试）:
    python dex1_1_controller.py --network-interface eth0

架构:
    dex1_1_controller.py (Python, 开发PC)
        │  DDS (以太网)
        ▼
    dex1_1_gripper_server (C++, 机器人PC2)
        │  串口 (ttyUSB)
        ▼
    M4010 电机 ×2 (左手 ID=1, 右手 ID=0)

DDS 话题:
    - 控制指令: rt/dex1/left/cmd, rt/dex1/right/cmd  (MotorCmds_, 各 1 个 MotorCmd)
    - 状态反馈: rt/dex1/left/state, rt/dex1/right/state (MotorStates_, 各 1 个 MotorState)

参考:
    - 官方 dex1_1_service: https://github.com/unitreerobotics/dex1_1_service
    - inspire_controller.py (因时灵巧手控制器, 同风格)
"""

import numpy as np
import threading
import time
from typing import Tuple, Dict, Any
from multiprocessing import Value, Array, Lock

# 宇树 SDK 导入
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.unitree_go.msg.dds_ import MotorCmds_, MotorStates_
from unitree_sdk2py.idl.default import unitree_go_msg_dds__MotorCmd_

# ---------------------------------------------------------
# 常量定义
# ---------------------------------------------------------
# DDS 话题
TOPIC_DEX1_LEFT_CMD = "rt/dex1/left/cmd"
TOPIC_DEX1_RIGHT_CMD = "rt/dex1/right/cmd"
TOPIC_DEX1_LEFT_STATE = "rt/dex1/left/state"
TOPIC_DEX1_RIGHT_STATE = "rt/dex1/right/state"

# 电机参数 (M4010)
# 电机转 5.4 rad → 滑块行程 9 cm → 约 0.6 rad/cm
MAPPED_MIN = 0.0        # 全闭（夹爪闭合时电机角度）
MAPPED_MAX = 5.40       # 全开（夹爪张开到最大时电机角度，对应滑块行程约 9 cm）

# PID 控制参数（来自实际验证使用的 robot_hand_unitree.py）
KP = 5.0
KD = 0.05


# ---------------------------------------------------------
# Dex1-1 夹爪控制器
# ---------------------------------------------------------
class Dex1_1_Controller:
    """
    Dex1-1 二指夹爪控制器

    功能：
    - 通过 DDS 与机器人端 dex1_1_gripper_server 通信
    - 后台线程持续订阅夹爪状态
    - 后台线程执行控制循环，发送目标角度
    - 提供简洁的 set/get 接口

    使用示例：
        controller = Dex1_1_Controller(network_interface="eth0")
        controller.set_gripper_targets(3.0, 3.0)  # 双手半开
        left_q, right_q = controller.get_current_dual_gripper_q()
        controller.close()
    """

    def __init__(
        self,
        fps: float = 100.0,
        simulation_mode: bool = False,
        skip_dds_init: bool = False,
        network_interface: str = None,
    ):
        """
        初始化 Dex1-1 夹爪控制器

        Args:
            fps: 控制频率 (Hz)，默认 100Hz
            simulation_mode: 仿真模式（DDS domain 1），默认 False（实机 domain 0）
            skip_dds_init: 跳过 DDS 初始化（外部已调用 ChannelFactoryInitialize 时设为 True）
            network_interface: DDS 网卡名，如 "eth0"。None 则使用默认接口。
        """
        print("[Dex1_1_Controller] 初始化中...")

        self.fps = fps
        self.simulation_mode = simulation_mode
        self.running = True

        # ---------------------------------------------------------
        # 初始化 DDS 通信（可选跳过）
        # ---------------------------------------------------------
        if not skip_dds_init:
            if self.simulation_mode:
                ChannelFactoryInitialize(1, networkInterface=network_interface)
            else:
                ChannelFactoryInitialize(0, networkInterface=network_interface)

        # ---------------------------------------------------------
        # 创建 DDS 发布者（发送控制指令）—— 左右手各一个
        # ---------------------------------------------------------
        self._left_cmd_publisher = ChannelPublisher(TOPIC_DEX1_LEFT_CMD, MotorCmds_)
        self._left_cmd_publisher.Init()
        self._right_cmd_publisher = ChannelPublisher(TOPIC_DEX1_RIGHT_CMD, MotorCmds_)
        self._right_cmd_publisher.Init()

        # ---------------------------------------------------------
        # 创建 DDS 订阅者（接收状态反馈）—— 左右手各一个
        # ---------------------------------------------------------
        self._left_state_subscriber = ChannelSubscriber(TOPIC_DEX1_LEFT_STATE, MotorStates_)
        self._left_state_subscriber.Init()
        self._right_state_subscriber = ChannelSubscriber(TOPIC_DEX1_RIGHT_STATE, MotorStates_)
        self._right_state_subscriber.Init()

        # ---------------------------------------------------------
        # 创建共享内存（线程间数据传递）
        # ---------------------------------------------------------
        # 目标位置（用户写入，控制线程读取）
        self._left_target_value = Value("d", 0.0, lock=True)
        self._right_target_value = Value("d", 0.0, lock=True)

        # 当前状态（订阅线程写入，用户读取）
        self._left_state_value = Value("d", 0.0, lock=True)
        self._right_state_value = Value("d", 0.0, lock=True)

        # 合并的状态和动作（对外暴露的完整数组）
        self._dual_gripper_lock = Lock()
        self._dual_gripper_state_array = Array("d", 2, lock=False)   # [left_q, right_q]
        self._dual_gripper_action_array = Array("d", 2, lock=False)  # [left_target, right_target]

        # ---------------------------------------------------------
        # 初始化目标位置为半开状态（避免启动时突变）
        # ---------------------------------------------------------
        mid_position = (MAPPED_MAX - MAPPED_MIN) / 2.0  # 2.70 rad
        with self._left_target_value.get_lock():
            self._left_target_value.value = mid_position
        with self._right_target_value.get_lock():
            self._right_target_value.value = mid_position

        # ---------------------------------------------------------
        # 启动订阅线程（后台持续读取夹爪状态）
        # ---------------------------------------------------------
        self._subscribe_thread = threading.Thread(target=self._subscribe_state)
        self._subscribe_thread.daemon = True
        self._subscribe_thread.start()

        # ---------------------------------------------------------
        # 启动控制线程（后台持续发送控制指令）
        # ---------------------------------------------------------
        self._control_thread = threading.Thread(target=self._control_loop)
        self._control_thread.daemon = True
        self._control_thread.start()

        # 等待 DDS 连接
        self._wait_for_connection()

        print(f"[Dex1_1_Controller] 初始化完成! 控制频率: {fps} Hz")

    # ---------------------------------------------------------
    # 等待 DDS 连接
    # ---------------------------------------------------------
    def _wait_for_connection(self, timeout: float = 10.0):
        """等待 DDS 订阅收到有效数据"""
        start_time = time.time()
        while True:
            left_q, right_q = self.get_current_dual_gripper_q()
            # 注：刚启动时夹爪可能在任意位置（包括 0.0），
            # 只要订阅线程运行了一段时间就认为已连接
            if self._subscribe_ready:
                print(f"[Dex1_1_Controller] DDS 连接成功! 当前角度: L={left_q:.3f} rad, R={right_q:.3f} rad")
                break

            if time.time() - start_time > timeout:
                print(f"[Dex1_1_Controller] 警告: DDS 连接超时 (>{timeout}s)，继续运行...")
                break

            time.sleep(0.01)
            print("[Dex1_1_Controller] 等待 DDS 订阅...")

    # ---------------------------------------------------------
    # 订阅线程：持续读取夹爪状态
    # ---------------------------------------------------------
    def _subscribe_state(self):
        """后台线程：从 DDS 持续读取左右夹爪的电机角度"""
        self._subscribe_ready = False
        while self.running:
            try:
                left_msg = self._left_state_subscriber.Read()
                right_msg = self._right_state_subscriber.Read()

                if left_msg is not None and right_msg is not None:
                    with self._left_state_value.get_lock():
                        self._left_state_value.value = left_msg.states[0].q
                    with self._right_state_value.get_lock():
                        self._right_state_value.value = right_msg.states[0].q
                    self._subscribe_ready = True

                time.sleep(0.002)  # 500 Hz 读取频率

            except Exception as e:
                print(f"[Dex1_1_Controller] 订阅线程错误: {e}")
                time.sleep(0.1)

    # ---------------------------------------------------------
    # 控制线程：持续发送控制指令
    # ---------------------------------------------------------
    def _control_loop(self):
        """后台线程：读取目标角度，通过 DDS 发送给夹爪电机"""
        # 初始化指令消息 — 左右手各一个，每个只有 1 个电机
        left_cmd = MotorCmds_()
        left_cmd.cmds = [unitree_go_msg_dds__MotorCmd_()]
        right_cmd = MotorCmds_()
        right_cmd.cmds = [unitree_go_msg_dds__MotorCmd_()]

        # 设置 PID 参数（一次设定，不再改变）
        left_cmd.cmds[0].dq = 0.0
        left_cmd.cmds[0].tau = 0.0
        left_cmd.cmds[0].kp = KP
        left_cmd.cmds[0].kd = KD

        right_cmd.cmds[0].dq = 0.0
        right_cmd.cmds[0].tau = 0.0
        right_cmd.cmds[0].kp = KP
        right_cmd.cmds[0].kd = KD

        dt = 1.0 / self.fps

        print(f"[Dex1_1_Controller] 控制线程启动, 频率: {self.fps} Hz")

        try:
            while self.running:
                start_time = time.time()

                # 1. 读取目标角度
                with self._left_target_value.get_lock():
                    left_target = self._left_target_value.value
                with self._right_target_value.get_lock():
                    right_target = self._right_target_value.value

                # 2. 读取当前状态
                with self._left_state_value.get_lock():
                    left_current = self._left_state_value.value
                with self._right_state_value.get_lock():
                    right_current = self._right_state_value.value

                # 3. 发送指令
                left_cmd.cmds[0].q = float(left_target)
                right_cmd.cmds[0].q = float(right_target)

                self._left_cmd_publisher.Write(left_cmd)
                self._right_cmd_publisher.Write(right_cmd)

                # 4. 更新合并的状态和动作数组
                with self._dual_gripper_lock:
                    self._dual_gripper_state_array[0] = left_current
                    self._dual_gripper_state_array[1] = right_current
                    self._dual_gripper_action_array[0] = left_target
                    self._dual_gripper_action_array[1] = right_target

                # 5. 频率控制
                elapsed = time.time() - start_time
                sleep_time = max(0, dt - elapsed)
                if sleep_time > 0:
                    time.sleep(sleep_time)

        except Exception as e:
            print(f"[Dex1_1_Controller] 控制线程错误: {e}")
        finally:
            print("[Dex1_1_Controller] 控制线程已停止")

    # ---------------------------------------------------------
    # 公开接口：设置双手目标角度
    # ---------------------------------------------------------
    def set_gripper_targets(self, left_q: float, right_q: float):
        """
        设置双手夹爪目标角度

        Args:
            left_q: 左手夹爪目标角度 (rad)，范围 0.0（全闭）~ 5.40（全开）
            right_q: 右手夹爪目标角度 (rad)，范围 0.0（全闭）~ 5.40（全开）

        Note:
            - 换算关系：约 0.6 rad/cm，即滑块每移动 1 cm，电机转约 0.6 rad
            - 建议不要设为 0.0（完全闭合可能夹坏物体），最小留 0.3~0.5
        """
        left_q = float(left_q)
        right_q = float(right_q)

        with self._left_target_value.get_lock():
            self._left_target_value.value = left_q
        with self._right_target_value.get_lock():
            self._right_target_value.value = right_q

    # ---------------------------------------------------------
    # 公开接口：设置双手目标角度为同一角度
    # ---------------------------------------------------------
    def set_dual_gripper_target(self, q: float):
        """
        设置双手夹爪为同一目标角度

        Args:
            q: 目标角度 (rad)
        """
        self.set_gripper_targets(q, q)

    # ---------------------------------------------------------
    # 公开接口：获取双手当前角度
    # ---------------------------------------------------------
    def get_current_dual_gripper_q(self) -> Tuple[float, float]:
        """
        获取双手夹爪当前角度

        Returns:
            Tuple[float, float]: (左手角度, 右手角度)，单位 rad
        """
        with self._left_state_value.get_lock():
            left_q = self._left_state_value.value
        with self._right_state_value.get_lock():
            right_q = self._right_state_value.value

        return left_q, right_q

    # ---------------------------------------------------------
    # 公开接口：获取夹爪完整状态
    # ---------------------------------------------------------
    def get_gripper_state(self) -> Dict[str, Any]:
        """
        获取夹爪完整状态

        Returns:
            Dict: {
                "left_q": float,        # 左手当前角度 (rad)
                "right_q": float,       # 右手当前角度 (rad)
                "left_target": float,   # 左手目标角度 (rad)
                "right_target": float,  # 右手目标角度 (rad)
            }
        """
        left_q, right_q = self.get_current_dual_gripper_q()

        with self._dual_gripper_lock:
            left_target = self._dual_gripper_action_array[0]
            right_target = self._dual_gripper_action_array[1]

        return {
            "left_q": left_q,
            "right_q": right_q,
            "left_target": left_target,
            "right_target": right_target,
        }

    # ---------------------------------------------------------
    # 公开接口：夹爪回到初始位置
    # ---------------------------------------------------------
    def ctrl_dual_gripper_go_home(self, home_position: float = None):
        """
        双手夹爪回到张开位置

        Args:
            home_position: 初始位置角度 (rad)，默认 MAPPED_MAX（全开）
        """
        if home_position is None:
            home_position = MAPPED_MAX

        print(f"[Dex1_1_Controller] 夹爪回到初始位置 ({home_position:.2f} rad)...")
        self.set_dual_gripper_target(home_position)

    # ---------------------------------------------------------
    # 公开接口：闭合夹爪
    # ---------------------------------------------------------
    def close_gripper(self, gap: float = 0.5):
        """
        闭合夹爪到指定最小角度

        Args:
            gap: 闭合角度 (rad)，默认 0.5（几乎闭合但留有余量避免夹坏）
        """
        print(f"[Dex1_1_Controller] 闭合夹爪到 {gap:.2f} rad...")
        self.set_dual_gripper_target(gap)

    # ---------------------------------------------------------
    # ↓↓↓ g1_executor.py 兼容接口 — 与 InspireController API 对齐 ↓↓↓
    # ---------------------------------------------------------

    def set_hand_targets(self, left_q, right_q):
        """
        [g1_executor 兼容] 设置双手目标角度，接受 ndarray 或标量

        Args:
            left_q: 左手目标角度，ndarray (n,) 或标量。取第一个元素。
            right_q: 右手目标角度，ndarray (n,) 或标量。取第一个元素。

        Note:
            dex1-1 只有 1 个电机/手，但 g1_executor 传的是 (action_horizon, 6) 形状。
            这里统一取 flat[0] 做为标量目标。
        """
        left_val = float(np.array(left_q).flatten()[0])
        right_val = float(np.array(right_q).flatten()[0])
        self.set_gripper_targets(left_val, right_val)

    def ctrl_dual_hand(self, left_q, right_q):
        """
        [g1_executor 兼容] ctrl_dual_hand — set_hand_targets 的别名
        g1_executor._execute_action() 实际调用此方法
        """
        self.set_hand_targets(left_q, right_q)

    def get_current_dual_hand_q(self) -> Tuple[np.ndarray, np.ndarray]:
        """
        [g1_executor 兼容] 获取双手当前关节角度

        Returns:
            Tuple[np.ndarray, np.ndarray]: (左手角度, 右手角度)
            每个 ndarray shape 为 (1,)，因为 dex1-1 每手只有 1 个电机。

        Note:
            InspireController 返回 shape (6,)，这里返回 (1,) 以匹配 dex1-1 的 1 个电机。
            g1_executor._get_robot_state() 中会调用 .reshape(1, -1)，
            兼容 state.left_hand / state.right_hand 的格式。
        """
        left_q, right_q = self.get_current_dual_gripper_q()
        return np.array([left_q]), np.array([right_q])

    def get_hand_state(self) -> Dict[str, np.ndarray]:
        """
        [g1_executor 兼容] 获取手部完整状态

        Returns:
            Dict: {
                "left_hand_q": np.ndarray (1,),
                "right_hand_q": np.ndarray (1,),
            }
        """
        left_q, right_q = self.get_current_dual_hand_q()
        return {
            "left_hand_q": left_q,
            "right_hand_q": right_q,
        }

    def ctrl_label(self, label: str):
        """
        [g1_executor 兼容] 字符串命令控制

        Args:
            label: 命令字符串
                - "open" / "home" → 全开 (MAPPED_MAX = 5.40 rad)
                - "close" → 闭合 (留 0.5 rad 余量)
                - 其他 → 全开（默认安全行为）
        """
        label_lower = label.lower()
        if label_lower in ("open", "home"):
            self.ctrl_dual_gripper_go_home()
        elif label_lower == "close":
            self.close_gripper()
        else:
            print(f"[Dex1_1_Controller] 未知命令 '{label}'，执行默认安全动作（全开）")
            self.ctrl_dual_gripper_go_home()

    # ---------------------------------------------------------
    # ↑↑↑ g1_executor.py 兼容接口 END ↑↑↑
    # ---------------------------------------------------------

    # ---------------------------------------------------------
    # 公开接口：关闭控制器
    # ---------------------------------------------------------
    def close(self):
        """关闭控制器，释放资源"""
        print("[Dex1_1_Controller] 正在关闭...")

        self.running = False

        # 等待线程结束
        if self._subscribe_thread.is_alive():
            self._subscribe_thread.join(timeout=1.0)
        if self._control_thread.is_alive():
            self._control_thread.join(timeout=1.0)

        print("[Dex1_1_Controller] 已关闭")

    # ---------------------------------------------------------
    # 上下文管理器支持
    # ---------------------------------------------------------
    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()


# ---------------------------------------------------------
# 单元测试 / 键盘控制示例
# ---------------------------------------------------------
if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Dex1-1 夹爪控制器 — 键盘控制测试")
    parser.add_argument("-n", "--network-interface", type=str, default=None,
                        help="DDS 网卡名，如 eth0、eth1。不指定则使用默认接口。")
    parser.add_argument("--sim", action="store_true",
                        help="仿真模式（DDS domain 1）")
    parser.add_argument("--fps", type=float, default=100.0,
                        help="控制频率 (Hz)，默认 100")
    args = parser.parse_args()

    print("=" * 60)
    print("Dex1-1 二指夹爪控制器 — 键盘控制测试")
    print("=" * 60)
    print(f"  网卡: {args.network_interface or '默认'}")
    print(f"  模式: {'仿真' if args.sim else '实机'}")
    print(f"  频率: {args.fps} Hz")
    print()
    print("  操作说明:")
    print("    A / D     — 左夹爪 张开/闭合")
    print("    ← / →    — 右夹爪 张开/闭合")
    print("    W / S     — 双手同时 张开/闭合")
    print("    H         — 双手回到全开位置 (Home)")
    print("    Q         — 退出")
    print(f"  角度范围: {MAPPED_MIN:.1f} (全闭) ~ {MAPPED_MAX:.1f} (全开) rad")
    print("=" * 60)

    # 使用上下文管理器确保资源释放
    with Dex1_1_Controller(
        fps=args.fps,
        simulation_mode=args.sim,
        network_interface=args.network_interface,
    ) as controller:

        # 全局控制变量
        left_val = (MAPPED_MAX - MAPPED_MIN) / 2.0   # 初始半开
        right_val = (MAPPED_MAX - MAPPED_MIN) / 2.0
        step = 0.3  # 每次按键调整步长 (rad)，约 0.5 cm 滑块行程

        running = True

        # 导入键盘监听
        try:
            from sshkeyboard import listen_keyboard, stop_listening
        except ImportError:
            print("\n⚠️  未安装 sshkeyboard，使用 input() 交互模式")
            print("   输入格式: <left_q> <right_q>  (如 '3.0 3.0')")
            print("   输入 'h' 回 Home, 'q' 退出\n")

            # 简单交互模式
            while running:
                try:
                    cmd = input("> ").strip()
                    if cmd.lower() == 'q':
                        running = False
                    elif cmd.lower() == 'h':
                        controller.ctrl_dual_gripper_go_home()
                        time.sleep(0.5)
                    elif cmd:
                        parts = cmd.split()
                        if len(parts) == 1:
                            q = float(parts[0])
                            controller.set_dual_gripper_target(q)
                        elif len(parts) >= 2:
                            controller.set_gripper_targets(float(parts[0]), float(parts[1]))

                    state = controller.get_gripper_state()
                    print(f"  状态: L={state['left_q']:.3f}  R={state['right_q']:.3f}"
                          f"  |  目标: L={state['left_target']:.3f}  R={state['right_target']:.3f}")
                except KeyboardInterrupt:
                    break
                except ValueError:
                    print("  输入格式错误，请重新输入")
        else:
            # sshkeyboard 键盘监听模式
            def on_press(key):
                global left_val, right_val, running

                if key == 'a':
                    left_val = min(MAPPED_MAX, left_val + step)
                    print(f"\r  左夹爪张开 → {left_val:.2f} rad", end="")
                elif key == 'd':
                    left_val = max(MAPPED_MIN, left_val - step)
                    print(f"\r  左夹爪闭合 → {left_val:.2f} rad", end="")
                elif key == 'left':
                    right_val = min(MAPPED_MAX, right_val + step)
                    print(f"\r  右夹爪张开 → {right_val:.2f} rad", end="")
                elif key == 'right':
                    right_val = max(MAPPED_MIN, right_val - step)
                    print(f"\r  右夹爪闭合 → {right_val:.2f} rad", end="")
                elif key == 'w':
                    left_val = min(MAPPED_MAX, left_val + step)
                    right_val = min(MAPPED_MAX, right_val + step)
                    print(f"\r  双手张开 → L={left_val:.2f} R={right_val:.2f} rad", end="")
                elif key == 's':
                    left_val = max(MAPPED_MIN, left_val - step)
                    right_val = max(MAPPED_MIN, right_val - step)
                    print(f"\r  双手闭合 → L={left_val:.2f} R={right_val:.2f} rad", end="")
                elif key == 'h':
                    left_val = MAPPED_MAX
                    right_val = MAPPED_MAX
                    print(f"\r  回 Home → L={left_val:.2f} R={right_val:.2f} rad", end="")
                elif key == 'q':
                    running = False
                    stop_listening()

                # 更新目标值
                controller.set_gripper_targets(left_val, right_val)

            # 启动键盘监听
            from sshkeyboard import listen_keyboard
            listen_keyboard_thread = threading.Thread(
                target=listen_keyboard,
                kwargs={"on_press": on_press, "sequential": False},
                daemon=True,
            )
            listen_keyboard_thread.start()

            # 主循环：持续显示状态
            try:
                while running:
                    state = controller.get_gripper_state()
                    print(f"\r  状态[L={state['left_q']:.3f} R={state['right_q']:.3f}]"
                          f"  目标[L={state['left_target']:.3f} R={state['right_target']:.3f}]",
                          end="", flush=True)
                    time.sleep(0.05)
            except KeyboardInterrupt:
                pass
            finally:
                print("\n")

    print("控制器已安全关闭")
