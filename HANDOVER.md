# XR Teleoperate 交接文档 — 新用户 PC 配置指南

> **适用场景**：机器人端（PC2）已配置完毕，新用户仅需在自己的笔记本上部署 xr_teleoperate 项目。
>
> **前提条件**：
> - 你自己的笔记本电脑（Ubuntu 20.04 / 22.04）
> - PICO 4 Ultra Enterprise 头显（或 Meta Quest 3 / Apple Vision Pro）
> - 一根网线（连接笔记本和机器人）
> - 机器人 G1 已开机，PC2 上的灵巧手服务和图像服务已由管理员启动

---

## 一、一次性环境配置

### 1.1 安装 Miniconda（如果还没有）

```bash
# 下载安装
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh
bash Miniconda3-latest-Linux-x86_64.sh
# 重启终端或 source ~/.bashrc
```

### 1.2 创建 conda 环境

```bash
conda create -n tv python=3.10 pinocchio=3.1.0 numpy=1.26.4 -c conda-forge
conda activate tv
```

### 1.3 克隆项目并安装依赖

> **说明**：下面的地址是 ZZXX 的 fork，相比官方原版增加了：
> - `teleop_hand_and_arm.py` 的 null 安全检查（防止 `head_img` 为 `None` 时崩溃）
> - `teleop/robot_control/dex1_1_controller.py` — Dex1-1 二指夹爪的 Python 控制器


```bash
cd ~
git clone https://github.com/3457109829zzx-svg/xr_teleoperate.git
cd xr_teleoperate
git submodule update --init --depth 1

# 安装 teleimager
cd teleop/teleimager
pip install -e . --no-deps

# 安装 televuer
cd ../televuer
pip install -e .

# 安装 unitree_sdk2_python
cd ~
git clone https://github.com/unitreerobotics/unitree_sdk2_python.git
cd unitree_sdk2_python
pip install -e .
```

### 1.4 生成 SSL 证书（用于 PICO 头显 WebRTC 连接）

```bash
cd ~/xr_teleoperate/teleop/televuer

# PICO / Quest 用这个即可
openssl req -x509 -nodes -days 365 -newkey rsa:2048 -keyout key.pem -out cert.pem

# 把证书复制到默认路径
mkdir -p ~/.config/xr_teleoperate/
cp cert.pem key.pem ~/.config/xr_teleoperate/

# 设置环境变量（可选，保险起见）
echo 'export XR_TELEOP_CERT="$HOME/xr_teleoperate/teleop/televuer/cert.pem"' >> ~/.bashrc
echo 'export XR_TELEOP_KEY="$HOME/xr_teleoperate/teleop/televuer/key.pem"' >> ~/.bashrc
source ~/.bashrc
```

### 1.5 开放防火墙端口

```bash
sudo ufw allow 8012
```

---

## 二、连接机器人

### 2.1 物理连接

用**网线**连接笔记本网口和机器人。宇树 G1 机器人背部有网口面板：

```
笔记本（网口） ←──网线──→ G1 机器人交换机
```

### 2.2 配置笔记本有线网卡 IP

机器人 PC2 的 IP 是 `192.168.123.164`，笔记本需要在同一网段。

```bash
# 先查看你的有线网卡名称（一般是 enp 或 eth 开头）
ip link show

# 假设有线网卡是 enp109s0（**每个人不一样！用自己的网卡名替换**）
sudo ip addr add 192.168.123.2/24 dev enp109s0
```

> ⚠️ **重要**：每个人的笔记本网卡名称不同。`ip link show` 查看后，记住你的有线网卡名，后面启动程序要用 `--network-interface` 参数指定。

### 2.3 验证连接

```bash
ping 192.168.123.164
# 应该能 ping 通
```

---

## 三、启动遥操作

> **前提确认**：PC2 上的服务已由管理员启动：
> - ✅ `teleimager-server --rs`（图像推送服务）
> - ✅ `sudo ./inspire_g1_new`（因时灵巧手）或 `sudo ./dex1_1_gripper_server --network eth0`（Dex1-1 夹爪）

### 3.1 启动笔记本端遥操作程序

```bash
conda activate tv
cd ~/xr_teleoperate/teleop
```

**根据末端执行器类型选择命令**：

#### 因时 Inspire DFX 灵巧手：

```bash
python teleop_hand_and_arm.py \
    --arm=G1_29 \
    --ee=inspire_dfx \
    --img-server-ip 192.168.123.164 \
    --network-interface enp109s0 \
    --input-mode=controller \
    --display-mode=immersive \
    --record \
    --task-dir ./utils/data/
```

#### Dex1-1 二指夹爪：

```bash
python teleop_hand_and_arm.py \
    --arm=G1_29 \
    --ee=dex1 \
    --img-server-ip 192.168.123.164 \
    --network-interface enp109s0 \
    --input-mode=controller \
    --display-mode=immersive \
    --record \
    --task-dir ./utils/data/ \
    --task-name "你的任务名" \
    --task-goal "用英文描述任务目标（GR00T VLM 需要英文）" \
    --max-episodes 10
```

> ⚠️ `--network-interface` 请替换为你自己的有线网卡名（`ip link show` 查看）！

### 3.2 参数速查

| 参数 | 说明 |
|------|------|
| `--arm=G1_29` | 29 自由度 G1 |
| `--ee=inspire_dfx` | 末端执行器：因时灵巧手 |
| `--ee=dex1` | 末端执行器：Dex1-1 二指夹爪 |
| `--ee=dex3` | 末端执行器：Dex3-1 三指灵巧手 |
| `--network-interface` | **你的有线网卡名，每个人不同！** |
| `--img-server-ip` | 机器人图像服务器 IP，固定 `192.168.123.164` |
| `--input-mode=controller` | 用手柄控制 |
| `--input-mode=hand` | 用手势跟踪控制（可控制灵巧手） |
| `--display-mode=immersive` | 沉浸式（显示机器人摄像头画面） |
| `--display-mode=pass-through` | 通透模式（显示 VR 自带摄像头） |
| `--record` | 开启数据录制 |
| `--task-name` | 数据子目录名 |
| `--task-goal` | 任务目标（英文），用于 GR00T 训练 |
| `--max-episodes` | 最大录制 episode 数 |

---

## 四、PICO 头显连接

### 4.1 连接 WiFi

PICO 需要和笔记本在**同一局域网**（连同一个 WiFi 路由器，或连机器人自带的 WiFi）。

### 4.2 打开浏览器进入 Vuer

在 PICO 浏览器中打开（注意替换 IP 为你笔记本的 IP）：

```
https://192.168.123.2:8012/?ws=wss://192.168.123.2:8012
```

> 笔记本 IP 是你的有线网卡 IP（上面设的 `192.168.123.2`）

### 4.3 首次使用：信任证书

第一次打开会看到安全警告页面，点击 **Advanced** → **Proceed to IP (unsafe)**。

### 4.4 进入 VR 模式

在 Vuer 页面中点击 **Virtual Reality**，允许所有权限，开始 VR 会话。

此时头显中会看到机器人头部相机的画面。

---

## 五、操作流程

### 5.1 手柄按钮功能

| 手柄 | 按钮 | 功能 |
|------|------|------|
| 左手 | **Y** | 启动遥操作 / 暂停 / 继续（循环切换） |
| 左手 | **X** | 结束遥操作，双臂复位 |
| 右手 | **B** | 开始录制 / 停止录制（反复操作产生多个 episode） |
| 右手 | **A** | 删除最新录制的 episode |
| 右手 | **扳机** | 控制灵巧手/夹爪开合 |

### 5.2 键盘快捷键（备用）

| 按键 | 功能 |
|------|------|
| `r` | 启动遥操作 |
| `s` | 开始/停止录制 |
| `q` | 退出程序 |

### 5.3 标准操作流程

1. **戴上 PICO 头显**，在浏览器中进入 Vuer → Virtual Reality
2. **对齐初始姿态**：将你的手臂放到与机器人初始姿态相似的位置
3. **按 Y（或键盘 r）**：启动遥操作。终端提示 `IK 偏移量标定完成`
4. **开始控制**：移动手臂控制机器人，右手扳机控制夹爪/灵巧手
5. **按 B 开始录制**：终端提示 `Recording started`
6. **操作完成后按 B 停止录制**：数据自动保存
7. **按 Y 暂停**，再按 Y 继续；**按 X 结束**遥操作
8. **按 q 退出**程序

> ⚠️ **安全提示**：
> - 退出前先让手臂回到初始姿态，再按 q
> - 与机器人保持安全距离
> - 紧急情况：同时按下两个手柄摇杆 = 软急停（切换到阻尼模式）

---

## 六、参数速查表

### 按末端执行器

| 末端执行器 | `--ee` 参数 | PC2 服务 | 备注 |
|-----------|------------|---------|------|
| Inspire DFX 灵巧手 | `inspire_dfx` | `sudo ./inspire_g1_new` | 6 自由度灵巧手 |
| Dex1-1 二指夹爪 | `dex1` | `sudo ./dex1_1_gripper_server --network eth0` | 简单夹持 |
| Dex3-1 三指灵巧手 | `dex3` | 自带 | 仅仿真可用 |

### 按控制模式

| `--input-mode` | 控制方式 | 能否控制灵巧手 |
|---------------|---------|:---:|
| `controller` | PICO 手柄追踪 | ❌（只用扳机） |
| `hand` | 手势跟踪 | ✅ |

---

## 七、可选：查看机器人视角图像流

如果想在笔记本上直接查看图像（不通过头显）：

```bash
(tv) ~/xr_teleoperate/teleop/teleimager/src$ python -m teleimager.image_client --host 192.168.123.164
```

或在浏览器打开：`https://192.168.123.164:60001`，点击 Start 按钮。

---

## 八、常见问题

### Q: 启动时报 "Network interface not found"

你的有线网卡名和 `--network-interface` 参数不匹配。运行 `ip link show` 查看正确的网卡名。

### Q: PICO 头显看不到画面

1. 确认 PC2 上 `teleimager-server --rs` 在运行
2. 确认笔记本能 ping 通 `192.168.123.164`
3. 确认 PICO 和笔记本在同一网络
4. 检查笔记本防火墙：`sudo ufw status`

### Q: 灵巧手不受控制

1. 确认 PC2 上的灵巧手服务在运行
2. 检查 Dex1-1 是否被多个进程占用（Dex1-1 一直开合说明有冲突）

### Q: 启动后手臂位置突变

正常现象——IK 偏移量标定需要几帧完成。标定完成前手臂会有小幅抖动。

### Q: Realsense 画面黑屏

1. 拔插 Realsense Type-C 数据线
2. 重新配置 PC2 的 IP：`sudo ip addr add 192.168.123.164/24 dev eth0`
3. 检查是否有其他进程占用：`sudo lsof /dev/video*`

---

## 九、数据存储位置

录制数据默认保存在：

```
~/xr_teleoperate/teleop/utils/data/<task_name>/
```

目录结构：
```
data/
└── <task_name>/
    ├── episode_0/
    │   ├── meta/          # 任务描述等元信息
    │   ├── images/        # 头显和前腕相机图像
    │   └── joints.parquet # 关节角度
    ├── episode_1/
    └── ...
```

数据可用于 [unitree_IL_lerobot](https://github.com/unitreerobotics/unitree_IL_lerobot) 训练模仿学习策略。

---

## 十、数据采集规范

为确保采集高质量数据：

1. **动作要平滑**：不要突然快速移动，机器人会抖动
2. **保持一致性**：同一个任务的所有 episode 用相同的 `--task-goal`
3. **task-goal 用英文**：GR00T 的 VLM 是英文训练的，描述要具体，如：
   - `"pick up the cube from the table and place it onto the plate"`
   - `"grasp the aviation plug and insert it into the socket"`
4. **失败就删除**：右手按 A 删除最新的失败 episode，无需退出程序
5. **注意磁盘空间**：每个 episode 包含图像数据，磁盘不足会导致录制中断
6. **初始姿态对齐**：每次启动遥操作前，确保你的手臂和机器人初始姿态一致，避免启动时突变
