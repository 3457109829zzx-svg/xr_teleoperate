import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch
from matplotlib import font_manager
import numpy as np

# Chinese font
font_path = '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
font_manager.fontManager.addfont(font_path)
plt.rcParams['font.family'] = 'Noto Sans CJK JP'

fig, ax = plt.subplots(1, 1, figsize=(16, 8))
ax.set_xlim(0, 16)
ax.set_ylim(0, 8)
ax.set_aspect('equal')
ax.axis('off')

# Color palette
c_vr      = '#4A90D9'  # blue - VR side
c_proc    = '#E8833A'  # orange - processing
c_robot   = '#50B86C'  # green - robot side
c_data    = '#9B59B6'  # purple - data
c_bg_box  = '#F5F7FA'
c_border  = '#CCCCCC'
c_arrow   = '#555555'
c_text    = '#333333'

def draw_box(ax, x, y, w, h, text, color, fontsize=10, text_color='white', bold=True):
    """Draw a rounded box with centered text"""
    box = FancyBboxPatch((x - w/2, y - h/2), w, h,
                          boxstyle="round,pad=0.15", linewidth=1.5,
                          edgecolor=color, facecolor=color, alpha=0.92)
    ax.add_patch(box)
    weight = 'bold' if bold else 'normal'
    ax.text(x, y, text, ha='center', va='center', fontsize=fontsize,
            color=text_color, weight=weight, fontfamily='Noto Sans CJK JP')

def draw_sub_box(ax, x, y, w, h, text, fontsize=8, text_color=c_text):
    """Draw a subtler sub-component box"""
    box = FancyBboxPatch((x - w/2, y - h/2), w, h,
                          boxstyle="round,pad=0.08", linewidth=1,
                          edgecolor=c_border, facecolor='white', alpha=0.95)
    ax.add_patch(box)
    ax.text(x, y, text, ha='center', va='center', fontsize=fontsize,
            color=text_color, fontfamily='Noto Sans CJK JP')

def draw_arrow(ax, x1, y1, x2, y2, color=c_arrow, lw=2.0, style='->'):
    """Draw an arrow"""
    ax.annotate('', xy=(x2, y2), xytext=(x1, y1),
                arrowprops=dict(arrowstyle=style, color=color, lw=lw,
                                connectionstyle="arc3,rad=0"))

def draw_bidirectional(ax, x1, y1, x2, y2, color=c_arrow, lw=1.8):
    """Bidirectional arrow (two offsets)"""
    dy = 0.18
    ax.annotate('', xy=(x2, y2+dy), xytext=(x1, y1+dy),
                arrowprops=dict(arrowstyle='->', color=color, lw=lw,
                                connectionstyle="arc3,rad=0"))
    ax.annotate('', xy=(x1, y1-dy), xytext=(x2, y2-dy),
                arrowprops=dict(arrowstyle='->', color=color, lw=lw,
                                connectionstyle="arc3,rad=0"))

# === Title ===
ax.text(8, 7.5, 'XR Teleoperate — 系统架构框图', ha='center', va='center',
        fontsize=18, weight='bold', color='#222222', fontfamily='Noto Sans CJK JP')

# === Group backgrounds ===
# VR group
vr_bg = FancyBboxPatch((0.3, 1.5), 4.2, 4.8, boxstyle="round,pad=0.3",
                        linewidth=0, facecolor='#EBF2FB', alpha=0.6)
ax.add_patch(vr_bg)
ax.text(2.4, 6.0, 'XR 设备端', ha='center', va='center', fontsize=11,
        weight='bold', color='#2C5F9E', fontfamily='Noto Sans CJK JP')

# Processing group
proc_bg = FancyBboxPatch((5.7, 1.5), 4.8, 4.8, boxstyle="round,pad=0.3",
                          linewidth=0, facecolor='#FDF0E6', alpha=0.6)
ax.add_patch(proc_bg)
ax.text(8.1, 6.0, '核心处理层', ha='center', va='center', fontsize=11,
        weight='bold', color='#B85D19', fontfamily='Noto Sans CJK JP')

# Robot group
robot_bg = FancyBboxPatch((11.7, 1.5), 4.2, 4.8, boxstyle="round,pad=0.3",
                           linewidth=0, facecolor='#EDF7EF', alpha=0.6)
ax.add_patch(robot_bg)
ax.text(13.8, 6.0, '机器人执行端', ha='center', va='center', fontsize=11,
        weight='bold', color='#3A7D44', fontfamily='Noto Sans CJK JP')

# === VR side ===
draw_box(ax, 2.4, 5.0, 3.0, 0.9, 'VR 头显\n(WebXR 数据采集)', c_vr, fontsize=9)
draw_sub_box(ax, 2.4, 3.8, 2.6, 0.6, '头部/腕部 6-DoF 位姿')
draw_sub_box(ax, 2.4, 3.0, 2.6, 0.6, '手部 25 点骨架数据')
draw_sub_box(ax, 2.4, 2.2, 2.6, 0.6, '手柄扳机/按键指令')

# === Processing side ===
draw_box(ax, 8.1, 5.0, 3.6, 0.9, 'Televuer Wrapper\n(坐标系转换 & 滤波)', c_proc, fontsize=9)

draw_sub_box(ax, 6.6, 3.8, 2.0, 0.65, 'Pinocchio\n运动学建模')
draw_sub_box(ax, 9.6, 3.8, 2.0, 0.65, 'CasADi + IPOPT\nIK 优化求解')

draw_sub_box(ax, 8.1, 2.9, 2.8, 0.6, 'dex-retargeting 手部重定向')

draw_sub_box(ax, 8.1, 2.2, 2.5, 0.55, '前馈力矩 (RNE τ_ff)')

# === Robot side ===
draw_box(ax, 13.8, 5.0, 3.0, 0.9, 'Unitree G1\n人形机器人', c_robot, fontsize=9)
draw_sub_box(ax, 13.8, 3.8, 2.6, 0.6, '双臂 14 关节控制')
draw_sub_box(ax, 13.8, 3.0, 2.6, 0.6, '因时 Inspire 灵巧手')
draw_sub_box(ax, 13.8, 2.2, 2.6, 0.6, '头部 RealSense 相机')

# === Arrows: VR → Processing ===
draw_arrow(ax, 3.95, 5.0, 5.65, 5.0, c_arrow, lw=2.0)
ax.text(4.8, 5.3, '位姿/骨架', ha='center', va='bottom', fontsize=7, color=c_arrow)

# === Arrows: Processing → Robot ===
draw_arrow(ax, 9.95, 5.0, 11.65, 5.0, c_arrow, lw=2.0)
ax.text(10.8, 5.3, 'DDS 指令', ha='center', va='bottom', fontsize=7, color=c_arrow)

# IK → arm control
draw_arrow(ax, 9.6, 3.8, 12.4, 3.8, c_arrow, lw=1.5)
ax.text(11.0, 4.0, '关节角度', ha='center', va='bottom', fontsize=7, color=c_arrow)

# Hand → EE
draw_arrow(ax, 9.55, 2.9, 12.4, 2.9, c_arrow, lw=1.5)
ax.text(11.0, 3.1, '手指关节', ha='center', va='bottom', fontsize=7, color=c_arrow)

# === Feedback loop: Camera back to VR ===
# Dotted curve from robot camera back to VR
style_curve = "arc3,rad=-0.5"
ax.annotate('', xy=(2.4, 2.2), xytext=(13.8, 2.2),
            arrowprops=dict(arrowstyle='->', color='#D64550', lw=2.2,
                            connectionstyle=style_curve, linestyle='dashed'))
ax.text(8.1, 0.7, '实时图像回传 (ZMQ/WebRTC)', ha='center', va='center',
        fontsize=9, color='#D64550', weight='bold', fontfamily='Noto Sans CJK JP',
        bbox=dict(boxstyle='round,pad=0.3', facecolor='white', edgecolor='#D64550', alpha=0.85))

# === DDS communication label ===
ax.text(8.1, 1.15, '基于 CycloneDDS 的实时通信 (250Hz 控制 / 30Hz 遥操作)',
        ha='center', va='center', fontsize=8, color='#777777', fontfamily='Noto Sans CJK JP')

# === Data recording box on right side ===
draw_box(ax, 13.8, 0.9, 2.8, 0.55, '数据录制 (模仿学习)', c_data, fontsize=8)

# === Legend ===
legend_y = 0.35
legend_items = [
    ('VR 位姿采集', c_vr),
    ('核心算法处理', c_proc),
    ('机器人控制执行', c_robot),
    ('数据录制与学习', c_data),
]
for i, (label, color) in enumerate(legend_items):
    lx = 2.5 + i * 3.3
    rect = mpatches.Rectangle((lx - 0.15, legend_y - 0.12), 0.3, 0.24,
                               linewidth=0, facecolor=color, alpha=0.85)
    ax.add_patch(rect)
    ax.text(lx + 0.2, legend_y, label, ha='left', va='center', fontsize=8,
            color='#555555', fontfamily='Noto Sans CJK JP')

plt.tight_layout()
plt.savefig('/home/zzx/xr_teleoperate/xr_teleoperate_arch.png', dpi=200,
            bbox_inches='tight', facecolor='white', edgecolor='none')
plt.close()
print("Done: /home/zzx/xr_teleoperate/xr_teleoperate_arch.png")
