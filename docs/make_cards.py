#!/usr/bin/env python3
"""Generate two X-post images for the FNIRSI DPS-150 ROS 2 driver."""
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager as fm
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch

JP = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
JPB = "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc"
jp = fm.FontProperties(fname=JP)
jpb = fm.FontProperties(fname=JPB)
mono = fm.FontProperties(family="monospace")

BG = "#0d1117"
FG = "#e6edf3"
ACCENT = "#2f81f7"
GREEN = "#3fb950"
MUTED = "#8b949e"
CARD = "#161b22"
BORDER = "#30363d"

# ---------- Image 1: feature card ----------
fig = plt.figure(figsize=(12.8, 7.2), dpi=125)
fig.patch.set_facecolor(BG)
ax = fig.add_axes([0, 0, 1, 1]); ax.set_xlim(0, 128); ax.set_ylim(0, 72); ax.axis("off")
ax.set_facecolor(BG)

ax.text(7, 63, "FNIRSI DPS-150", fontproperties=jpb, fontsize=40, color=FG)
ax.text(7, 55.5, "ROS 2 ドライバー", fontproperties=jpb, fontsize=30, color=ACCENT)
ax.text(7, 50, "プログラマブルDC電源を ROS 2 から制御・モニタリング",
        fontproperties=jp, fontsize=15, color=MUTED)

# architecture row
def box(x, w, label, sub, color):
    ax.add_patch(FancyBboxPatch((x, 31), w, 9, boxstyle="round,pad=0.4,rounding_size=1.2",
                                fc=CARD, ec=color, lw=2.2))
    ax.text(x + w/2, 37.2, label, fontproperties=jpb, fontsize=16, color=FG, ha="center")
    ax.text(x + w/2, 33.4, sub, fontproperties=jp, fontsize=11, color=MUTED, ha="center")

box(7, 30, "DPS-150", "実機（USB-C）", ACCENT)
box(49, 30, "ROS 2 ノード", "C++ / rclcpp", GREEN)
box(91, 30, "ロボット / アプリ", "topic・service", ACCENT)

def arrow(x0, x1, label):
    ax.add_patch(FancyArrowPatch((x0, 35.5), (x1, 35.5), arrowstyle="-|>",
                                 mutation_scale=22, color=MUTED, lw=2))
    ax.text((x0+x1)/2, 37.8, label, fontproperties=jp, fontsize=10.5, color=MUTED, ha="center")
arrow(37.5, 48.5, "USB CDC serial")
arrow(79.5, 90.5, "ROS 2 API")

# features
feats = [
    "電圧・電流・電力・温度・保護状態を topic で取得",
    "電圧/電流の設定、出力ON/OFF、プリセットを service で操作",
    "自作の制御回路なしで電源をプログラム制御",
    "POSIX termios 直叩き（外部シリアルライブラリ・Python依存なし）",
]
for i, t in enumerate(feats):
    y = 24 - i*4.6
    ax.text(8, y, "✓", fontproperties=jpb, fontsize=15, color=GREEN)
    ax.text(11.5, y, t, fontproperties=jp, fontsize=14.5, color=FG)

ax.text(7, 2.5, "github.com/Raptor-zip/fnirsi_dps150_ros2_driver",
        fontproperties=jp, fontsize=13, color=ACCENT)
ax.text(121, 2.5, "MIT License", fontproperties=jp, fontsize=12, color=MUTED, ha="right")
fig.savefig("/tmp/x_post/card_features.png", facecolor=BG)
print("wrote card_features.png")

# ---------- Image 2: terminal demo ----------
fig2 = plt.figure(figsize=(12.8, 7.2), dpi=125)
fig2.patch.set_facecolor(BG)
ax2 = fig2.add_axes([0, 0, 1, 1]); ax2.set_xlim(0, 128); ax2.set_ylim(0, 72); ax2.axis("off")
# terminal window
ax2.add_patch(FancyBboxPatch((6, 6), 116, 60, boxstyle="round,pad=0.6,rounding_size=1.5",
                             fc="#0b0e14", ec=BORDER, lw=2))
# title bar dots
for i, c in enumerate(["#ff5f56", "#ffbd2e", "#27c93f"]):
    ax2.add_patch(plt.Circle((11 + i*3, 62.5), 0.9, color=c))
ax2.text(64, 62, "dps150 — ROS 2", fontproperties=jp, fontsize=12, color=MUTED, ha="center")

lines = [
    ("$ ros2 launch fnirsi_dps150_driver dps150.launch.xml \\", FG),
    ("    port:=/dev/serial/by-id/usb-Artery_AT32_...-if00", FG),
    ("[INFO] [dps150]: Connected to DPS-150", GREEN),
    ("", FG),
    ("$ ros2 service call /dps150/set_voltage \\", FG),
    ("    fnirsi_dps150_driver/srv/SetFloat32 \"{value: 12.0}\"", FG),
    ("$ ros2 service call /dps150/enable_output \\", FG),
    ("    std_srvs/srv/SetBool \"{data: true}\"", FG),
    ("response: success=True  message='output enabled'", GREEN),
    ("", FG),
    ("$ ros2 topic echo /dps150/state", FG),
    ("connected: true", FG),
    ("output_voltage: 12.0", ACCENT),
    ("output_current: 0.039", ACCENT),
    ("output_power:   0.46", ACCENT),
    ("temperature:    31.4", FG),
    ("mode: CV   protection: OK", MUTED),
]
y = 56
for txt, col in lines:
    ax2.text(10, y, txt, fontproperties=mono, fontsize=13.5, color=col, va="top")
    y -= 2.85
fig2.savefig("/tmp/x_post/card_terminal.png", facecolor=BG)
print("wrote card_terminal.png")
