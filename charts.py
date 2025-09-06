import subprocess
import os
import pandas as pd
import io
import matplotlib.pyplot as plt

# ---------- 配置中文字体 ----------
plt.rcParams['font.sans-serif'] = ['SimHei']
plt.rcParams['axes.unicode_minus'] = False

# ---------- 调用 Rust exe 并获取 stdout ----------
exe_path = os.path.abspath("target\\debug\\opti_radar_main.exe")

# 假设 exe 运行时不需要额外参数，如果需要可在列表中添加
result = subprocess.run([exe_path], capture_output=True, text=True)

if result.returncode != 0:
    print("Rust 可执行文件运行失败")
    print(result.stderr)
    exit(1)

# ---------- 从 stdout 读取 CSV 数据 ----------
# Rust 输出示例：
# TargetID,TrueX,TrueY,TrueZ,EstX,EstY,EstZ,AvgError
# Target_1,39.23,34.96,... 
stdout_str = result.stdout
# 将 stdout 字符串转换成类似文件对象
csv_buffer = io.StringIO(stdout_str)

# 使用 pandas 读取
df = pd.read_csv(csv_buffer)

# ---------- 绘制 3D 位置对比图 ----------
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

fig = plt.figure(figsize=(10, 7))
ax = fig.add_subplot(111, projection='3d')

# 真值点
ax.scatter(df['TrueX'], df['TrueY'], df['TrueZ'], c='green', marker='o', s=60, label='真实位置')
# 估计点
ax.scatter(df['EstX'], df['EstY'], df['EstZ'], c='red', marker='^', s=60, label='估计位置')

# 自动调整坐标轴范围
x_all = pd.concat([df['TrueX'], df['EstX']])
y_all = pd.concat([df['TrueY'], df['EstY']])
z_all = pd.concat([df['TrueZ'], df['EstZ']])
ax.set_xlim(x_all.min() - 1, x_all.max() + 1)
ax.set_ylim(y_all.min() - 1, y_all.max() + 1)
ax.set_zlim(z_all.min() - 1, z_all.max() + 1)

# 添加估计点标签，略微偏移
for i, row in df.iterrows():
    ax.text(row['EstX']+0.5, row['EstY']+0.5, row['EstZ']+0.5, row['TargetID'], color='red', fontsize=9)

ax.set_xlabel("X 位置")
ax.set_ylabel("Y 位置")
ax.set_zlabel("Z 位置")
ax.set_title("目标真实位置 vs 估计位置")
ax.legend()
plt.tight_layout()
plt.show()

# ---------- 绘制误差柱状图 ----------
plt.figure(figsize=(8,5))
plt.bar(df['TargetID'], df['AvgError'], color='skyblue')
plt.ylabel('平均误差 (米)')
plt.title('每个目标的估计误差')
plt.grid(axis='y', linestyle='--', alpha=0.7)
plt.show()
