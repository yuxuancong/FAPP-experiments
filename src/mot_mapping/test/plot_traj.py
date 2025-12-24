#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
KF轨迹对比绘图脚本
绘制三种运动状态下标准KF与自适应KF的轨迹对比
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os

plt.rcParams['font.sans-serif'] = ['SimHei', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False

motion_types = [
    ('constant_velocity', '匀速直线运动'),
    ('sinusoidal', '正弦变速运动'),
    ('variable_accel', '变加速运动')
]

fig, axes = plt.subplots(3, 2, figsize=(14, 12))

for idx, (motion_en, motion_cn) in enumerate(motion_types):
    filename = f'traj_{motion_en}.csv'
    if not os.path.exists(filename):
        print(f"文件不存在: {filename}")
        continue
    
    df = pd.read_csv(filename)
    
    # 左图: XY轨迹对比
    ax1 = axes[idx, 0]
    ax1.plot(df['gt_x'], df['gt_y'], 'k-', linewidth=2, label='真值轨迹')
    ax1.plot(df['skf_x'], df['skf_y'], 'b--', linewidth=1.5, alpha=0.8, label='标准KF')
    ax1.plot(df['akf_x'], df['akf_y'], 'r:', linewidth=1.5, alpha=0.8, label='自适应KF')
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_title(f'{motion_cn} - XY轨迹对比')
    ax1.legend(loc='best')
    ax1.grid(True, alpha=0.3)
    ax1.axis('equal')
    
    # 右图: 位置误差随时间变化
    ax2 = axes[idx, 1]
    ax2.plot(df['t'], df['skf_err'], 'b-', linewidth=1, alpha=0.7, label='标准KF误差')
    ax2.plot(df['t'], df['akf_err'], 'r-', linewidth=1, alpha=0.7, label='自适应KF误差')
    
    # 计算滑动平均
    window = 20
    skf_smooth = df['skf_err'].rolling(window=window, min_periods=1).mean()
    akf_smooth = df['akf_err'].rolling(window=window, min_periods=1).mean()
    ax2.plot(df['t'], skf_smooth, 'b-', linewidth=2, label=f'标准KF (滑动平均)')
    ax2.plot(df['t'], akf_smooth, 'r-', linewidth=2, label=f'自适应KF (滑动平均)')
    
    ax2.set_xlabel('时间 (s)')
    ax2.set_ylabel('位置误差 (m)')
    ax2.set_title(f'{motion_cn} - 位置误差对比')
    ax2.legend(loc='best')
    ax2.grid(True, alpha=0.3)
    
    # 计算并显示RMSE
    rmse_skf = np.sqrt(np.mean(df['skf_err']**2))
    rmse_akf = np.sqrt(np.mean(df['akf_err']**2))
    ax2.text(0.02, 0.98, f'RMSE: 标准={rmse_skf:.3f}, 自适应={rmse_akf:.3f}',
             transform=ax2.transAxes, fontsize=9, verticalalignment='top',
             bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

plt.tight_layout()
plt.savefig('kf_compare_result.png', dpi=150, bbox_inches='tight')
plt.savefig('kf_compare_result.pdf', bbox_inches='tight')
print("图表已保存: kf_compare_result.png / kf_compare_result.pdf")
plt.show()
