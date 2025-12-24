# FAPP-KF: 卡尔曼滤波器对比实验

基于 [FAPP](https://github.com/arclab-hku/FAPP) (Fast and Adaptive Perception and Planning) 平台的卡尔曼滤波器对比研究。

[![Original Paper](https://img.shields.io/badge/Paper-IEEE%20TRO-004088)](https://ieeexplore.ieee.org/document/10816005)
[![arXiv](https://img.shields.io/badge/arXiv-2312.08743-24CC00)](https://arxiv.org/pdf/2312.08743.pdf)

---

## 📋 项目概述

本项目在 FAPP 多目标跟踪仿真平台上，实现并对比了两种卡尔曼滤波器：

| 滤波器类型 | 特点 | 适用场景 |
|-----------|------|----------|
| **Standard KF** | 固定观测噪声协方差 $R$ | 噪声稳定、短期跟踪 |
| **Adaptive KF** | 基于创新序列动态估计 $R_t$ | 噪声变化、长期跟踪 |

## 🚀 快速开始

### 环境要求

- Ubuntu 20.04
- ROS Noetic
- Eigen3, PCL 1.10+

### 编译

```bash
cd ~/catkin_ws/src
git clone https://github.com/YOUR_USERNAME/FAPP.git
cd ..
catkin build
source devel/setup.bash
```

### 运行仿真

```bash
# 终端1: 启动仿真环境
roslaunch fapp_planner quick_start.launch

# 终端2: 启动 KF 对比节点
rosrun mot_mapping kf_compare_node _output_csv:=/path/to/kf_compare_log.csv
```

### 参数配置

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `~output_csv` | `/tmp/kf_compare_log.csv` | CSV 输出路径 |
| `~gt_topic` | `/map_generator/obj_gt` | 真值话题 |
| `~est_topic` | `/states` | 估计话题 |
| `~dt` | `0.02` | 采样周期 (秒) |

---

## 📊 实验结果

### 性能对比

| 指标 | Standard KF | Adaptive KF | 改进 |
|------|-------------|-------------|------|
| 稳态 RMSE | 15.05 m | 14.60 m | **↓ 3.0%** |
| 最低 RMSE | 14.94 m | 14.46 m | **↓ 3.2%** |
| 收敛时间 | ~12 秒 | ~20 秒 | - |

### 结论

- **Adaptive KF** 在稳态精度上优于 Standard KF（改进 ~3-4%）
- **Standard KF** 收敛速度更快
- 长期跟踪任务推荐使用 **Adaptive KF**

详细分析见 [KF_Comparison_Report.md](KF_Comparison_Report.md)

---

## 📁 项目结构

```
FAPP/
├── src/
│   ├── mot_mapping/                    # 多目标跟踪模块
│   │   ├── include/
│   │   │   └── kf_compare.hpp          # KF 算法实现
│   │   ├── src/
│   │   │   └── kf_compare_node.cpp     # ROS 对比节点
│   │   └── test/
│   │       └── test_kf_compare.cpp     # 离线测试
│   ├── planner/                        # 规划模块
│   └── simulation/                     # 仿真模块
│       └── uav_simulator/
│           └── map_generator/          # 地图生成（含 GT 发布）
├── KF_Comparison_Report.md             # 实验报告
└── README.md
```

---

## 🔧 核心算法

### Standard KF
$$R = \sigma_r^2 I, \quad \sigma_r = 0.5 \text{ (固定)}$$

### Adaptive KF
$$\hat{R}_k = \frac{1}{N} \sum_{i=k-N+1}^{k} \nu_i \nu_i^T - H P_{k|k-1} H^T$$
$$R_k = \alpha \hat{R}_k + (1-\alpha) R_{k-1}, \quad \alpha = 0.3$$

---

## 📚 参考文献

本项目基于以下工作：

```bibtex
@article{lu2024fapp,
  title={FAPP: Fast and Adaptive Perception and Planning for UAVs in Dynamic Cluttered Environments},
  author={Lu, Minghao and Fan, Xiyu and Chen, Han and Lu, Peng},
  journal={IEEE Transactions on Robotics},
  year={2024}
}
```

---

## 📄 许可证

本项目遵循 [MIT License](LICENSE)

---

**作者**: 大连理工大学  
**日期**: 2025-12-24


