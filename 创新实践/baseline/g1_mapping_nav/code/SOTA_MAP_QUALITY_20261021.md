# 2D 栅格地图"毛刺/放射条纹"问题：SOTA 与最佳实践
> 检索日期：2026-10-21（arXiv API，条目均为真实论文）

## 1. 现象与成因

你看到的"毛刺/放射条纹"不是显示错误，是 **2D 占据栅格建图中的典型伪影**：

| 现象 | 成因 | 我们的具体情况 |
|---|---|---|
| 放射状细条纹 | 激光射到量程边缘/未观测区，ray-casting 把"未知"画成细条 | `max_laser_range` 设得大 + 2Hz 低帧率 |
| 孤立黑点（毛刺） | 单次噪声回波、动态物体、位姿抖动 | 雷达噪声 + 里程计/位姿误差 |
| 墙断断续续、大片空 | 观测不足、视场小、遮挡 | `/robot1/scan` **只有 200° 视场**；`/robot1/velodyne` 的 ranges 是 `-.inf`（无效） |
| 货架几乎不出现 | **激光只认碰撞体，Depot 货架只有视觉无碰撞** | 只能看到 `depot_collision` 的外围墙+少量箱排 |

## 2. 学术 SOTA（2026 年 10 月）

### 2.1 鲁棒里程计 / 扫描匹配（毛刺的第一来源是位姿误差）
- **HP2-SLAM: Adaptive Hybrid ICP**（2608.14996, 2026-08）——自适应混合 ICP，抗噪声/退化。
- **Degeneracy-Orthogonal Geometric Constraints for LiDAR SLAM**（2609.36753, 2026-09）——退化场景（长走廊/开阔区）几何约束。
- **Improving Map Consistency … Information-Aware Odometry and Retroactive Loop Closure**（2607.13516, 2026-07）——**回头闭环重优化**，直接改善地图一致性、消除重影。
- **Chain-SLAM**（2609.12221, 2026-09）——链式多会话回环，后端全局一致。

### 2.2 占据建图（Occupancy Mapping）
- **Large-Scale Continuous Occupancy Mapping via Variance-Weighted Submap Joining**（2606.10442, 2026-06）——方差加权子图融合，系统性减少拼接伪影。
- **GOFusion: Real-Time Gaussian-Occupancy Submap Fusion**（2610.25261, 2026-10）——高斯占据 + 子图融合，实时且平滑。
- **Finite-Sample Conformal Coverage Recovery … Occupancy Map Estimation**（2607.14906, 2026-07）——对"未知/覆盖"的不确定性做校准（抑制虚假占据）。
- **RICH-SLAM: Incremental and Continuous Hilbert Mapping**（2606.17534, 2026-06）——连续隐式表示，避免离散栅格的量化毛刺。

### 2.3 神经隐式 / 学习式建图（新一代 SOTA）
- **XGrid-Mapping**（2512.20976, 2025-12）——显式-隐式混合栅格子图，增量式神经 LiDAR 建图。
- **Neural Implicit Energy Optimization for LiDAR Mapping**（2610.28598, 2026-10）。
- **LIN-SLAM: Lightweight LiDAR–Inertial Neural SLAM**（2610.21199, 2026-10）——机载实时。
- **Hi-LOAM: Hierarchical Implicit Neural Fields**（2604.01720, 2026-04）。

### 2.4 地图后处理 / 精修（最直接对应"去毛刺"）
- **VoxelFix: Post-Hoc Semantic Correction of Completed 3D Voxel Maps**（2609.05114, 2026-09）——**事后地图修正**，去掉错误/稀疏体素。
- **VEOcc++: Persistent Voxel Feature Maps … Online + Offline Refinement**（2610.27704, 2026-10）——在线建图 + 离线精修双阶段。
- **PseudoMapLabeler**（2608.12600, 2026-08）——置信度感知的伪标签，用于清理。

### 2.5 动态物体移除（移动目标会在图上留"拖影/毛刺"）
- **All-St3R SLAM: Pixel-wise Motion Masking**（2610.16143, 2026-10）。
- **MotionGS-SLAM: Event-Modulated Gaussian Splatting for Motion-Blur Robust SLAM**（2608.15024, 2026-08）。
- **FMCW-LIO: Doppler LiDAR-Inertial Odometry**（2609.29374, 2026-09）——用多普勒区分动/静点。
- **Does Dynamic-Point Filtering Help When Texture Is Scarce?**（2610.10564, 2026-10）——**受控研究**：动态点过滤在特征稀疏场景的收益有限（说明别指望单靠滤波）。

### 2.6 点云去噪（底层）
- **GUARD: Geometric Uncertainty-Aware Point Cloud Denoising**（2610.09068, 2026-10）。
- **Test-time Adaptation for Point Cloud Denoising under Unseen Noise**（2610.27410, 2026-10）。
- **SIMPC**（2605.26894）、**U-CAN**（2510.25210）、**Adaptive Dual-Weighted Gravitational Denoising**（2512.10386）——无监督/自监督去噪。
- 数据集：**Ghost-FWL**（2603.28224, 2026-03，全波形 LiDAR 鬼影检测/去除）。

## 3. 工业界/工程最佳实践（可直接落地）

1. **先保证输入正确**（最重要）：
   - 2D 扫描必须 **360°**、`ranges` 合法（不能是 `-.inf`）、带时间戳、帧率 ≥10Hz。
   - 给**货架补碰撞体**（否则激光看不见，地图必然空旷）。
2. **占据更新用对数几率（log-odds）+ 命中/未命中**，不要用单帧二值化；合理设置 `occupied/free` 阈值。
3. **限制量程**（如 10–15m）并丢弃最近 0.2m，减少量程边缘的放射条纹。
4. **动/静分离**：ray-casting free-space carving + 动态点过滤（参考 2610.10564 的结论：要配合其它手段）。
5. **回环 + 后端重优化**：跑完一段后做 pose-graph 优化与"重投影建图"（2607.13516、Chain-SLAM）。
6. **离线精修**：`VoxelFix`/`VEOcc++` 式后处理，或工程上常用的形态学去小连通域 + 孔洞填充（我们已在用）。
7. **不确定度建模**：方差加权/GP/连续表示（2606.10442、GOFusion、RICH-SLAM），从源头抑制虚假占据。

## 4. 针对本项目的结论

- **根因不是算法选错**，而是**输入/世界配置**：
  1. `/robot1/velodyne` ranges 全 `-.inf`（无效）；
  2. `/robot1/scan` 仅 **200°** 视场、~2Hz；
  3. **货架无碰撞体**，激光看不到。
- 所以"调 occupancy_threshold"治标不治本：0.1 有墙但毛刺多，0.25/0.5 墙消失。
- **要得到干净实时图，按优先级**：
  1. 给货架/墙补碰撞体（或改用 `warehouse_cross` 那套有碰撞的世界）；
  2. 使用 360°、≥10Hz 的有效 2D 扫描；
  3. slam 参数：log-odds 合理阈值 + 量程 10–15m；
  4. 加**实时降噪中继**（已做）+ 跑完做**离线精修**；
  5. 若要"绝对干净"的参考底图，用**GT 几何图**。

## 5. 参考（真实 arXiv 条目）

2608.14996 · 2609.36753 · 2607.13516 · 2609.12221 · 2606.10442 · 2610.25261 · 2607.14906 · 2606.17534 · 2512.20976 · 2610.28598 · 2610.21199 · 2604.01720 · 2609.05114 · 2610.27704 · 2608.12600 · 2610.16143 · 2608.15024 · 2609.29374 · 2610.10564 · 2610.09068 · 2610.27410 · 2605.26894 · 2510.25210 · 2512.10386 · 2603.28224
