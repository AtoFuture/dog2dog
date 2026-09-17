# 一次目标检测的结果。
#
# 接口 03（G2 视觉探索 -> G3 集成评测）
# 契约来源：baseline/README.md 的接口表
#
# 语义冻结（按契约硬规矩 2：改语义必须全队通知，加字段可随时）
#   * pose 是 **map 系** 下的三维坐标 —— 契约点名的红线。
#     要改成相机系或机体系必须全队通知，否则三组一起挂。
#   * stamp 是 **图像采集时刻**，不是处理完成时刻。
#     用处理时刻会让三维坐标与当时的 TF 对不上，尤其在回传时延大的链路上。
#
# 本消息**不含抓拍图**。图走独立话题，见 DetectionSnapshot.msg。

string class_name        # 类别名（COCO 类名，或异常类 person_fallen / smoke）

float32 confidence       # 置信度 0~1

geometry_msgs/PoseWithCovariance pose
# 三维坐标，map 系。
#
# position    目标代表点。
#             代表点的定义（bbox 中心 / 底部中点）由 G2 侧参数控制，
#             并在 Detection3D 之外的文档中约定 —— 这个语义一变，
#             G3 的评测口径会跟着变，比坐标系变化更隐蔽。
#
# covariance  只填左上 3x3（x/y/z 的方差），其余置零。
#             数值来自三维解算时像素的离散程度，是**下界**：
#             不含外参、TF、时间同步误差。
#             用途：G3 可据此筛掉坐标不可信的检测，而不必全盘接受或全盘丢弃。
#
# orientation **不使用**，恒为单位四元数 (0,0,0,1)。
#             G3 不应对它做任何假设 —— 检测结果本质是一个点，没有朝向。

builtin_interfaces/Time stamp   # 图像采集时刻

uint32 id                # 跟踪 ID（去重用）。
                         # 同一目标被连续检出时 id 不变。G3 据此可区分
                         # 「同一目标的多次上报」与「多个不同目标」——
                         # 没有它，检出率/误检率的统计只能依赖
                         # "上游发了几条"这种脆弱假设。

string source            # 来源：coco | pose | heuristic
                         # 让 G3 能**确定性过滤**，不必靠 class_name 字符串约定去猜。
                         # 例如「异常类不计入误检率」直接判 source != "coco" 即可。

bool has_snapshot        # 本检测在 /detections_snapshot 上是否有对应抓拍图。
                         # G3 做指标统计时不必订阅图像话题；只在需要看图时订阅，
                         # 并靠 (id, stamp) 匹配。
