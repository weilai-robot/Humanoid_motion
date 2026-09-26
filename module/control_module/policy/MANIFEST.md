# policy 模型台账

- 外层 = policy/ 根目录中的模型为活跃模型
- 内层（aichive) = 历史模型

## 活跃模型

| 文件名                                    | 来源                                       |
| -------------------------------------- | ---------------------------------------- |
| rl\_walk\_leg\_20260901\_step7500.onnx | 参数辨识+低抬步态模型，可实现前进、停止、横移、转向               |
| rl\_walk\_leg\_20260818\_step5999.onnx | 基于源码训练5999轮模型，可实现前进、后退、停止、横移、转向，用于视频拍摄模型 |

## BeyondMimic X1 舞蹈资产

| 文件 | 来源与校验 |
| --- | --- |
| `dance_x1_4999.bin` | `Beyondminic-Weilai-X1/trained_models/dance_x1_model_4999.pt` 的 160→29 Actor、观测归一化和训练关节参数；SHA256 `6939c1c69a6d509a85b4f7cfbbbd0ea8dae875f7507a30256364f3e3ec1c5f77` |
| `dance_x1_motion.bin` | `whole_body_tracking/motions/dance_x1_train.npz`，647 帧、50 Hz，参考关节和 `lumbar_pitch_link` 锚点；SHA256 `dd3376f615ee86daf6df4f1b13972942376d5a2275e904c4efb1674691bf884d` |

上述资产由 `module/control_module/tools/export_beyondmimic_x1.py` 导出。模型和轨迹中的 29 关节已转换为训练时的 PhysX 顺序，加载时由 `DanceController` 与 YAML 逐一核对。

## 历史模型

| 文件名                                    | 来源                   |
| -------------------------------------- | -------------------- |
| rl\_walk\_leg\_20260723\_step5000.onnx | 基于源码训练5000轮模型，用于陈总展示 |
| rl\_walk\_leg.onnx                     | 源码部署仓库携带腿部12dof模型    |
| rl\_walk\_leg\_shoulder.onnx           | 源码部署仓库携带腿肩协同模型       |
