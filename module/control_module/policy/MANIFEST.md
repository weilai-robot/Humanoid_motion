# policy 模型台账

- 外层 = policy/ 根目录中的模型为当前或可回退模型；是否加载由 YAML `policy_file` 决定
- 内层（aichive) = 历史模型

## 活跃模型

| 文件名                                    | 来源                                       |
| -------------------------------------- | ---------------------------------------- |
| rl\_walk\_leg\_20260901\_step7500.onnx | 参数辨识+低抬步态模型，可实现前进、停止、横移、转向               |
| rl\_walk\_leg\_20260818\_step5999.onnx | 基于源码训练5999轮模型，可实现前进、后退、停止、横移、转向，用于视频拍摄模型 |

## AMP 模型

| 文件 | 状态与来源 |
|---|---|
| `x1_amp_soup59c_50hz.onnx` | `rl_walk_amp` 当前使用；从 X1_29_AMP 的 `acceptance/v59c_eval/soup59c_50.policy.npz` 导出，50 Hz、29DOF；SHA256 `3e09b76f1e38085a2093abccc1e6f80c86751c9758f7f21d0f497163c99ebd00` |
| `x1_amp_v46s_soup.onnx` | 上一版 50 Hz AMP 部署模型，保留作回退 |

## 历史模型

| 文件名                                    | 来源                   |
| -------------------------------------- | -------------------- |
| rl\_walk\_leg\_20260723\_step5000.onnx | 基于源码训练5000轮模型，用于陈总展示 |
| rl\_walk\_leg.onnx                     | 源码部署仓库携带腿部12dof模型    |
| rl\_walk\_leg\_shoulder.onnx           | 源码部署仓库携带腿肩协同模型       |

## BeyondMimic X1 v11 舞蹈

专用 `rl_x1_dance.yaml` 加载，不改变 AMP 默认模型。

| 文件 | 来源与用途 |
| --- | --- |
| `dance_x1_v11_4999.bin` | `dance_x1_v11_model_4999.pt`，160→512→256→128→29，ELU，`X1DNCE2`；辨识 PD/armature；SHA256 `4b146822e4e37dfc1c73e35b5712d8893aacfcd82d5fb82e59d47f66b070d239` |
| `dance_x1_motion.bin` | `dance_x1_train.npz`，647 帧、50 Hz，按 PhysX 关节名称重排；SHA256 `dd3376f615ee86daf6df4f1b13972942376d5a2275e904c4efb1674691bf884d` |
| `dance_x1_v11_4999.json` | checkpoint / robot config / motion 来源 hash、逐关节参数及导出误差 |

`tools/export_beyondmimic_x1.py` 从训练 `robots/x1.py` 读取参数；归一化使用 `_std + 0.01`。
Mac 已执行离线验证；Ubuntu C++/ROS/DCU 与真机验收未执行。参见父仓库 `doc/BeyondMimic_X1_舞蹈部署.md`，当前按键是 A 零位 → B stand → Guide（准备位 → 连续就绪等待 → 自动播放）。准备位选训练 default_q，实际到位校验与模型资产一致，播放仍从参考帧 0 开始。历史手动等待探针不等同于当前自动接管；当前结果见 `doc/analysis/x1_dance_v11_20261017/guide_auto_start/`。
