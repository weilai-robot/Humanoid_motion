# policy 模型台账

- 外层 = policy/ 根目录中的模型为活跃模型
- 内层（aichive) = 历史模型

## 活跃模型

| 文件名                            | 来源                                                                                                                                                                                                                   |
| ------------------------------ | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| rl\_walk\_leg\_20260901\_step7500\.onnx | v5\_copy model\_7500 血统（训练 run：wyn\_7500\_add\_gait\_&\_ankle\_roll\_DR = v3b model\_5000 续训，yaw\_hold gate 修复版）；真机验证 20260831/0901 零跌倒（见 20260902 README）；原 rl\_walk\_leg\_7500\_new\.onnx 改名，md5 79A15F3C；真机 rl\_x1.yaml 在用（yaml 引用名待更新） |
| rl\_walk\_leg\_20260818\_step5999\.onnx | 待补（原 rl\_walk\_leg\_5999\.onnx 改名，md5 9F01E3AD；疑源码重训线，与 `test_logs/20260814/源码重训模型/model_5999.pt` 关系待确认） |

## 历史模型

| 文件名                        | 来源                                                                                                    |
| -------------------------- | ----------------------------------------------------------------------------------------------------- |
| rl\_walk\_leg\_20260723\_step5000\.onnx | 待补（原 rl\_walk\_leg\.onnx 改名，md5 707F9980；疑 v3b model\_5000 = q3 阶段1 交付 `exp_add_all_parameter_new_urdf_damping_0_reward_v1` 从零 5000，待确认；原被 rl\_x1\_sim\.yaml + build\.sh 引用） |
| rl\_walk\_leg\.onnx | 待补（原 rl\_walk\_leg\_source\.onnx 改名，md5 00CD00F2；名含 source，疑 exp\_010 源码基线模型，待确认） |
| rl\_walk\_leg\_shoulder\.onnx | 待补（2026-07-21 导出，shoulder 变体 policy；yaml 中仍有 rl\_walk\_leg\_shoulder controller 引用） |
