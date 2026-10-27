#!/bin/bash
# run_real_nav.sh — 薄 wrapper, 委托 run_main.sh（无手柄导航配置 x1_cfg_real_nav.yaml）
# 原颈部/setcap/local_setup 逻辑已收归 run_main.sh 统一维护（三件套+颈部+setcap 只此一份）
# 模式切换须用外部脚本（手柄不可用）：../scripts/switch_x1_mode.sh ready
# 用法: bash run_real_nav.sh [run_main.sh 的任意参数，如 --check-only]
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}"
exec bash ./run_main.sh --cfg ./cfg/x1_cfg_real_nav.yaml "$@"
