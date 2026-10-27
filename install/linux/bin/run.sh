#!/bin/bash
# run.sh — 薄 wrapper, 委托 run_main.sh（带手柄配置 x1_cfg.yaml）
# 原颈部/setcap/local_setup 逻辑已收归 run_main.sh 统一维护（三件套+颈部+setcap 只此一份）
# 用法: bash run.sh [run_main.sh 的任意参数，如 --check-only]
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}"
exec bash ./run_main.sh --cfg ./cfg/x1_cfg.yaml "$@"
