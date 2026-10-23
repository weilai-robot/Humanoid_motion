#!/bin/bash
set -euo pipefail

# X1 BeyondMimic v12 154维舞蹈：复用颈部保持、ROS 环境和 EtherCAT 权限处理。
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export X1_CONTROL_CONFIG=./cfg/x1_cfg_dance.yaml
exec bash "${SCRIPT_DIR}/run_real_nav.sh"
