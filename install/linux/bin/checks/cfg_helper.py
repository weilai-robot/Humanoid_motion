#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""yaml 字段查询小工具，供 run_main.sh preflight 与 rt_verify.sh 复用。
依赖 PyYAML（ROS2 humble 环境自带；缺失请 pip install pyyaml 或 apt install python3-yaml）。

子命令：
  get <yaml> <dotted.path>           打印标量值（如 ethercat.rt_priority）
  module-cfg <top_cfg> <ModuleName>  打印该 module 的 cfg_file_path
  onnx-models <rl_yaml>              遍历 controllers 的 onnx_conf.policy_file，每行一个

退出码：0=有值 1=字段缺失 2=文件/YAML 错 3=缺 PyYAML 64=用法错
"""
import sys

try:
    import yaml
except ImportError:
    sys.stderr.write("[cfg_helper] 缺少 PyYAML，请 pip install pyyaml 或 apt install python3-yaml\n")
    sys.exit(3)


def load_yaml(path):
    try:
        with open(path, "r", encoding="utf-8") as f:
            return yaml.safe_load(f)
    except FileNotFoundError:
        sys.stderr.write("[cfg_helper] 文件不存在: %s\n" % path)
        sys.exit(2)
    except yaml.YAMLError as e:
        sys.stderr.write("[cfg_helper] YAML 解析失败 %s: %s\n" % (path, e))
        sys.exit(2)


def walk(obj, dotted):
    cur = obj
    for key in dotted.split("."):
        if isinstance(cur, dict) and key in cur:
            cur = cur[key]
        else:
            return None
    return cur


def cmd_get(args):
    yaml_path, dotted = args
    data = load_yaml(yaml_path)
    val = walk(data, dotted)
    if val is None:
        sys.exit(1)
    print(val)
    sys.exit(0)


def cmd_module_cfg(args):
    top_cfg, module_name = args
    data = load_yaml(top_cfg)
    modules = walk(data, "aimrt.module.modules")
    if not isinstance(modules, list):
        sys.exit(1)
    for m in modules:
        if isinstance(m, dict) and m.get("name") == module_name:
            p = m.get("cfg_file_path")
            if p:
                print(p)
                sys.exit(0)
            break
    sys.exit(1)


def cmd_onnx_models(args):
    (rl_cfg,) = args
    data = load_yaml(rl_cfg)
    controllers = walk(data, "controllers")
    # rl_x1.yaml 的 controllers 是 map（pd_idle:/pd_keep:/... 各为 key），
    # 兼容 list 写法，遍历取 onnx_conf.policy_file。
    found = False
    iterable = controllers.values() if isinstance(controllers, dict) \
        else (controllers if isinstance(controllers, list) else [])
    for cdef in iterable:
        if not isinstance(cdef, dict):
            continue
        p = walk(cdef, "onnx_conf.policy_file")
        if p:
            print(p)
            found = True
    sys.exit(0 if found else 1)


USAGE = "用法: cfg_helper.py {get <yaml> <dotted> | module-cfg <top_cfg> <Module> | onnx-models <rl_yaml>}\n"


def main():
    args = sys.argv[1:]
    if not args:
        sys.stderr.write(USAGE)
        sys.exit(64)
    cmd = args[0]
    rest = args[1:]
    try:
        if cmd == "get" and len(rest) == 2:
            cmd_get(rest)
        elif cmd == "module-cfg" and len(rest) == 2:
            cmd_module_cfg(rest)
        elif cmd == "onnx-models" and len(rest) == 1:
            cmd_onnx_models(rest)
        else:
            sys.stderr.write(USAGE)
            sys.exit(64)
    except SystemExit:
        raise
    except Exception as e:
        sys.stderr.write("[cfg_helper] 错误: %s\n" % e)
        sys.exit(2)


if __name__ == "__main__":
    main()
