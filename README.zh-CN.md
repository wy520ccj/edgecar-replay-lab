# EdgeCar Replay Lab

面向资源受限智能车辆的确定性回放与安全验证实验室。

这是一个从零编写的 C++17 软件在环参考项目：重复运行同一场景，注入延迟、丢帧、时间戳过期和感知失败，检查安全监督器是否及时降级/停车，并自动生成 JSONL 遥测和 HTML 报告。

## 一键运行

```powershell
./scripts/run_demo.ps1
```

脚本会配置、编译、运行测试，执行全部 12 个演示场景，并生成 `out/demo/index.html`。Linux/macOS 可运行 `./scripts/run_demo.sh`。

## 核心亮点

- 确定性回放：固定随机种子和场景输入，重复运行得到相同状态轨迹。
- 安全监督：`NORMAL / DEGRADED / SAFE_STOP / LATCHED_STOP` 四级状态，统一限速、转角和变化率。
- 故障注入：延迟、丢帧、车道丢失、检测器失败、坏时间戳、超速和转向突变。
- 可观察性：每帧记录原始指令、最终指令、阶段耗时、安全状态和原因。
- 可移植性：核心不依赖车载 SDK，可在 x86-64 构建，并做 ARM64 交叉编译。

## 真实性边界

本仓库是软件在环/回放验证工具，不是道路自动驾驶系统，也不声称通过车规认证。默认执行器是空执行器，不访问串口、电机或真实车辆。历史课程工程、EdgeBoard 镜像、受限源码、比赛数据集、第三方视频和来源不明模型均未上传。

## 目录

```text
include/edgecar/   公共接口与数据结构
src/               回放、场景、安全、遥测实现
apps/              replay / bench / report 命令
tests/             单元测试与故障场景测试
scenarios/         12 个可复现 YAML 场景
docs/              架构与评测说明
```

原创代码采用 Apache-2.0，第三方声明见 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。
