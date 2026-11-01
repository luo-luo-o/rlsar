# dog 开发记录

## 常用命令

```bash
colcon build --merge-install --symlink-install

source install/setup.sh

ros2 launch rl_sar gazebo.launch.py rname:=dog

ros2 run rl_sar rl_sim
```

## 模型调整

### 简单总结
简单调整，但是模型策略训练的时候似乎还是有不少问题。不确定是策略模型的参数的问题还是机械结构模型的问题。

### log

1. 重新调整 urdf 文件的关节角度参数，修复原本模型的角度问题。
2. 调整腿部的碰撞体为圆柱体拟合，避免直接使用原有stl模型参数，适配仿真器，优化训练效果

### TODO 

- 考虑将足部碰撞体修改为胶囊体拟合

## Isaac 训练

基于 https://github.com/fan-ziqi/robot_lab 框架

### log

1. 参考 go2 多次 dog 调整 reward 参数训练，考虑 stand - slow_walk 的训练流程

### TODO

1. 优化 reward 参数
2. 优化调参工作流
3. 优化 输入输出 维度的设计，适配 rl-sar 框架

## sim-to-sim

基于 https://github.com/fan-ziqi/rl_sar 框架

### log

1. 将机器狗模型设计导入到 rl_sar_zoo （根据 READNE.md 的要求）
2. 尝试 gazebo 仿真，实测可以正常加载 urdf 模型，但是加载 policy 存在问题，无法正常操控（怀疑是模型本身的问题）

### TODO

1. 检查关节输入输出是否与之前的设计对应
2. 继续优化 isaac 参数训练策略模型

## sim-to-real

TODO