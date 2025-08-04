# Robot Tool - 标准化机器人控制器接口

一个统一的机器人控制器接口包，支持多种机器人平台，自动处理逆运动学、插值计算和运动规划。

## 📋 前置要求

### Conda环境配置

本包依赖Conda环境中的`pinocchio`库进行逆运动学计算。请按以下步骤配置环境：

#### 1. 激活Conda

**临时激活（当前终端生效）：**
```bash
source ~/miniconda3/bin/activate
```

**永久激活（每次启动终端自动加载）：**
```bash
echo "source ~/miniconda3/bin/activate" >> ~/.bashrc
source ~/.bashrc
```

#### 2. 创建conda环境

```bash
conda create -n humble python=3.10.13
```

> **重要提示：** 这个Python版本需要跟你当前系统的Python版本一模一样（使用`python3 --version`查看系统的Python版本）

#### 3. 进入humble环境

```bash
conda activate humble
```

进入后终端的开头会显示`(humble)`，表示进入成功。

#### 4. 安装依赖库

```bash
conda install pinocchio casadi -c conda-forge
```

这将安装逆运动学计算所需的`pinocchio`库和优化求解器`casadi`。

## 🚀 快速开始

### 1. 配置文件设置

在 `config/robot_config.yaml` 中配置您的机器人参数：

```yaml
jaka_controller:
  ros__parameters:
    # URDF文件路径（必须）
    urdf_path: "/path/to/your/robot.urdf"
    
    # 手臂前缀（必须）- 用于区分左右臂
    arm_prefix: ["r", "l"]  # 右臂前缀'r'，左臂前缀'l'
    
    # 末端执行器链接名称（非必须）- URDF中的末端链接名
    end_effector_link_name: ["rt", "lt"]  # 右臂末端'rt'，左臂末端'lt'
    
    # 关节数量（必须）
    num_joints: 7  # 每个手臂的关节数量
    
    # 可视化选项（可选）
    visualize: false
    
    # 伺服发布频率（可选）
    servo_publish_rate: 125.0  # Hz
    
    # 插值参数（可选）
    interpolation:
      default_step: 0.05  # 默认插值步长（米）
      max_points: 1000    # 最大插值点数
      default_type: "LINEAR"  # 默认插值类型
    
    # 运动参数（可选）
    motion:
      default_velocity: 0.1      # 默认速度
      default_acceleration: 0.1  # 默认加速度

    
    # 碰撞检测
    enable_collision_detection: true
    igno_coll_pairs: "[['l6_0','l7_0'],['r6_0','r7_0']]" ### 节卡机械臂的l6_0和l7_0，r6_0和r7_0在pinocchio碰撞检测时会莫名判断出碰撞，所以需要忽略

```

### 2. 实现您的机器人控制器

创建您的机器人控制器类，继承 `BaseRobotController`：
有四个必须重写的函数

```python
from robot_tool.base_robot_controller import BaseRobotController
from robot_tool.data_struct import Arm
from typing import List

class MyRobotController(BaseRobotController):
    def __init__(self):
        # 基类会自动从配置文件读取参数
        super().__init__()
        # 初始化您的机器人连接
        self.robot = MyRobot()
    
    # ==================== 必须实现的4个函数 ====================
    
    def get_current_pose(self, arm=Arm.right) -> List[float]:
        """获取当前末端执行器位姿"""
        # 返回值格式：[x, y, z, rx, ry, rz]
        # x, y, z: 位置（米）
        # rx, ry, rz: 旋转（弧度）
        if arm == Arm.right:
            pose = self.robot.get_right_pose()
            return [pose.x, pose.y, pose.z, pose.rx, pose.ry, pose.rz]
        else:
            pose = self.robot.get_left_pose()
            return [pose.x, pose.y, pose.z, pose.rx, pose.ry, pose.rz]
    
    def get_current_joint_angles(self, arm=Arm.right) -> List[float]:
        """获取当前关节角度"""
        # 返回值格式：[j1, j2, j3, j4, j5, j6, j7]（弧度）
        if arm == Arm.right:
            return self.robot.get_right_joints()  # 返回列表格式
        else:
            return self.robot.get_left_joints()
    

    
    def move_to_joint_angles(self, joint_angles, velocity, acceleration, arm=Arm.right, block=False):
        """关节空间移动"""
        # joint_angles: {Arm.left: [angles], Arm.right: [angles]}
        if arm == Arm.left:
            self.robot.move_left_arm(joint_angles[Arm.left], velocity, acceleration)
        elif arm == Arm.right:
            self.robot.move_right_arm(joint_angles[Arm.right], velocity, acceleration)
        elif arm == Arm.both:
            self.robot.move_both_arms(joint_angles[Arm.left], joint_angles[Arm.right], velocity, acceleration)
        
        if block:
            self.robot.wait_for_motion_complete()
        return True
    
    """
    如果不需要伺服运动，则不需要这个函数
    """
    def servo_move_to_joint_angles(self, joint_angles, velocity=None, acceleration=None, arm=Arm.right):
        """伺服移动（用于插值运动）"""
        # joint_angles: {Arm.left: [[angles], [angles], ...], Arm.right: [[angles], [angles], ...]}
        if isinstance(joint_angles, dict):
            left_sequence = joint_angles.get(Arm.left, [])
            right_sequence = joint_angles.get(Arm.right, [])
            
            for i in range(len(left_sequence)):
                self.robot.servo_move(left_sequence[i], right_sequence[i])
        return True
```

### 3. 使用示例

```python
from robot_tool.data_struct import MoveToRequest
from robot_tool.interpolate import InterpolateType

# 创建控制器
controller = MyRobotController()

# 创建移动请求
request = MoveToRequest(
    target_pose=[0.5, 0.2, 0.5, 0.0, 0.0, 0.0],  # [x, y, z, rx, ry, rz]
    arm=Arm.left,
    interpolate=True,  # 使用插值（伺服模式）
    interpolate_type=InterpolateType.LINEAR,
    step=0.05,  # 插值步长
    velocity=0.1,
    acceleration=0.1,
    block=True
)

# 执行移动
controller.move_to_tag(request)
```

## 📋 关键要点

### 配置文件参数说明

| 参数 | 类型 | 说明 | 示例 |
|------|------|------|------|
| `urdf_path` | string | URDF文件路径 | `/path/to/robot.urdf` |
| `arm_prefix` | list | 手臂前缀 | `["r", "l"]` |
| `end_effector_link_name` | list | 末端链接名 | `["rt", "lt"]` |
| `num_joints` | int | 关节数量 | `7` |
| `servo_publish_rate` | float | 伺服频率 | `125.0` |
| `interpolation.default_step` | float | 插值步长 | `0.05` |

### 必须实现的4个函数

1. **`get_current_pose(arm)`** - 返回 `[x, y, z, rx, ry, rz]` 列表
2. **`get_current_joint_angles(arm)`** - 返回 `[j1, j2, j3, j4, j5, j6, j7]` 列表
3. **`move_to_joint_angles(...)`** - 关节空间移动
4. **`servo_move_to_joint_angles(...)`** - 伺服移动

### 数据格式要求

- **位置单位**：米（m）
- **旋转单位**：弧度（rad）
- **关节角度单位**：弧度（rad）
- **关节速度单位**：弧度/秒（rad/s）

## 🚀 启动方式

```bash
# 使用launch文件启动
ros2 launch robot_tool jaka_controller.launch.py

# 或者直接运行节点
ros2 run robot_tool jaka_controller_node
```

## ✨ 特性

- **自动单位转换**：支持毫米/米、度/弧度的自动转换
- **智能插值**：自动生成平滑运动轨迹
- **逆运动学**：自动计算关节角度
- **双臂支持**：支持单臂和双臂操作
- **标准化接口**：统一的API，易于扩展 