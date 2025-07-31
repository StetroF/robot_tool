# 机器人控制器标准化接口使用指南

## 概述

本指南介绍如何为新的机器人实现标准化的控制器接口。通过继承 `BaseRobotController` 类，您可以轻松为任何机器人创建统一的控制器接口。**现在只需要实现3个核心方法！**

## 1. 参数配置

### 初始化参数说明

```python
class MyRobotController(BaseRobotController):
    def __init__(self, urdf_path, arm_prefix, end_effector_link_name, num_joints=6, visualize=False):
        super().__init__(urdf_path, arm_prefix, end_effector_link_name, num_joints, visualize)
```

**参数说明：**
- `urdf_path` (str): URDF文件路径，用于逆运动学计算
- `arm_prefix` (List[str]): 手臂前缀列表，格式为 `['right_prefix', 'left_prefix']`
  - 例如：`['r', 'l']` 或 `['right_', 'left_']`
- `end_effector_link_name` (List[str]): 末端执行器链接名称列表，格式为 `['right_end_effector', 'left_end_effector']`
  - 例如：`['rt', 'lt']` 或 `['right_gripper', 'left_gripper']`
- `num_joints` (int): 每个手臂的关节数量，默认为6
- `visualize` (bool): 是否启用可视化，默认为False

**示例配置：**
```python
# JAKA机器人配置
urdf_path = "/path/to/dual_arm.urdf"
arm_prefix = ['r', 'l']  # 右臂前缀'r'，左臂前缀'l'
end_effector_link_name = ['rt', 'lt']  # 右臂末端'rt'，左臂末端'lt'
num_joints = 7  # 7自由度机械臂

# 其他机器人配置示例
arm_prefix = ['right_', 'left_']  # 使用完整前缀
end_effector_link_name = ['right_gripper', 'left_gripper']  # 使用描述性名称
```

## 2. 必须实现的抽象方法（仅3个！）

### 2.1 get_current_pose(arm=Arm.right) -> List[float]

**功能：** 获取当前末端执行器位姿

**参数：**
- `arm`: 手臂选择，`Arm.left` 或 `Arm.right`

**返回值：** 必须是 `List[float]` 格式，包含6个元素 `[x, y, z, rx, ry, rz]`

**示例实现：**
```python
def get_current_pose(self, arm=Arm.right) -> List[float]:
    if arm == Arm.right:
        pose = self.robot.get_right_pose()
        # 确保返回 [x, y, z, rx, ry, rz] 格式
        return [pose.x, pose.y, pose.z, pose.rx, pose.ry, pose.rz]
    else:
        pose = self.robot.get_left_pose()
        return [pose.x, pose.y, pose.z, pose.rx, pose.ry, pose.rz]
```

### 2.2 get_current_joint_angles(arm=Arm.right)

**功能：** 获取当前关节角度

**参数：**
- `arm`: 手臂选择，`Arm.left` 或 `Arm.right`

**返回值：** 可以是任何格式，基类会自动标准化为 `List[float]`

**示例实现：**
```python
def get_current_joint_angles(self, arm=Arm.right):
    if arm == Arm.right:
        # 返回任何格式，基类会自动处理
        return self.robot.get_right_joints()  # 可以是列表、numpy数组、自定义对象等
    else:
        return self.robot.get_left_joints()

# 支持的返回格式示例：
# 1. 列表格式
return [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0]

# 2. numpy数组
return numpy.array([1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0])

# 3. ROS2消息对象
return self.robot_state.joint_pos_right  # JointValue对象

# 4. 字典格式
return {'joint1': 1.0, 'joint2': 2.0, ...}
```

### 2.3 get_current_joint_velocities(arm=Arm.right)

**功能：** 获取当前关节速度

**参数：**
- `arm`: 手臂选择，`Arm.left` 或 `Arm.right`

**返回值：** 可以是任何格式，基类会自动标准化为 `List[float]`

**示例实现：**
```python
def get_current_joint_velocities(self, arm=Arm.right):
    if arm == Arm.right:
        return self.robot.get_right_velocities()
    else:
        return self.robot.get_left_velocities()
```

## 3. 可选重写的方法

### 3.1 move_to_joint_angles() - 关节空间移动

**功能：** 移动到指定关节角度（关节空间移动）

**何时需要重写：** 当您的机器人需要特殊的关节空间移动实现时

**默认行为：** 基类会抛出 `NotImplementedError`，子类必须实现

**示例实现：**
```python
def move_to_joint_angles(self, joint_angles, velocity, acceleration, arm=Arm.right, block=False):
    if arm == Arm.left:
        # 移动左臂
        self.robot.move_left_arm(joint_angles[Arm.left], velocity, acceleration)
    elif arm == Arm.right:
        # 移动右臂
        self.robot.move_right_arm(joint_angles[Arm.right], velocity, acceleration)
    elif arm == Arm.both:
        # 同时移动双臂
        self.robot.move_both_arms(joint_angles[Arm.left], joint_angles[Arm.right], velocity, acceleration)
    
    if block:
        # 等待运动完成
        self.robot.wait_for_motion_complete()
    
    return True
```

### 3.2 servo_move_to_joint_angles() - 伺服移动

**功能：** 伺服移动到指定关节角度（用于插值运动）

**何时需要重写：** 当您的机器人需要特殊的伺服移动实现时

**默认行为：** 基类会抛出 `NotImplementedError`，子类必须实现

**示例实现：**
```python
def servo_move_to_joint_angles(self, joint_angles, velocity=None, acceleration=None, arm=Arm.right):
    if isinstance(joint_angles, dict):
        # 处理字典格式的多组关节角度
        if Arm.left in joint_angles and Arm.right in joint_angles:
            left_sequence = joint_angles[Arm.left]
            right_sequence = joint_angles[Arm.right]
            
            for i in range(len(left_sequence)):
                # 发送每组关节角度
                self.robot.servo_move(left_sequence[i], right_sequence[i])
    else:
        # 处理其他格式
        self.robot.servo_move(joint_angles)
    
    return True
```

## 4. 完整实现示例

```python
from base_robot_controller import BaseRobotController
from data_struct import Arm, MoveToRequest
from interpolate import InterpolateType
import numpy as np

class MyRobotController(BaseRobotController):
    def __init__(self, urdf_path, arm_prefix, end_effector_link_name, num_joints=6, visualize=False):
        super().__init__(urdf_path, arm_prefix, end_effector_link_name, num_joints, visualize)
        
        # 初始化您的机器人连接
        self.robot = MyRobot()  # 您的机器人类
        self.robot.connect()
    
    # ==================== 必须实现的3个核心方法 ====================
    
    def get_current_pose(self, arm=Arm.right) -> List[float]:
        """获取当前末端执行器位姿"""
        if arm == Arm.right:
            pose = self.robot.get_right_pose()
            return [pose.x, pose.y, pose.z, pose.rx, pose.ry, pose.rz]
        else:
            pose = self.robot.get_left_pose()
            return [pose.x, pose.y, pose.z, pose.rx, pose.ry, pose.rz]
    
    def get_current_joint_angles(self, arm=Arm.right):
        """获取当前关节角度"""
        if arm == Arm.right:
            return self.robot.get_right_joint_angles()  # 返回任何格式
        else:
            return self.robot.get_left_joint_angles()
    
    def get_current_joint_velocities(self, arm=Arm.right):
        """获取当前关节速度"""
        if arm == Arm.right:
            return self.robot.get_right_joint_velocities()
        else:
            return self.robot.get_left_joint_velocities()
    
    # ==================== 可选重写的方法 ====================
    
    def move_to_joint_angles(self, joint_angles, velocity, acceleration, arm=Arm.right, block=False):
        """关节空间移动"""
        if arm == Arm.left:
            self.robot.move_left_arm(joint_angles[Arm.left], velocity, acceleration)
        elif arm == Arm.right:
            self.robot.move_right_arm(joint_angles[Arm.right], velocity, acceleration)
        elif arm == Arm.both:
            self.robot.move_both_arms(joint_angles[Arm.left], joint_angles[Arm.right], velocity, acceleration)
        
        if block:
            self.robot.wait_for_motion_complete()
        
        return True
    
    def servo_move_to_joint_angles(self, joint_angles, velocity=None, acceleration=None, arm=Arm.right):
        """伺服移动"""
        # 实现伺服移动逻辑
        self.robot.servo_move(joint_angles, arm)
        return True

# 使用示例
def main():
    # 1. 创建控制器实例
    controller = MyRobotController(
        urdf_path="/path/to/your/robot.urdf",
        arm_prefix=['right_', 'left_'],
        end_effector_link_name=['right_gripper', 'left_gripper'],
        num_joints=6
    )
    
    # 2. 获取当前状态（自动标准化）
    left_angles = controller._get_standardized_joint_angles(Arm.left)
    right_angles = controller._get_standardized_joint_angles(Arm.right)
    current_pose = controller.get_current_pose(Arm.left)
    
    print(f"左臂关节角度: {left_angles}")
    print(f"右臂关节角度: {right_angles}")
    print(f"左臂当前位姿: {current_pose}")
    
    # 3. 创建移动请求
    request = MoveToRequest(
        target_pose=[0.5, 0.2, 0.5, 0.0, 0.0, 0.0],  # [x, y, z, rx, ry, rz]
        arm=Arm.left,
        interpolate=True,  # 使用插值（伺服模式）
        interpolate_type=InterpolateType.LINEAR,
        step=0.05,  # 插值步长
        velocity=0.1,  # 速度
        acceleration=0.1,  # 加速度
        block=True  # 阻塞等待完成
    )
    
    # 4. 执行移动（基类自动处理所有复杂逻辑）
    controller.move_to_tag(request)
    
    # 5. 直接关节空间移动
    joint_angles = {
        Arm.left: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        Arm.right: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    }
    controller.move_to_joint_angles(joint_angles, velocity=0.1, acceleration=0.1, arm=Arm.both, block=True)

if __name__ == "__main__":
    main()
```

## 5. 基类提供的功能

### 5.1 自动处理的功能
- **数据格式标准化**：自动处理各种关节角度数据格式
- **逆运动学计算**：自动计算位姿到关节角度的转换
- **插值计算**：自动生成平滑的运动轨迹
- **运动模式选择**：根据请求自动选择伺服模式或关节空间模式

### 5.2 可用的辅助方法
- `_get_standardized_joint_angles(arm)` - 获取标准化关节角度
- `_get_standardized_joint_velocities(arm)` - 获取标准化关节速度
- `_standardize_joint_angles(data)` - 手动标准化数据

## 6. 关键要点

### 6.1 最小实现
**只需要实现3个方法：**
1. `get_current_pose()` - 返回位姿
2. `get_current_joint_angles()` - 返回关节角度（任何格式）
3. `get_current_joint_velocities()` - 返回关节速度（任何格式）

### 6.2 数据格式灵活性
- `get_current_joint_angles()` 和 `get_current_joint_velocities()` 可以返回任何格式
- 基类会自动处理格式转换
- `get_current_pose()` 必须返回 `[x, y, z, rx, ry, rz]` 格式

### 6.3 运动控制
- 基类提供默认的运动实现
- 子类可以重写 `move_to_joint_angles()` 和 `servo_move_to_joint_angles()` 以提供自定义实现
- 如果不重写，基类会抛出异常提示需要实现

### 6.4 使用方式
```python
# 创建控制器
controller = MyRobotController(urdf_path, arm_prefix, end_effector_link_name)

# 直接使用高级接口
controller.move_to_tag(request)  # 基类处理所有复杂逻辑

# 或使用低级接口
controller.move_to_joint_angles(joint_angles, velocity, acceleration, arm, block)
```

## 7. 优势总结

### 7.1 极简实现
- **只需实现3个核心方法**
- 基类处理所有复杂逻辑
- 代码量减少80%以上

### 7.2 高度灵活
- 支持任何数据格式
- 自动格式转换
- 可选的运动实现

### 7.3 功能完整
- 自动插值计算
- 自动逆运动学
- 支持单臂和双臂操作

通过这种设计，您可以轻松为任何机器人实现标准化的控制器接口，同时保持代码的简洁性和可维护性！ 