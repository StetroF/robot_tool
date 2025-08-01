from robot_tool.ik import Arm_IK
from abc import ABC, abstractmethod
from robot_tool.interpolate import PoseInterpolator
from typing import List, Dict, Tuple, Optional, Union
import numpy as np
import math
from pydantic import BaseModel
from typing import Literal
from enum import Enum
from robot_tool.interpolate import InterpolateType
class Arm:
    left = 0
    right = 1
    both = -1

class MoveToRequest(BaseModel):
    target_pose: list[float]    #### [x,y,z,rx,ry,rz]
    arm: int = Arm.right
    interpolate: bool = False
    interpolate_type: int = InterpolateType.LINEAR
    step: float = 0.01  ## 插值步长
    velocity: float = 20.0  ##°/s
    acceleration: float = 20.0  ##°/s^2
    block: bool = False ##运动时是否阻塞
    



class BaseRobotController(ABC):
    def __init__(self, urdf_path, arm_prefix: List[str], 
                 end_effector_link_name: List[str], num_joints=6, visualize=False):
        self.R_inverse_solution = Arm_IK(urdf_path, arm_prefix[0], 
                                       end_effector_link_name[0], 
                                       num_joints=num_joints, 
                                       visualize=visualize)
        self.L_inverse_solution = Arm_IK(urdf_path, arm_prefix[1], 
                                       end_effector_link_name[1], 
                                       num_joints=num_joints, 
                                       visualize=visualize)
        self.interpolator = PoseInterpolator()
        self.num_joints = num_joints
    
    def move_to_tag(self, request: MoveToRequest):
        """
        移动到目标位姿（自动根据插值决定使用伺服模式）
        Args:
            request: MoveToRequest对象，包含目标位姿、手臂选择、插值参数等
        """
        target_pose = request.target_pose
        arm = request.arm
        current_pose = self.get_current_pose(arm)
        
        if request.interpolate:
            # 插值模式：使用伺服控制，计算多组关节角度
            interpolated_poses = self.interpolate_pose(
                current_pose, target_pose, request.step, request.interpolate_type)
            self.node.get_logger().info(f'插值数量: {len(interpolated_poses)}')
            # 计算所有插值点的关节角度序列
            joint_angles_sequence = self._calculate_interpolated_joint_angles(interpolated_poses, arm)
            
            # 伺服模式移动
            self._execute_servo_motion(joint_angles_sequence, arm)
        else:
            # 非插值模式：使用普通移动，只计算一组关节角度
            angles = self._calculate_single_pose_joint_angles(target_pose, arm)
            self._execute_joint_motion(angles, request.velocity, request.acceleration, arm, request.block)
    
    def _calculate_interpolated_joint_angles(self, interpolated_poses, arm) -> Dict[Arm, List[List[float]]]:
        """计算插值位姿序列对应的关节角度序列"""
        joint_angles_sequence = {
            Arm.left: [],
            Arm.right: []
        }
        
        # 获取初始关节角度作为第一次IK求解的参考
        init_left_angles = None
        init_right_angles = None
        
        if arm in (Arm.left, Arm.both):
            init_left_angles = self._get_standardized_joint_angles(Arm.left)
        
        if arm in (Arm.right, Arm.both):
            init_right_angles = self._get_standardized_joint_angles(Arm.right)
        
        for i, pose in enumerate(interpolated_poses):
            angles = self._calculate_single_pose_joint_angles_with_init(
                pose, arm, init_left_angles, init_right_angles)
            joint_angles_sequence[Arm.left].append(angles[Arm.left])
            joint_angles_sequence[Arm.right].append(angles[Arm.right])
            
            # 更新下一次IK求解的初始角度
            if arm in (Arm.left, Arm.both):
                init_left_angles = angles[Arm.left]
            if arm in (Arm.right, Arm.both):
                init_right_angles = angles[Arm.right]
        
        # 根据手臂选择，只保留需要的关节角度序列
        if arm == Arm.left:
            # 只移动左臂，右臂保持当前位置
            right_angles_list = self._get_standardized_joint_angles(Arm.right)
            joint_angles_sequence[Arm.right] = [right_angles_list] * len(interpolated_poses)
        elif arm == Arm.right:
            # 只移动右臂，左臂保持当前位置
            left_angles_list = self._get_standardized_joint_angles(Arm.left)
            joint_angles_sequence[Arm.left] = [left_angles_list] * len(interpolated_poses)
        
        return joint_angles_sequence
    
    def _execute_servo_motion(self, joint_angles_sequence, arm):
        """执行伺服运动"""
        # 基类提供默认实现，子类可以重写
        self.servo_move_to_joint_angles(joint_angles_sequence, arm=arm)
    
    def _execute_joint_motion(self, joint_angles, velocity, acceleration, arm, block):
        """执行关节运动"""
        # 基类提供默认实现，子类可以重写
        self.move_to_joint_angles(joint_angles, velocity, acceleration, arm=arm, block=block)
    
    def _calculate_single_pose_joint_angles(self, pose, arm) -> Dict[Arm, List[float]]:
        """计算单一位姿对应的关节角度"""
        angles = {
            Arm.left: [0.0] * self.num_joints,
            Arm.right: [0.0] * self.num_joints
        }
        
        # 将位姿转换为4x4齐次变换矩阵
        target_matrix = self.pose_to_matrix(pose)
        
        if arm in (Arm.right, Arm.both):
            # 使用标准化的方法获取当前关节角度
            init_joint_angles = self._get_standardized_joint_angles(Arm.right)
            result = self.R_inverse_solution.ik_fun(target_matrix, init_joint_angles=init_joint_angles)
            if result[0] is not None:  # 检查是否成功
                angles[Arm.right] = result[0].tolist() if hasattr(result[0], 'tolist') else list(result[0])
        if arm in (Arm.left, Arm.both):
            # 使用标准化的方法获取当前关节角度
            init_joint_angles = self._get_standardized_joint_angles(Arm.left)
            result = self.L_inverse_solution.ik_fun(target_matrix, init_joint_angles=init_joint_angles)
            if result[0] is not None:  # 检查是否成功
                angles[Arm.left] = result[0].tolist() if hasattr(result[0], 'tolist') else list(result[0])
            
        return angles
    
    def _calculate_single_pose_joint_angles_with_init(self, pose, arm, init_left_angles=None, init_right_angles=None) -> Dict[Arm, List[float]]:
        """计算单一位姿对应的关节角度（支持传入初始关节角度）"""
        angles = {
            Arm.left: [0.0] * self.num_joints,
            Arm.right: [0.0] * self.num_joints
        }
        
        # 将位姿转换为4x4齐次变换矩阵
        target_matrix = self.pose_to_matrix(pose)
        
        if arm in (Arm.right, Arm.both):
            # 使用传入的初始角度，如果没有则使用当前角度
            if init_right_angles is not None:
                init_joint_angles = self._standardize_joint_angles(init_right_angles)
            else:
                init_joint_angles = self._get_standardized_joint_angles(Arm.right)
            
            result = self.R_inverse_solution.ik_fun(target_matrix, init_joint_angles=init_joint_angles)
            if result[0] is not None:  # 检查是否成功
                angles[Arm.right] = result[0].tolist() if hasattr(result[0], 'tolist') else list(result[0])
        
        if arm in (Arm.left, Arm.both):
            # 使用传入的初始角度，如果没有则使用当前角度
            if init_left_angles is not None:
                init_joint_angles = self._standardize_joint_angles(init_left_angles)
            else:
                init_joint_angles = self._get_standardized_joint_angles(Arm.left)
            result = self.L_inverse_solution.ik_fun(target_matrix, init_joint_angles=init_joint_angles)
            if result[0] is not None:  # 检查是否成功
                angles[Arm.left] = result[0].tolist() if hasattr(result[0], 'tolist') else list(result[0])
            
        return angles
        
    def pose_to_matrix(self, pose):
        """将x,y,z,rx,ry,rz转化为4x4的齐次矩阵 (ZYX欧拉角)"""
        x, y, z, rx, ry, rz = pose
        
        # 计算旋转矩阵 (ZYX欧拉角顺序)
        # R = Rz(rz) * Ry(ry) * Rx(rx)
        cos_rx, sin_rx = math.cos(rx), math.sin(rx)
        cos_ry, sin_ry = math.cos(ry), math.sin(ry)
        cos_rz, sin_rz = math.cos(rz), math.sin(rz)
        
        # Rz(rz)
        Rz = np.array([
            [cos_rz, -sin_rz, 0],
            [sin_rz, cos_rz, 0],
            [0, 0, 1]
        ])
        
        # Ry(ry)
        Ry = np.array([
            [cos_ry, 0, sin_ry],
            [0, 1, 0],
            [-sin_ry, 0, cos_ry]
        ])
        
        # Rx(rx)
        Rx = np.array([
            [1, 0, 0],
            [0, cos_rx, -sin_rx],
            [0, sin_rx, cos_rx]
        ])
        
        # 组合旋转矩阵
        R = Rz @ Ry @ Rx
        
        # 构建4x4齐次变换矩阵
        matrix = np.eye(4)
        matrix[:3, :3] = R
        matrix[:3, 3] = [x, y, z]
        
        return matrix
    
    def interpolate_pose(self, current_pose, target_pose, step, interpolate_type):
        c_x,c_y,c_z,c_rx,c_ry,c_rz = current_pose
        t_x,t_y,t_z,t_rx,t_ry,t_rz = target_pose
        
        # 单位转换：位置从mm转为m，旋转从度转为弧度
        # 检查位置是否需要转换（mm -> m）
        if any(abs(x) > 100 for x in [c_x,c_y,c_z]):
            c_x, c_y, c_z = c_x/1000.0, c_y/1000.0, c_z/1000.0
        if any(abs(x) > 100 for x in [t_x,t_y,t_z]):
            t_x, t_y, t_z = t_x/1000.0, t_y/1000.0, t_z/1000.0
        
        # 检查旋转是否需要转换（度 -> 弧度）
        if any(abs(x) > 10 for x in [c_rx,c_ry,c_rz]):
            c_rx, c_ry, c_rz = c_rx/180.0*math.pi, c_ry/180.0*math.pi, c_rz/180.0*math.pi
        if any(abs(x) > 10 for x in [t_rx,t_ry,t_rz]):
            t_rx, t_ry, t_rz = t_rx/180.0*math.pi, t_ry/180.0*math.pi, t_rz/180.0*math.pi
        
        # 重新构建转换后的位姿
        current_pose = [c_x, c_y, c_z, c_rx, c_ry, c_rz]
        target_pose = [t_x, t_y, t_z, t_rx, t_ry, t_rz]
        
        # 添加调试信息
        if hasattr(self, 'node') and self.node:
            self.node.get_logger().info(f'转换后current_pose: {current_pose}')
            self.node.get_logger().info(f'转换后target_pose: {target_pose}')
            self.node.get_logger().info(f'step_size: {step}')

        positions, rotations = self.interpolator.interpolate(current_pose, target_pose, step, interpolate_type)
        
        # 将位置和旋转组合成完整的位姿列表
        poses = []
        for i in range(len(positions)):
            pose = list(positions[i]) + list(rotations[i])
            poses.append(pose)
        
        # 添加调试信息
        if hasattr(self, 'node') and self.node:
            self.node.get_logger().info(f'插值点数: {len(poses)}')
        
        return poses
    
    # ==================== 简化的抽象方法 ====================
    
    @abstractmethod
    def get_current_pose(self, arm=Arm.right) -> List[float]:
        """
        获取当前末端执行器位姿
        Args:
            arm: 手臂选择
        Returns:
            List[float]: [x, y, z, rx, ry, rz] 格式的位姿
        """
        pass
    
    @abstractmethod
    def get_current_joint_angles(self, arm=Arm.right) -> Union[List[float], np.ndarray, object]:
        """
        获取当前关节角度
        Args:
            arm: 手臂选择
        Returns:
            可以是任何格式，基类会自动标准化为List[float]
        """
        pass
    
    # @abstractmethod
    def get_current_joint_velocities(self, arm=Arm.right) -> Union[List[float], np.ndarray, object]:
        """
        获取当前关节速度
        Args:
            arm: 手臂选择
        Returns:
            可以是任何格式，基类会自动标准化为List[float]
        """
        pass
    
    # ==================== 可选重写的方法 ====================
    
    def move_to_joint_angles(self, joint_angles: Dict[Arm, List[float]], 
                           velocity: Union[float, List[float]], 
                           acceleration: Union[float, List[float]], 
                           arm=Arm.right, block=False):
        """
        移动到指定关节角度（关节空间移动）
        子类可以重写此方法以提供自定义实现
        """
        raise NotImplementedError("子类必须实现 move_to_joint_angles 方法")
    
    def servo_move_to_joint_angles(self, joint_angles: Union[Dict[Arm, List[List[float]]], List[List[float]]], 
                                 velocity=None, acceleration=None, arm=Arm.right):
        """
        伺服移动到指定关节角度（用于插值运动）
        子类可以重写此方法以提供自定义实现
        """
        raise NotImplementedError("子类必须实现 servo_move_to_joint_angles 方法")
    
    # ==================== 辅助方法 ====================
    
    def _standardize_joint_angles(self, joint_data):
        """
        标准化关节角度数据，确保返回列表格式
        Args:
            joint_data: 可能是JointValue对象、numpy数组、列表等
        Returns:
            List[float]: 标准化的关节角度列表
        """
        if joint_data is None:
            return [0.0] * self.num_joints
        
        # 如果是JointValue对象
        if hasattr(joint_data, 'joint_values'):
            joint_values = joint_data.joint_values
            if hasattr(joint_values, 'tolist'):
                return joint_values.tolist()
            else:
                return list(joint_values)
        
        # 如果是numpy数组
        if hasattr(joint_data, 'tolist'):
            return joint_data.tolist()
        
        # 如果是列表或元组
        if isinstance(joint_data, (list, tuple)):
            return list(joint_data)
        
        # 其他情况，尝试转换为列表
        try:
            return list(joint_data)
        except:
            return [0.0] * self.num_joints
    
    def _get_standardized_joint_angles(self, arm):
        """
        获取标准化的关节角度
        Args:
            arm: 手臂选择
        Returns:
            List[float]: 标准化的关节角度列表
        """
        joint_data = self.get_current_joint_angles(arm)
        return self._standardize_joint_angles(joint_data)
    
    def _get_standardized_joint_velocities(self, arm):
        """
        获取标准化的关节速度
        Args:
            arm: 手臂选择
        Returns:
            List[float]: 标准化的关节速度列表
        """
        velocity_data = self.get_current_joint_velocities(arm)
        return self._standardize_joint_angles(velocity_data)  # 复用关节角度标准化方法
    
    
    
    
