#!/usr/bin/env python3
"""
测试碰撞检测数据结构是否正确

这个脚本用于验证：
1. Arm_IK的collision_detect方法期望的输入格式
2. BaseRobotController传递给collision_detect的数据格式
3. 确保数据长度匹配
"""

import rclpy
import numpy as np
from robot_tool.jaka_controller import JakaController
from robot_tool.base_robot_controller import Arm, MoveToRequest
from robot_tool.interpolate import InterpolateType

def test_data_structure():
    """测试数据结构"""
    print("="*50)
    print("碰撞检测数据结构测试")
    print("="*50)
    
    # 初始化ROS2
    rclpy.init()
    
    # 创建控制器
    controller = JakaController()
    
    print(f"1. 检查Arm_IK的full_robot模型:")
    print(f"   - 完整模型关节数 (full_robot.model.nq): {controller.R_inverse_solution.full_robot.model.nq}")
    print(f"   - 简化模型关节数 (robot.model.nq): {controller.R_inverse_solution.robot.model.nq}")
    print(f"   - 左臂关节数: {len(controller.L_inverse_solution.robot.model.joints)}")
    print(f"   - 右臂关节数: {len(controller.R_inverse_solution.robot.model.joints)}")
    
    print(f"\n2. 检查关节角度数据结构:")
    
    # 测试单臂关节角度
    test_angles = {
        Arm.left: [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7],
        Arm.right: [0.8, 0.9, 1.0, 1.1, 1.2, 1.3, 1.4]
    }
    
    print(f"   - 左臂关节角度: {test_angles[Arm.left]}")
    print(f"   - 右臂关节角度: {test_angles[Arm.right]}")
    print(f"   - 左臂关节角度长度: {len(test_angles[Arm.left])}")
    print(f"   - 右臂关节角度长度: {len(test_angles[Arm.right])}")
    
    # 测试构建完整关节角度
    print(f"\n3. 测试_build_full_joint_angles方法:")
    
    # 模拟当前关节角度
    controller._get_standardized_joint_angles = lambda arm: [0.0] * 7
    
    full_angles_left = controller._build_full_joint_angles(test_angles, Arm.left)
    full_angles_right = controller._build_full_joint_angles(test_angles, Arm.right)
    full_angles_both = controller._build_full_joint_angles(test_angles, Arm.both)
    
    print(f"   - 只移动左臂时的完整关节角度长度: {len(full_angles_left)}")
    print(f"   - 只移动右臂时的完整关节角度长度: {len(full_angles_right)}")
    print(f"   - 移动双臂时的完整关节角度长度: {len(full_angles_both)}")
    print(f"   - 期望的完整关节角度长度: {controller.R_inverse_solution.full_robot.model.nq}")
    
    print(f"\n4. 验证数据长度匹配:")
    expected_length = controller.R_inverse_solution.full_robot.model.nq
    actual_lengths = [len(full_angles_left), len(full_angles_right), len(full_angles_both)]
    
    for i, length in enumerate(actual_lengths):
        if length == expected_length:
            print(f"   ✅ 长度匹配正确")
        else:
            print(f"   ❌ 长度不匹配: 期望 {expected_length}, 实际 {length}")
    
    print(f"\n5. 测试碰撞检测调用:")
    try:
        # 使用一个简单的测试位姿
        test_pose = [0.5, 0.0, 0.5, 0.0, 0.0, 0.0]  # [x, y, z, rx, ry, rz]
        
        # 计算关节角度
        angles = controller._calculate_single_pose_joint_angles(test_pose, Arm.right)
        print(f"   - 计算得到的关节角度: {angles}")
        
        # 构建完整关节角度
        full_angles = controller._build_full_joint_angles(angles, Arm.right)
        print(f"   - 构建的完整关节角度长度: {len(full_angles)}")
        
        # 尝试调用碰撞检测
        collision_detected, collision_pair = controller.R_inverse_solution.collision_detect(full_angles)
        print(f"   - 碰撞检测结果: {collision_detected}, {collision_pair}")
        print(f"   ✅ 碰撞检测调用成功")
        
    except Exception as e:
        print(f"   ❌ 碰撞检测调用失败: {e}")
    
    # 清理资源
    controller.node.destroy_node()
    rclpy.shutdown()
    
    print(f"\n" + "="*50)
    print("测试完成")
    print("="*50)

if __name__ == '__main__':
    test_data_structure() 