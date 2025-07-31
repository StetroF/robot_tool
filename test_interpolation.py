#!/usr/bin/env python3

import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from robot_tool.interpolate import PoseInterpolator, InterpolateType
import numpy as np

def test_interpolation():
    """测试插值功能"""
    interpolator = PoseInterpolator()
    
    # 测试用例1：正常情况（米单位）
    print("=== 测试用例1：正常情况（米单位） ===")
    start_pose = [0.0, 0.0, 0.5, 0.0, 0.0, 0.0]  # [x, y, z, rx, ry, rz]
    end_pose = [0.2, 0.0, 0.5, 0.0, 0.0, 0.0]
    step = 0.05
    
    positions, rotations = interpolator.interpolate(start_pose, end_pose, step, InterpolateType.LINEAR)
    print(f"位置点数: {len(positions)}")
    print(f"旋转点数: {len(rotations)}")
    print(f"总点数: {len(positions)}")
    
    # 测试用例2：毫米单位（应该自动转换）
    print("\n=== 测试用例2：毫米单位（应该自动转换） ===")
    start_pose_mm = [0.0, 0.0, 500.0, 0.0, 0.0, 0.0]  # 500mm
    end_pose_mm = [200.0, 0.0, 500.0, 0.0, 0.0, 0.0]  # 200mm
    step = 0.05  # 5cm步长
    
    positions, rotations = interpolator.interpolate(start_pose_mm, end_pose_mm, step, InterpolateType.LINEAR)
    print(f"位置点数: {len(positions)}")
    print(f"旋转点数: {len(rotations)}")
    print(f"总点数: {len(positions)}")
    
    # 测试用例3：度数单位（应该自动转换）
    print("\n=== 测试用例3：度数单位（应该自动转换） ===")
    start_pose_deg = [0.0, 0.0, 0.5, 0.0, 0.0, 0.0]  # 0度
    end_pose_deg = [0.2, 0.0, 0.5, 0.0, 0.0, 90.0]   # 90度
    step = 0.05
    
    positions, rotations = interpolator.interpolate(start_pose_deg, end_pose_deg, step, InterpolateType.LINEAR)
    print(f"位置点数: {len(positions)}")
    print(f"旋转点数: {len(rotations)}")
    print(f"总点数: {len(positions)}")
    
    # 测试用例4：极端情况（大距离）
    print("\n=== 测试用例4：极端情况（大距离） ===")
    start_pose_large = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    end_pose_large = [1000.0, 1000.0, 1000.0, 0.0, 0.0, 0.0]  # 1000mm
    step = 0.01  # 1cm步长
    
    positions, rotations = interpolator.interpolate(start_pose_large, end_pose_large, step, InterpolateType.LINEAR)
    print(f"位置点数: {len(positions)}")
    print(f"旋转点数: {len(rotations)}")
    print(f"总点数: {len(positions)}")

if __name__ == "__main__":
    test_interpolation() 