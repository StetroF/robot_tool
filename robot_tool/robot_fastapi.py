from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
import numpy as np
from typing import List, Dict, Optional
from enum import Enum
import math
from data_struct import Arm, InterpolateType  # 导入Arm枚举和InterpolateType
from fastapi.middleware.cors import CORSMiddleware
from jaka_controller import JakaController
import rclpy
from typing import Literal
# 定义请求/响应模型
class Pose(BaseModel):
    x: float  # 米
    y: float  # 米
    z: float  # 米
    rx: float  # 弧度
    ry: float  # 弧度
    rz: float  # 弧度
class MoveToRequest(BaseModel):
    target_pose: list[float]    #### [x,y,z,rx,ry,rz]
    arm: int = Arm.right
    interpolate: bool = False
    interpolate_type: int = InterpolateType.LINEAR
    step: float = 0.01  ## 插值步长
    velocity: float = 20.0  ##°/s
    acceleration: float = 20.0  ##°/s^2
    block: bool = False ##运动时是否阻塞
    




class RelativeMoveRequest(BaseModel):
    dx: float  # 米
    dy: float  # 米
    dz: float  # 米
    drx: Optional[float] = 0.0  # 弧度
    dry: Optional[float] = 0.0  # 弧度
    drz: Optional[float] = 0.0  # 弧度
    arm: int = Arm.right
    interpolate: bool = False
    interpolate_type: int = InterpolateType.LINEAR
    step: float = 0.01  # 插值步长
    velocity: Optional[float] = None
    acceleration: Optional[float] = None
    block: bool = False

class FastAPIRobotController:
    def __init__(self, robot_controller):
        self.app = FastAPI()
        self.app.add_middleware(
            CORSMiddleware,
            allow_origins=["*"],  # 允许所有来源
            allow_methods=["*"],
            allow_headers=["*"],
        )
        self.controller:JakaController = robot_controller
        self.add_api_routes()
        
    def add_api_routes(self):
        
        
        @self.app.get("/current_pose/{arm}", response_model=Pose)
        async def get_current_pose(arm: int):
            """获取当前位姿（米和弧度）"""
            try:
                # 从控制器获取原始数据（毫米和角度）
                raw_pose = self.controller.get_current_pose(arm)
                
                # 转换为米和弧度
                pose = Pose(
                    x=raw_pose[0] / 1000.0,
                    y=raw_pose[1] / 1000.0,
                    z=raw_pose[2] / 1000.0,
                    rx=math.radians(raw_pose[3]),
                    ry=math.radians(raw_pose[4]),
                    rz=math.radians(raw_pose[5])
                )
                return pose
            except Exception as e:
                raise HTTPException(status_code=500, detail=str(e))
        
        @self.app.post("/move_to_pose")
        async def move_to_pose(request: MoveToRequest):
            """移动到指定位姿"""
            try:
                # 将目标位姿转换为控制器需要的格式（毫米和角度）
                target_pose = [
                    request.target_pose[0] * 1000.0,
                    request.target_pose[1] * 1000.0,
                    request.target_pose[2] * 1000.0,
                    math.degrees(request.target_pose[3]),
                    math.degrees(request.target_pose[4]),
                    math.degrees(request.target_pose[5])
                ]
                
                # 创建MoveToRequest对象
                from data_struct import MoveToRequest as ControllerMoveToRequest
                controller_request = ControllerMoveToRequest(
                    target_pose=target_pose,
                    arm=request.arm,
                    interpolate=request.interpolate,
                    interpolate_type=request.interpolate_type,
                    step=request.step,
                    velocity=request.velocity,
                    acceleration=request.acceleration,
                    block=request.block
                )
                
                # 调用控制器方法
                self.controller.move_to_tag(controller_request)
                return {"status": "success", "message": "Movement command sent"}
            except Exception as e:
                raise HTTPException(status_code=500, detail=str(e))
        
        @self.app.post("/relative_move")
        async def relative_move(request: RelativeMoveRequest):
            """相对移动"""
            try:
                # 获取当前位姿（米和弧度）
                current_pose = await get_current_pose(request.arm)
                
                # 计算新位姿
                new_pose = Pose(
                    x=current_pose.x + request.dx,
                    y=current_pose.y + request.dy,
                    z=current_pose.z + request.dz,
                    rx=current_pose.rx + request.drx,
                    ry=current_pose.ry + request.dry,
                    rz=current_pose.rz + request.drz
                )
                
                # 转换为控制器需要的格式（毫米和角度）
                target_pose = [
                    new_pose.x * 1000.0,
                    new_pose.y * 1000.0,
                    new_pose.z * 1000.0,
                    math.degrees(new_pose.rx),
                    math.degrees(new_pose.ry),
                    math.degrees(new_pose.rz)
                ]
                
                if request.interpolate:
                    # 伺服模式下的相对移动
                    current_raw_pose = self.controller.get_current_pose(request.arm)
                    interpolated_poses = self.controller.interpolate_pose(
                        current_raw_pose,
                        target_pose,
                        request.step,  # 默认步长
                        request.interpolate_type
                    )
                    
                    # 计算所有插值点的关节角度
                    joint_angles_sequence = {
                        Arm.left: [],
                        Arm.right: []
                    }
                    for pose in interpolated_poses:
                        angles = self.controller._calculate_single_pose_joint_angles(pose, request.arm)
                        joint_angles_sequence[Arm.left].append(angles[Arm.left])
                        joint_angles_sequence[Arm.right].append(angles[Arm.right])
                    
                    # 根据手臂选择，只保留需要的关节角度序列
                    if request.arm == Arm.left:
                        # 只移动左臂，右臂保持当前位置
                        current_right_angles = self.controller.get_current_joint_angles(Arm.right)
                        joint_angles_sequence[Arm.right] = [current_right_angles.joint_values] * len(interpolated_poses)
                    elif request.arm == Arm.right:
                        # 只移动右臂，左臂保持当前位置
                        current_left_angles = self.controller.get_current_joint_angles(Arm.left)
                        joint_angles_sequence[Arm.left] = [current_left_angles.joint_values] * len(interpolated_poses)
                    
                    # 伺服模式移动
                    self.controller.servo_move_to_joint_angles(
                        joint_angles_sequence,
                        arm=request.arm
                    )
                else:
                    # 普通模式移动
                    from data_struct import MoveToRequest as ControllerMoveToRequest
                    controller_request = ControllerMoveToRequest(
                        target_pose=target_pose,
                        arm=request.arm,
                        interpolate=False,
                        velocity=request.velocity,
                        acceleration=request.acceleration,
                        block=request.block
                    )
                    self.controller.move_to_tag(controller_request)
                
                return {"status": "success", "message": "Relative movement executed"}
            except Exception as e:
                raise HTTPException(status_code=500, detail=str(e))
        
        @self.app.post("/servo_relative_move")
        async def servo_relative_move(request: RelativeMoveRequest):
            """伺服模式下的相对移动（强制使用插值）"""
            try:
                # 设置interpolate=True并调用relative_move
                request.interpolate = True
                return await relative_move(request)
            except Exception as e:
                raise HTTPException(status_code=500, detail=str(e))

# 使用示例
if __name__ == "__main__":
    import uvicorn
    import threading
    
    if not rclpy.ok():
        rclpy.init()
    # 创建Jaka控制器实例
    jaka_controller = JakaController()
    
    # 创建FastAPI包装器
    api_controller = FastAPIRobotController(jaka_controller)
    spin_ros_thread = threading.Thread(target=rclpy.spin, args=(jaka_controller.node,))
    spin_ros_thread.start()
    
    # 运行FastAPI应用
    uvicorn.run(api_controller.app, host="0.0.0.0", port=8000)