from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from jaka_controller import JakaController, MoveToRequest
from pydantic import BaseModel
from jaka_controller import Arm
from threading import Thread
import rclpy
import uvicorn
from typing import Literal
import math
from interpolate import InterpolateType
import logging
class RelativeMoveToRequest(BaseModel):
    arm: Literal[Arm.right, Arm.left] = Arm.left
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    interpolate: bool = False
    interpolate_type: Literal[InterpolateType.LINEAR, InterpolateType.BEZIER, InterpolateType.CIRCULAR_ARC, InterpolateType.CUBIC_POLY, InterpolateType.QUINTIC_POLY, InterpolateType.PARABOLIC] = InterpolateType.LINEAR
    step: float = 0.01
    velocity: float = 20.0
    acceleration: float = 20.0



class JakaFastAPI:
    def __init__(self):
        self.controller = JakaController()
        self.app = FastAPI()
        self.app.add_middleware(
            CORSMiddleware,
            allow_origins=["*"],
            allow_credentials=True,
            allow_methods=["*"],
            allow_headers=["*"],
        )
        self.logger = self.controller.node.get_logger()
        
    def _add_routes(self):
        self.app.add_api_route("/move_to_tag", self.move_to_tag, methods=["POST"])
        self.app.add_api_route("/get_current_pose", self.get_current_pose, methods=["GET"])
        self.app.add_api_route("/relative_move_to", self.relative_move_to, methods=["POST"])
    def move_to_tag(self, request: MoveToRequest):
        self.controller.move_to_tag(request)
    
    def relative_move_to(self, request: RelativeMoveToRequest):
        current_pose = self.controller.get_current_pose(request.arm)
        self.logger.info(f"current_pose: {current_pose}")
        current_pose[0]/=1000.0
        current_pose[1]/=1000.0
        current_pose[2]/=1000.0
        current_pose[3]/=180.0*math.pi
        current_pose[4]/=180.0*math.pi
        current_pose[5]/=180.0*math.pi
        target_pose = [current_pose[0] + request.x, current_pose[1] + request.y, current_pose[2] + request.z, current_pose[3] ,current_pose[4], current_pose[5]]
        self.controller.move_to_tag(MoveToRequest(target_pose=target_pose, arm=request.arm,interpolate=request.interpolate,interpolate_type=request.interpolate_type,step=request.step,velocity=request.velocity,acceleration=request.acceleration))
        # self.logger.info(f"target_pose: {target_pose}")
        current_pose = self.controller.get_current_pose(request.arm)
        self.logger.info(f"移动后pose: {current_pose}")
        return {"message": "Relative move to successful"}
    def get_current_pose(self, arm: Literal[Arm.right, Arm.left]=Arm.left):
        return self.controller.get_current_pose(arm)
    def get_current_joint_angles(self, arm: Literal[Arm.right, Arm.left]=Arm.left):
        return self.controller.get_current_joint_angles(arm)

    
    def run(self):
        self._add_routes()
        # self.app.run(host="0.0.0.0", port=8000)
        uvicorn.run(self.app, host="0.0.0.0", port=8000)
        
if __name__ == "__main__":
    if not rclpy.ok():
        rclpy.init()
    jaka_fastapi = JakaFastAPI()
    spin_thread = Thread(target=rclpy.spin, args=(jaka_fastapi.controller.node,))
    spin_thread.start()
    jaka_fastapi.run()
    spin_thread.join()
    
    