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
    


