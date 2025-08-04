"""
JAKA机器人控制器
1.调用ik.py的逆解接口,计算出关节角度，发送给/servo_joint_cmd,使得机器人运动
"""

import rclpy
from rclpy.node import Node
from jaka_robot_interfaces.msg import JointValue, MoveMode, RobotStateDual, CartesianPose, ServoJointCommand
from jaka_robot_interfaces.srv import MultiMovJ
from jiazhua_interfaces.msg import JiaZhuaDualCmd  # 需要导入夹爪消息类型
from typing import Literal, Union, List, Dict
from robot_tool.base_robot_controller import BaseRobotController
from robot_tool.base_robot_controller import Arm, MoveToRequest
import time
import threading
import time
from robot_tool.interpolate import InterpolateType
from ast import literal_eval
class JakaController(BaseRobotController):
    def __init__(self):
        self.node = Node('jaka_controller')
        self.robot_state: RobotStateDual = None
        
        self._init_ros2()
        super().__init__(self.urdf_path, self.arm_prefix, self.end_effector_link_name, self.num_joints, self.visualize,self.enable_collision_detection,self.igno_coll_pairs)

    def _init_ros2(self):
        self.node.declare_parameter('urdf_path', '/home/zy/Project/jaka3/ROS2/jaka_ws/src/dual_arm/urdf/dual_arm.urdf')
        self.node.declare_parameter('arm_prefix', ['r', 'l'])
        self.node.declare_parameter('end_effector_link_name', ['rt', 'lt'])
        self.node.declare_parameter('num_joints', 7)
        self.node.declare_parameter('visualize', False)
        self.node.declare_parameter('servo_publish_rate', 125.0)
        self.node.declare_parameter('enable_collision_detection', True)
        self.node.declare_parameter('igno_coll_pairs', "[['l6_0','l7_0'],['r6_0','r7_0']]")
        
        # 插值参数
        self.node.declare_parameter('interpolation.default_step', 0.01)
        self.node.declare_parameter('interpolation.max_points', 1000)
        self.node.declare_parameter('interpolation.default_type', 'BEZIER')
        
        # 运动参数
        self.node.declare_parameter('motion.default_velocity', 0.1)
        self.node.declare_parameter('motion.default_acceleration', 0.1)
        self.node.declare_parameter('motion.max_velocity', 1.0)
        self.node.declare_parameter('motion.max_acceleration', 1.0)
        
        # 获取参数值
        self.urdf_path = self.node.get_parameter('urdf_path').value
        self.arm_prefix = self.node.get_parameter('arm_prefix').value
        self.end_effector_link_name = self.node.get_parameter('end_effector_link_name').value
        self.num_joints = self.node.get_parameter('num_joints').value
        self.visualize = self.node.get_parameter('visualize').value
        self.servo_publish_rate = self.node.get_parameter('servo_publish_rate').value
        self.enable_collision_detection = self.node.get_parameter('enable_collision_detection').value
        self.igno_coll_pairs = self.node.get_parameter('igno_coll_pairs').value
        self.igno_coll_pairs = literal_eval(self.igno_coll_pairs) ##字符串转列表
        # 获取插值参数
        self.default_step = self.node.get_parameter('interpolation.default_step').value
        self.max_points = self.node.get_parameter('interpolation.max_points').value
        self.default_interpolate_type = self.node.get_parameter('interpolation.default_type').value
        
        # 获取运动参数
        self.default_velocity = self.node.get_parameter('motion.default_velocity').value
        self.default_acceleration = self.node.get_parameter('motion.default_acceleration').value
        self.max_velocity = self.node.get_parameter('motion.max_velocity').value
        self.max_acceleration = self.node.get_parameter('motion.max_acceleration').value
        
        # 打印配置信息
        self.node.get_logger().info(f'Loaded configuration:')
        self.node.get_logger().info(f'  URDF: {self.urdf_path}')
        self.node.get_logger().info(f'  Arm prefixes: {self.arm_prefix}')
        self.node.get_logger().info(f'  End effectors: {self.end_effector_link_name}')
        self.node.get_logger().info(f'  Joints: {self.num_joints}')
        self.node.get_logger().info(f'  Servo rate: {self.servo_publish_rate}Hz')
        self.node.get_logger().info(f'  Default step: {self.default_step}m')
        self.node.get_logger().info(f'  Default velocity: {self.default_velocity}')
        self.node.get_logger().info(f'  Default acceleration: {self.default_acceleration}')
        self.node.get_logger().info(f'  Enable collision detection: {self.enable_collision_detection}')
        self.node.get_logger().info(f'  Ignored collision pairs: {self.igno_coll_pairs}')
        self.node.create_subscription(RobotStateDual, 'robot_state_dual', self.robot_state_callback, 10)
        self.move_j_client = self.node.create_client(MultiMovJ, 'multi_movj')
        
        # 创建伺服发布器
        self.servo_publisher = self.node.create_publisher(ServoJointCommand, '/servo_joint_command', 10)
        
        # 等待服务可用
        while not self.move_j_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('Waiting for multi_movj service...')
    
    # ==================== 必须实现的抽象方法 ====================
    
    def get_current_pose(self, arm=Arm.right) -> List[float]:
        """获取当前末端执行器位姿"""
        if not self.robot_state:
            raise ValueError('robot_state is not initialized')
        
        if arm == Arm.right:
            return [
                self.robot_state.end_pose_right.x,
                self.robot_state.end_pose_right.y,
                self.robot_state.end_pose_right.z,
                self.robot_state.end_pose_right.rx,
                self.robot_state.end_pose_right.ry,
                self.robot_state.end_pose_right.rz
            ]
        else:
            return [
                self.robot_state.end_pose_left.x,
                self.robot_state.end_pose_left.y,
                self.robot_state.end_pose_left.z,
                self.robot_state.end_pose_left.rx,
                self.robot_state.end_pose_left.ry,
                self.robot_state.end_pose_left.rz
            ]
    
    def get_current_joint_angles(self, arm=Arm.right):
        """获取当前关节角度"""
        if not self.robot_state:
            raise ValueError('robot_state is not initialized')
        
        if arm == Arm.right:
            return self.robot_state.joint_pos_right
        else:
            return self.robot_state.joint_pos_left
    
    def get_current_joint_velocities(self, arm=Arm.right):
        """获取当前关节速度"""
        if not self.robot_state:
            raise ValueError('robot_state is not initialized')
        
        if arm == Arm.right:
            return self.robot_state.joint_vel_right
        else:
            return self.robot_state.joint_vel_left
    
    # ==================== 重写的运动方法 ====================
    
    def move_to_joint_angles(self, joint_angles: Dict[Arm, List[float]], 
                           velocity: Union[float, List[float]] = None, 
                           acceleration: Union[float, List[float]] = None, 
                           arm=Arm.right, block=False):
        """移动到指定关节角度"""
        # 使用配置文件中的默认值
        if velocity is None:
            velocity = self.default_velocity
        if acceleration is None:
            acceleration = self.default_acceleration
        request = MultiMovJ.Request()
        request.robot_id = arm
        request.is_block = block
        request.left_move_mode.mode = 0  # 绝对运动
        request.right_move_mode.mode = 0
        
        # 设置关节角度
        if arm == Arm.left:
            for i in range(self.num_joints):
                request.joint_pos_left.joint_values[i] = joint_angles[Arm.left][i]
            # 右臂保持当前状态
            current_right_angles = self._get_standardized_joint_angles(Arm.right)
            for i in range(self.num_joints):
                request.joint_pos_right.joint_values[i] = current_right_angles[i]
        elif arm == Arm.right:
            for i in range(self.num_joints):
                request.joint_pos_right.joint_values[i] = joint_angles[Arm.right][i]
            # 左臂保持当前状态
            current_left_angles = self._get_standardized_joint_angles(Arm.left)
            for i in range(self.num_joints):
                request.joint_pos_left.joint_values[i] = current_left_angles[i]
        elif arm == Arm.both:
            for i in range(self.num_joints):
                request.joint_pos_left.joint_values[i] = joint_angles[Arm.left][i]
                request.joint_pos_right.joint_values[i] = joint_angles[Arm.right][i]
        else:
            raise ValueError(f"Invalid arm: {arm}")
        
        # 设置速度和加速度
        if isinstance(velocity, (list, tuple)):
            request.vel = velocity
        else:
            request.vel = [velocity, velocity]
            
        if isinstance(acceleration, (list, tuple)):
            request.acc = acceleration
        else:
            request.acc = [acceleration, acceleration]
        
        # 发送请求
        future = self.move_j_client.call_async(request)
        
        if block:
            rclpy.spin_until_future_complete(self.node, future)
            if future.result():
                return future.result()
            else:
                raise RuntimeError("Failed to execute joint motion")
        else:
            return future
    
    def servo_move_to_joint_angles(self, joint_angles: Union[Dict[Arm, List[List[float]]], List[List[float]]], 
                                 velocity=None, acceleration=None, arm=Arm.right):
        """伺服移动到指定关节角度"""
        # 使用配置文件中的默认值
        if velocity is None:
            velocity = self.default_velocity
        if acceleration is None:
            acceleration = self.default_acceleration
        # 检查机器人状态是否可用
        if not self.is_robot_state_available():
            raise ValueError('robot_state is not available')
        
        #在joint_angles的列表队尾添加250个最后一个同样的数据
        ###为什么这么做？ 因为运动到最后一个点后似乎还有惯性，所以多发一点
        if isinstance(joint_angles, list):
            joint_angles.extend([joint_angles[-1]] * 250)
        elif isinstance(joint_angles, dict):
            joint_angles[Arm.left].extend([joint_angles[Arm.left][-1]] * 250)
            joint_angles[Arm.right].extend([joint_angles[Arm.right][-1]] * 250)
        
        joint_angles_list = self._process_servo_joint_angles(joint_angles, arm)
        
        # 计算发布间隔
        publish_interval = 1.0 / self.servo_publish_rate
        
        # 发布每组关节角度
        for i, angles in enumerate(joint_angles_list):
            msg = ServoJointCommand()
            
            # 设置左臂关节角度
            if Arm.left in angles:
                for j in range(self.num_joints):
                    msg.joint_pos_left.joint_values[j] = angles[Arm.left][j]
            else:
                # 保持当前左臂位置
                left_angles_list = self._get_standardized_joint_angles(Arm.left)
                for j in range(self.num_joints):
                    msg.joint_pos_left.joint_values[j] = left_angles_list[j]
            
            # 设置右臂关节角度
            if Arm.right in angles:
                for j in range(self.num_joints):
                    msg.joint_pos_right.joint_values[j] = angles[Arm.right][j]
            else:
                # 保持当前右臂位置
                right_angles_list = self._get_standardized_joint_angles(Arm.right)
                for j in range(self.num_joints):
                    msg.joint_pos_right.joint_values[j] = right_angles_list[j]
            
            # 发布消息
            self.servo_publisher.publish(msg)
            
            # 如果不是最后一组，则等待指定间隔
            if i < len(joint_angles_list) - 1:
                time.sleep(publish_interval)
        
        self.node.get_logger().info(f'Servo command published: {len(joint_angles_list)} waypoints at {self.servo_publish_rate}Hz')
    
    # ==================== 辅助方法 ====================
    
    def _process_servo_joint_angles(self, joint_angles, arm):
        """处理伺服关节角度输入格式"""
        if isinstance(joint_angles, dict):
            # 新格式：{Arm.left: [[j1,j2,j3,j4,j5,j6,j7], ...], Arm.right: [[j1,j2,j3,j4,j5,j6,j7], ...]}
            if Arm.left in joint_angles and Arm.right in joint_angles:
                # 确保两个手臂的序列长度相同
                left_sequence = joint_angles[Arm.left]
                right_sequence = joint_angles[Arm.right]
                max_length = max(len(left_sequence), len(right_sequence))
                
                # 如果长度不同，用最后一个值填充
                if len(left_sequence) < max_length:
                    last_left = left_sequence[-1] if left_sequence else [0.0] * self.num_joints
                    left_sequence.extend([last_left] * (max_length - len(left_sequence)))
                if len(right_sequence) < max_length:
                    last_right = right_sequence[-1] if right_sequence else [0.0] * self.num_joints
                    right_sequence.extend([last_right] * (max_length - len(right_sequence)))
                
                joint_angles_list = []
                for i in range(max_length):
                    joint_angles_list.append({
                        Arm.left: left_sequence[i],
                        Arm.right: right_sequence[i]
                    })
            else:
                # 兼容旧格式：单组关节角度
                joint_angles_list = [joint_angles]
        elif isinstance(joint_angles, (list, tuple)) and len(joint_angles) > 0:
            if isinstance(joint_angles[0], (list, tuple)):
                # 多组关节角度
                joint_angles_list = []
                for angles in joint_angles:
                    if arm == Arm.left:
                        right_angles_list = self._get_standardized_joint_angles(Arm.right)
                        joint_angles_list.append({Arm.left: angles, Arm.right: right_angles_list})
                    elif arm == Arm.right:
                        left_angles_list = self._get_standardized_joint_angles(Arm.left)
                        joint_angles_list.append({Arm.left: left_angles_list, Arm.right: angles})
                    elif arm == Arm.both:
                        if len(angles) == 2:
                            joint_angles_list.append({Arm.left: angles[0], Arm.right: angles[1]})
                        else:
                            raise ValueError("For both arms, joint_angles should be [[left_angles], [right_angles]]")
            else:
                # 单组关节角度（列表格式）
                if arm == Arm.left:
                    right_angles_list = self._get_standardized_joint_angles(Arm.right)
                    joint_angles_list = [{Arm.left: joint_angles, Arm.right: right_angles_list}]
                elif arm == Arm.right:
                    left_angles_list = self._get_standardized_joint_angles(Arm.left)
                    joint_angles_list = [{Arm.left: left_angles_list, Arm.right: joint_angles}]
                else:
                    raise ValueError("For both arms, joint_angles should be a dict with both arms")
        else:
            raise ValueError("Invalid joint_angles format")
        
        return joint_angles_list
    
    def robot_state_callback(self, msg: RobotStateDual):
        """机器人状态回调函数"""
        self.robot_state = msg
    
    def is_robot_state_available(self):
        """检查机器人状态是否可用"""
        return self.robot_state is not None
    
    def get_robot_state(self):
        """获取完整的机器人状态"""
        if not self.robot_state:
            raise ValueError('robot_state is not initialized')
        return self.robot_state


def main():
    rclpy.init()
    controller = JakaController()

    import math
    
    t = threading.Thread(target=rclpy.spin, args=(controller.node,))
    t.start()
    time.sleep(1)
    
    robot_pose = controller.get_current_pose(Arm.left)
    x, y, z, rx, ry, rz = robot_pose
    print(f'robot_pose: {x, y, z, rx, ry, rz}')

    x = x/1000.0
    y = y/1000.0
    z = z/1000.0
    rx = rx/180.0*math.pi
    ry = ry/180.0*math.pi
    rz = rz/180.0*math.pi
    target_pose = [x, y +0.4, z, rx, ry, rz]
     
    controller.move_to_tag(MoveToRequest(
        target_pose=target_pose, 
        arm=Arm.left, 
        interpolate=True, 
        interpolate_type=InterpolateType.LINEAR, 
        step=controller.default_step
    ))

    time.sleep(3)
    robot_pose = controller.get_current_pose(Arm.left)
    x, y, z, rx, ry, rz = robot_pose
    print(f'robot_pose: {x, y, z, rx, ry, rz}')
    
    while True:
        time.sleep(1)

if __name__ == '__main__':
    main()