import numpy as np
import sys,os
home_dir = os.path.expanduser("~")
conda_env_path = os.environ.get("CONDA_PREFIX",os.path.join(home_dir,"miniconda3","envs","humble"))
if conda_env_path:
    conda_site_packages = os.path.join(conda_env_path, "lib", f"python{sys.version_info.major}.{sys.version_info.minor}", "site-packages")
    print(f"conda path: {conda_site_packages}")
    if conda_site_packages not in sys.path:
        sys.path.append(conda_site_packages)
from pinocchio import casadi as cpin
from pinocchio.visualize import MeshcatVisualizer
import meshcat.geometry as mg
import pinocchio as pin
import casadi
import logging
import time


class Arm_IK:
    """
    一个通用的、基于优化的逆运动学求解器，适用于7自由度机械臂。
    """
    def __init__(self, urdf_path, arm_prefix, end_effector_link_name, num_joints=6, visualize=False,enable_collision_detection=False,igno_coll_pairs=[['l6_0','l7_0'],['r6_0','r7_0']]):
        np.set_printoptions(precision=5, suppress=True, linewidth=200)

        self.arm_prefix = arm_prefix
        self.visualize = visualize
        self.logger = logging.getLogger(__name__+'.Arm_IK')
        # 1. 加载完整的双臂机器人模型
        self.full_robot = pin.RobotWrapper.BuildFromURDF(urdf_path)
        
        # 2. 识别并选择当前手臂的运动链
        self.joint_names = [f"{self.arm_prefix}-j{i+1}" for i in range(7)]



        joints_to_actuate_ids = []

        for joint_name in self.joint_names:
            if self.full_robot.model.existJointName(joint_name):
                joint_id = self.full_robot.model.getJointId(joint_name)
                joints_to_actuate_ids.append(joint_id)
            else:
                print(f"Joint {joint_name} not found in model")
                raise ValueError(f"Joint {joint_name} not found in model")

        # 3. 从完整模型中构建出单臂的简化模型
        q0 = pin.neutral(self.full_robot.model)

        # 获取所有关节ID，然后排除我们要保留的关节
        all_joint_ids = list(range(1, self.full_robot.model.njoints))  # 跳过universe joint (id=0)
        joints_to_lock = [jid for jid in all_joint_ids if jid not in joints_to_actuate_ids]
        
        self.robot = self.full_robot.buildReducedRobot(
            list_of_joints_to_lock=joints_to_lock,
            reference_configuration=q0
        )
        self.logger.info(f"简化模型关节数: {self.robot.model.nq}")
        self.logger.info(f'关节数量: {self.robot.model.njoints}')
        # 4. 定义末端执行器框架 (TCP) - 可选
        if end_effector_link_name is not None:
            tool_frame_id_in_full_model = self.full_robot.model.getFrameId(end_effector_link_name)
            tool_placement = self.full_robot.model.frames[tool_frame_id_in_full_model].placement
            self.logger.info(f'Tool placement : {tool_placement}')
            # 关键修复：在简化模型中，关节名称可能保持不变，但需要验证
            last_joint_name = self.joint_names[-1]  # 'joint6'
            
            # 验证关节是否存在于简化模型中
            if self.robot.model.existJointName(last_joint_name):
                last_joint_id = self.robot.model.getJointId(last_joint_name)
                print(f"Adding ee frame for {arm_prefix} arm, joint {last_joint_name} with ID {last_joint_id}")
                
                self.robot.model.addFrame(
                    pin.Frame('ee',
                              last_joint_id,
                              tool_placement,
                              pin.FrameType.OP_FRAME) # type: ignore
                )
            else:
                # 如果找不到关节，使用最后一个关节ID
                last_joint_id = self.robot.model.njoints - 1
                print(f"Warning: Joint {last_joint_name} not found in reduced model for {arm_prefix}, using joint ID {last_joint_id}")
                
                self.robot.model.addFrame(
                    pin.Frame('ee',
                              last_joint_id,
                              tool_placement,
                              pin.FrameType.OP_FRAME) # type: ignore
                )
        else:
            print(f"No end effector specified for {arm_prefix} arm, using last joint as end effector")

        # 5. 初始化数据和可视化
        self.init_data = np.zeros(self.robot.model.nq)
        print(f'init_data: {self.init_data}')
        self.history_data = np.zeros(self.robot.model.nq)
        
        
        ###碰撞检测部分，然后禁用l6_0和l7_0的碰撞检测(因为这两个关节会被错误的判断出碰撞)
        if enable_collision_detection:
            self.enable_collision_detection = True
            
            self.geom_model = pin.buildGeomFromUrdf(self.full_robot.model, urdf_path, pin.GeometryType.COLLISION)
            self.geom_model.addAllCollisionPairs()
            self.geom_data = pin.GeometryData(self.geom_model)
            for pair in igno_coll_pairs:
                print(f"Ignoring collision pair: {pair}")
                self.geom_data = self.remove_collision_pair(self.geom_model,self.geom_data,pair[0],pair[1])
            
        
        
        if self.visualize:
            self.vis = MeshcatVisualizer(self.full_robot.model, self.full_robot.collision_model, self.full_robot.visual_model)
            self.vis.initViewer(open=True)
            self.vis.loadViewerModel(f"pinocchio/{self.arm_prefix}")
            self.vis.display(pin.neutral(self.full_robot.model))
            self._setup_target_visualizer()

        # 6. 设置基于CasADi的优化问题
        self._setup_optimization_problem()
    def collision_detect(self,q):
        """
        碰撞检测，返回是否发生碰撞，以及碰撞对名称
        传入参数: 
         q: [q1,q2,...,qn] 关节角度
        返回:
            collision_detected: 是否发生碰撞
            collision_pair: ['l6_0','l7_0'] 碰撞对名称
        """
        if len(q) != self.full_robot.model.nq:
            raise ValueError(f'关节角度数量不匹配,期望数量: {self.full_robot.model.nq}, 参数传入数量: {len(q)}')
        
        
        start_time = time.time()
        pin.forwardKinematics(self.full_robot.model, self.full_robot.data, q)
        pin.updateGeometryPlacements(self.full_robot.model, self.full_robot.data, self.geom_model, self.geom_data, q) 
        pin.computeCollisions(self.geom_model, self.geom_data, True)
        print(f'collision time: {time.time() - start_time}')                
        for k in range(len(self.geom_model.collisionPairs)):
            cp = self.geom_model.collisionPairs[k]
            cr = self.geom_data.collisionResults[k]
            if cr.isCollision():
                first_geom = self.geom_model.geometryObjects[cp.first].name
                second_geom = self.geom_model.geometryObjects[cp.second].name
                print(f"Collision pair {k}: {first_geom} and {second_geom}")
                return True,[first_geom,second_geom]
        return False,[]
    
    def test_collision(self):
        if not self.geom_model: #type: ignore
            print("碰撞检测未启用，无法运行碰撞测试")
            return
        if not self.visualize:
            print("可视化未启用，无法运行碰撞测试")
            return
        
        try:
            while True:
                # 1. 生成随机关节角度
                q = pin.randomConfiguration(self.full_robot.model)
                # 2. 更新运动学和几何位置
                pin.forwardKinematics(self.full_robot.model, self.full_robot.data, q)
                pin.updateGeometryPlacements(self.full_robot.model, self.full_robot.data, self.geom_model, self.geom_data, q)
                
                # 3. 更新可视化
                if self.visualize:
                    self.vis.display(q)
                
                # 4. 检测碰撞
                pin.computeCollisions(self.geom_model, self.geom_data, True)
                collision_detected = False
                # 5. 检查所有碰撞对
                collision_detected,collision_pair = self.collision_detect(q)
                if collision_detected:
                    print(f"\033[91m碰撞发生在: {collision_pair}\033[0m")
                    # if (collision_pair[0].startswith('l') and collision_pair[1].startswith('r')) or (collision_pair[0].startswith('r') and collision_pair[1].startswith('l')):
                    #     print(f'碰撞发生在左右臂之间')
                    time.sleep(5)
                    # time.sleep(5)
                else:
                    print(f"\033[92m无碰撞\033[0m")
                time.sleep(0.3)
                
        except KeyboardInterrupt:
            print("碰撞测试结束")
    

    def remove_collision_pair(self,geom_model,geom_data,start_joint_name:str,end_joint_name:str):
        for k in range(len(geom_model.collisionPairs)-1):
            cp = geom_model.collisionPairs[k]
            first_geom = geom_model.geometryObjects[cp.first].name  
            second_geom = geom_model.geometryObjects[cp.second].name
            if first_geom == start_joint_name and second_geom == end_joint_name:
                geom_model.removeCollisionPair(cp)
                break
        geom_data = pin.GeometryData(geom_model)
        return geom_data
    
    def _setup_target_visualizer(self):
        """为目标位姿设置一个可视化坐标轴。"""
        frame_viz_name = f'ee_target_{self.arm_prefix}'
        FRAME_AXIS_POSITIONS = np.array([[0,0,0],[1,0,0],[0,0,0],[0,1,0],[0,0,0],[0,0,1]], dtype=np.float32).T * 0.1
        FRAME_AXIS_COLORS = np.array([[1,0,0],[1,0.6,0],[0,1,0],[0.6,1,0],[0,0,1],[0,0.6,1]], dtype=np.float32).T
        self.vis.viewer[frame_viz_name].set_object(
            mg.LineSegments(
                mg.PointsGeometry(position=FRAME_AXIS_POSITIONS, color=FRAME_AXIS_COLORS),
                mg.LineBasicMaterial(linewidth=10, vertexColors=True)
            )
        )

    def _setup_optimization_problem(self):
        """配置逆运动学的优化问题。"""
        cmodel = cpin.Model(self.robot.model)
        cdata = cmodel.createData()

        cq = casadi.SX.sym("q", self.robot.model.nq, 1)
        cTf = casadi.SX.sym("tf", 4, 4)
        cpin.framesForwardKinematics(cmodel, cdata, cq)

        # 检查是否有end effector帧，如果没有则使用最后一个关节
        if self.robot.model.existFrame("ee"):
            gripper_id = self.robot.model.getFrameId("ee")
            error_func = casadi.Function(
                "error", [cq, cTf],
                [casadi.vertcat(cpin.log6(cdata.oMf[gripper_id].inverse() * cpin.SE3(cTf)).vector)]
            )
        else:
            # 使用最后一个关节作为end effector 
            last_joint_id = self.robot.model.njoints - 1
            
            print(f"No 'ee' frame found, using last joint (ID: {last_joint_id}) as end effector")
            
            # 创建基于关节位置的误差函数
            def create_joint_error_func():
                # 获取最后一个关节的变换矩阵
                joint_placement = cdata.oMi[last_joint_id]
                # 计算与目标位姿的误差
                error = casadi.vertcat(cpin.log6(joint_placement.inverse() * cpin.SE3(cTf)).vector)
                return casadi.Function("error", [cq, cTf], [error])
            
            error_func = create_joint_error_func()

        self.opti = casadi.Opti()
        var_q = self.opti.variable(self.robot.model.nq)
        param_tf = self.opti.parameter(4, 4)

        margin_ratio = 0.05  # 5%的边距
        lower_limits = self.robot.model.lowerPositionLimit
        upper_limits = self.robot.model.upperPositionLimit

        # 计算收缩后的限制
        joint_ranges = upper_limits - lower_limits
        safety_margin = joint_ranges * margin_ratio

        contracted_lower = lower_limits + safety_margin
        contracted_upper = upper_limits - safety_margin
        
        #位置误差函数
        error_vec = error_func(var_q, param_tf)
        pos_error = error_vec[:3]
        ori_error = error_vec[3:]
        
        # 成本函数：位置误差权重更高，姿态误差权重较低，加上正则化项以处理冗余
        totalcost = casadi.sumsqr(1.0 * pos_error) + casadi.sumsqr(0.1 * ori_error)
        regularization = casadi.sumsqr(var_q)
        self.opti.minimize(8 * totalcost + 0.01 * regularization )


        # 约束：关节角度限制
        self.opti.subject_to(self.opti.bounded(
            contracted_lower,
            var_q,
            contracted_upper
        ))

        opts = {'ipopt': {'print_level': 0, 'max_iter': 50, 'tol': 1e-4}, 'print_time': False}
        self.opti.solver("ipopt", opts)
        
        # 将内部变量保存为类成员
        self.var_q = var_q
        self.param_tf = param_tf

    def ik_fun(self, target_pose,init_joint_angles=None):
        """
        求解给定目标位姿的逆运动学。
        :param target_pose: 4x4的齐次变换矩阵。
        :return: (关节角度列表, 是否成功)
        """
        
        self.opti.set_initial(self.var_q, init_joint_angles) if init_joint_angles is not None else self.opti.set_initial(self.var_q, self.init_data)
        self.opti.set_value(self.param_tf, target_pose)
        if self.visualize:
            self.vis.viewer[f'ee_target_{self.arm_prefix}'].set_transform(target_pose)

        try:
            sol = self.opti.solve_limited()
            sol_q = self.opti.value(self.var_q)

            # 更新历史数据以实现平滑过渡
            max_diff = max(abs(self.history_data - sol_q)) if np.any(self.history_data) else 0
            if max_diff > 30.0 / 180.0 * np.pi:
                print(f"Warning: Large joint jump detected on arm {self.arm_prefix}. Resetting IK initial guess.")
                self.init_data = np.zeros(self.robot.model.nq)
            else:
                self.init_data = sol_q
            self.history_data = sol_q

            if self.visualize:
                self.vis.display(sol_q)

            # 暂时禁用碰撞检测
            is_collision = False
            return sol_q, not is_collision

        except Exception as e:
            print(f"ERROR in IK convergence for arm {self.arm_prefix}: {e}")
            return None, False


def main():
    ik_solver = Arm_IK(urdf_path="/home/zy/Project/jaka3/ROS2/jaka_ws/src/dual_arm/urdf/dual_arm.urdf", 
                       arm_prefix="r",
                       end_effector_link_name='rt',
                       visualize=True,
                       enable_collision_detection=True,
                       igno_coll_pairs=[['l6_0','l7_0'],['r6_0','r7_0']])
    print(f'Ik solver init')
    import time
    # while True:
    ik_solver.test_collision()
    # ik_solver.ik_fun(np.eye(4))
    time.sleep(1)
if __name__ == "__main__":
    main()