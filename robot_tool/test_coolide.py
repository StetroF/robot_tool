import numpy as np
import sys, os
import time
import pinocchio as pin
import meshcat
import meshcat.geometry as g
import hppfcl

# 初始化路径
home_dir = os.path.expanduser("~")
conda_env_path = os.environ.get("CONDA_PREFIX", os.path.join(home_dir, "miniconda3", "envs", "humble"))
if conda_env_path:
    conda_site_packages = os.path.join(conda_env_path, "lib", f"python{sys.version_info.major}.{sys.version_info.minor}", "site-packages")
    if conda_site_packages not in sys.path:
        sys.path.append(conda_site_packages)

# 加载 URDF 模型
urdf_path = "/home/zy/Project/jaka3/ROS2/jaka_ws/src/dual_arm/urdf/dual_arm.urdf"
# model = pin.buildModelFromUrdf(urdf_path)
full_robot = pin.RobotWrapper.BuildFromURDF(urdf_path)
joint_names = [f"r-j{i+1}" for i in range(7)]
joint_names.extend([f"l-j{i+1}" for i in range(7)])
joints_to_actuate_ids = []
for joint_name in joint_names:
    if full_robot.model.existJointName(joint_name):
        joint_id = full_robot.model.getJointId(joint_name)
        joints_to_actuate_ids.append(joint_id)
    else:
        print(f"Joint {joint_name} not found in model")
        raise ValueError(f"Joint {joint_name} not found in model")
all_joint_ids = list(range(1, full_robot.model.njoints))  # 跳过universe joint (id=0)
joints_to_lock = [jid for jid in all_joint_ids if jid not in joints_to_actuate_ids]
q0 = pin.neutral(full_robot.model)
robot = full_robot.buildReducedRobot(
    list_of_joints_to_lock=joints_to_lock,
    reference_configuration=q0
)
###为左臂和右臂都添加末端执行器
for arm in ["r","l"]:
    last_joint_name = f"{arm}-j6"
    tool_frame_id_in_full_model = full_robot.model.getFrameId(last_joint_name)
    tool_placement = full_robot.model.frames[tool_frame_id_in_full_model].placement
    last_joint_id = robot.model.getJointId(last_joint_name)
    robot.model.addFrame(
        pin.Frame('ee',
                  last_joint_id,
                  tool_placement,
                  pin.FrameType.OP_FRAME) # type: ignore
        )


geom_model = pin.buildGeomFromUrdf(robot.model, urdf_path, pin.GeometryType.COLLISION)
geom_model_viz = pin.buildGeomFromUrdf(robot.model, urdf_path, pin.GeometryType.VISUAL)
geom_model.addAllCollisionPairs()






# 创建数据对象
data = robot.model.createData()
geom_data = pin.GeometryData(geom_model)

# 初始化 MeshCat
vis = meshcat.Visualizer().open()
print("MeshCat 可视化已启动，请访问: http://localhost:7000")

# 清除之前的可视化内容
vis.delete()

# 添加机器人几何体到可视化
for i, geom in enumerate(geom_model_viz.geometryObjects):
    geom_name = geom.name
    geometry = geom.geometry
    if geom_name == "l-ee" or geom_name == "r-ee":
        vis[geom_name].set_object(g.Cylinder(0.01, 0.05))
        vis[geom_name].set_property("color", [0, 0, 1, 1])
        continue
    if isinstance(geometry, hppfcl.Box):
        vis[geom_name].set_object(g.Box(geometry.halfSide * 2))
    elif isinstance(geometry, hppfcl.Cylinder):
        vis[geom_name].set_object(g.Cylinder(geometry.radius, geometry.halfLength * 2))
    elif isinstance(geometry, hppfcl.Sphere):
        vis[geom_name].set_object(g.Sphere(geometry.radius))
    elif isinstance(geometry, hppfcl.BVHModelOBBRSS):
        vis[geom_name].set_object(g.Box([0.1, 0.1, 0.1]))
    elif isinstance(geometry, hppfcl.BVHModelOBBRSS):
        vis[geom_name].set_object(g.Box([0.1, 0.1, 0.1]))
    ##末端执行器显示为蓝色圆锥
    
    vis[geom_name].set_property("color", [0, 1, 0, 0.5])  # 初始绿色

# 相机设置
def get_camera_transform():
    translation = np.array([1.0, 0.0, 0.5])
    ry = np.pi/2
    R = np.array([[np.cos(ry), 0, np.sin(ry)], [0, 1, 0], [-np.sin(ry), 0, np.cos(ry)]])
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = translation
    return T

vis["/Cameras/default"].set_transform(get_camera_transform())

print("开始碰撞检测测试，按 Ctrl+C 停止...")
##禁用l6_0和l7_0的碰撞检测
def remove_collision_pair(geom_model,geom_data,start_joint,end_joint):
    for k in range(len(geom_model.collisionPairs)-1):
        cp = geom_model.collisionPairs[k]
        collision_detected = True
        first_geom = geom_model.geometryObjects[cp.first].name  
        second_geom = geom_model.geometryObjects[cp.second].name
        if first_geom == start_joint and second_geom == end_joint:
            geom_model.removeCollisionPair(cp)
            break
    geom_data = pin.GeometryData(geom_model)
    return geom_data


geom_data = remove_collision_pair(geom_model,geom_data,"l6_0","l7_0")
geom_data = remove_collision_pair(geom_model,geom_data,"r6_0","r7_0")

import time
try:
    while True:
        # 随机生成关节角度
        q = pin.randomConfiguration(robot.model)
        start_time = time.time()

        # 更新运动学和几何位置
        pin.forwardKinematics(robot.model, data, q)
        pin.updateGeometryPlacements(robot.model, data, geom_model, geom_data, q)
        pin.updateGeometryPlacements(robot.model, data, geom_model_viz, geom_data, q)

        # 更新可视化
        for i, geom in enumerate(geom_model_viz.geometryObjects):
            vis[geom.name].set_transform(geom_data.oMg[i].homogeneous)

        # 重置所有几何体为绿色
        for geom in geom_model_viz.geometryObjects:
            vis[geom.name].set_property("color", [0, 1, 0, 0.5])

        # 检测碰撞
        pin.computeCollisions(geom_model, geom_data, True)
        end_time = time.time()
        print(f'碰撞检测时间: {end_time - start_time} 秒')
        collision_detected = False
        # print(f'geom_model.collisionPairs: {geom_model.collisionPairs[0]}')
        # 检查所有碰撞对
        for k in range(len(geom_model.collisionPairs)):
            cr = geom_data.collisionResults[k]
            cp = geom_model.collisionPairs[k]
            if cr.isCollision():
                collision_detected = True
                # 获取碰撞对中的几何体名称
                first_geom = geom_model.geometryObjects[cp.first].name
                second_geom = geom_model.geometryObjects[cp.second].name
                
                # 将碰撞的几何体标红
                vis[first_geom].set_property("color", [1, 0, 0, 1])
                vis[second_geom].set_property("color", [1, 0, 0, 1])
                
                print(f"\033[91m碰撞发生在: {first_geom} 和 {second_geom}\033[0m")
                time.sleep(5)
        if not collision_detected:
            print(f"\033[92m无碰撞\033[0m")

        time.sleep(1)

except KeyboardInterrupt:
    print("程序终止")
    vis.delete()