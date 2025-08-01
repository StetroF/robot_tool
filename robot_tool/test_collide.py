import numpy as np
import sys,os
home_dir = os.path.expanduser("~")
conda_env_path = os.environ.get("CONDA_PREFIX",os.path.join(home_dir,"miniconda3","envs","humble"))
if conda_env_path:
    conda_site_packages = os.path.join(conda_env_path, "lib", f"python{sys.version_info.major}.{sys.version_info.minor}", "site-packages")
    print(f"conda path: {conda_site_packages}")
    if conda_site_packages not in sys.path:
        sys.path.append(conda_site_packages)


import pinocchio as pin
import numpy as np

# 加载 URDF 模型（包含碰撞几何）
model = pin.buildModelFromUrdf("/home/zy/Project/jaka3/ROS2/jaka_ws/src/dual_arm/urdf/dual_arm.urdf")
geom_model = pin.buildGeomFromUrdf(model, "/home/zy/Project/jaka3/ROS2/jaka_ws/src/dual_arm/urdf/dual_arm.urdf", pin.GeometryType.COLLISION)

# 创建数据对象
data = model.createData()
geom_data = pin.GeometryData(geom_model)

# 设置机器人位姿
q = np.random.rand(model.nq)
print(q)
pin.forwardKinematics(model, data, q)
pin.updateGeometryPlacements(model, data, geom_model, geom_data, q)

# 检测自碰撞
collision_pairs = pin.computeCollisions(geom_model, geom_data, True)  # 检查所有几何体对
print(collision_pairs)
# for pair, is_colliding in collision_pairs.items():
#     print(f"几何体 {pair.first} 和 {pair.second} 是否碰撞: {is_colliding}")