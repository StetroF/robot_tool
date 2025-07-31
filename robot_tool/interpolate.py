import numpy as np
from enum import Enum, auto
from typing import List, Tuple, Dict, Optional
visual = False
if visual:
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D

class InterpolateType:
    LINEAR = 0             # 直线插值
    BEZIER =1             # 贝塞尔曲线
    CIRCULAR_ARC = 2      # 圆弧插值
    CUBIC_POLY = 3         # 三次多项式
    QUINTIC_POLY = 4       # 五次多项式
    PARABOLIC = 5          # 抛物线插值


class PoseInterpolator:
    # 各类插值的默认参数
    DEFAULT_PARAMS = {
        InterpolateType.BEZIER: {'height_factor': 0.5},
        InterpolateType.CIRCULAR_ARC: {'arc_height': 0.5},
        InterpolateType.CUBIC_POLY: {'curvature': 1.0},
        InterpolateType.QUINTIC_POLY: {'curvature': 1.0},
        InterpolateType.PARABOLIC: {'height': 0.5}
    }
    
    def __init__(self):
        pass
    
    @staticmethod
    def _calculate_num_points(start_pos: np.ndarray, end_pos: np.ndarray, step_size: float) -> int:
        """计算需要插值的点数"""
        distance = np.linalg.norm(end_pos - start_pos)
        num_points = max(2, int(np.ceil(distance / step_size)))
        
        # 添加最大点数限制，防止生成过多插值点
        max_points = 1000  # 最大1000个点
        if num_points > max_points:
            print(f"警告：计算的点数({num_points})超过最大限制({max_points})，将使用最大限制")
            print(f"距离: {distance}, 步长: {step_size}")
            num_points = max_points
        
        return num_points
    
    @staticmethod
    def _interpolate_rotations(start_rot: np.ndarray, end_rot: np.ndarray, num_points: int) -> np.ndarray:
        """线性插值旋转部分"""
        t = np.linspace(0, 1, num_points)
        return start_rot + (end_rot - start_rot) * t[:, np.newaxis]
    
    @staticmethod
    def _linear_interpolation(start_pos: np.ndarray, end_pos: np.ndarray, num_points: int) -> np.ndarray:
        """直线插值"""
        t = np.linspace(0, 1, num_points)
        return start_pos + (end_pos - start_pos) * t[:, np.newaxis]
    
    @staticmethod
    def _bezier_interpolation(start_pos: np.ndarray, end_pos: np.ndarray, 
                            num_points: int, height_factor: float = 0.5) -> np.ndarray:
        """
        二次贝塞尔曲线插值
        :param height_factor: 控制点高度系数 (0-1之间，值越大弧度越大)
        """
        # 计算基础控制点 (中点)
        base_control = (start_pos + end_pos) / 2
        # 计算高度方向 (Z轴方向)
        height_dir = np.array([0, 0, 1])
        # 计算控制点位置
        control_point = base_control + height_dir * height_factor * np.linalg.norm(end_pos - start_pos)
        
        t = np.linspace(0, 1, num_points)
        t_matrix = np.vstack([(1-t)**2, 2*(1-t)*t, t**2]).T
        points = np.vstack([start_pos, control_point, end_pos])
        return np.dot(t_matrix, points)
    
    @staticmethod
    def _circular_arc_interpolation(start_pos: np.ndarray, end_pos: np.ndarray, 
                                   num_points: int, arc_height: float = 0.5) -> np.ndarray:
        """
        圆弧插值
        :param arc_height: 圆弧高度系数 (0-1之间，值越大弧度越大)
        """
        # 计算弦的中点
        chord_mid = (start_pos + end_pos) / 2
        chord_vec = end_pos - start_pos
        chord_len = np.linalg.norm(chord_vec)
        
        # 计算圆弧半径和圆心
        if arc_height <= 0:
            return PoseInterpolator._linear_interpolation(start_pos, end_pos, num_points)
        
        radius = (arc_height**2 + (chord_len/2)**2) / (2 * arc_height)
        center_dir = np.cross(chord_vec, np.array([0, 0, 1]))
        if np.linalg.norm(center_dir) < 1e-6:
            center_dir = np.cross(chord_vec, np.array([0, 1, 0]))
        
        center_dir = center_dir / np.linalg.norm(center_dir)
        center = chord_mid + center_dir * (radius - arc_height)
        
        # 计算起始和结束角度
        start_angle = np.arctan2(*(start_pos - center)[:2])
        end_angle = np.arctan2(*(end_pos - center)[:2])
        
        # 确保角度变化方向正确
        if end_angle < start_angle:
            end_angle += 2 * np.pi
        
        angles = np.linspace(start_angle, end_angle, num_points)
        points = []
        for angle in angles:
            x = center[0] + radius * np.cos(angle)
            y = center[1] + radius * np.sin(angle)
            # Z坐标线性插值
            z = center[2] + (angle - start_angle) / (end_angle - start_angle) * (end_pos[2] - start_pos[2])
            points.append([x, y, z])
        
        return np.array(points)
    
    @staticmethod
    def _cubic_poly_interpolation(start_pos: np.ndarray, end_pos: np.ndarray, 
                                num_points: int, curvature: float = 1.0) -> np.ndarray:
        """
        三次多项式插值
        :param curvature: 曲率系数 (值越大曲线越弯曲)
        """
        t = np.linspace(0, 1, num_points)
        # 三次多项式: a*t^3 + b*t^2 + c*t + d
        a = 2*curvature*(start_pos - end_pos)
        b = 3*curvature*(end_pos - start_pos)
        c = np.zeros_like(start_pos)
        d = start_pos
        return a*t[:, np.newaxis]**3 + b*t[:, np.newaxis]**2 + c*t[:, np.newaxis] + d
    
    @staticmethod
    def _quintic_poly_interpolation(start_pos: np.ndarray, end_pos: np.ndarray, 
                                   num_points: int, curvature: float = 1.0) -> np.ndarray:
        """
        五次多项式插值
        :param curvature: 曲率系数 (值越大曲线越弯曲)
        """
        t = np.linspace(0, 1, num_points)
        # 五次多项式: a*t^5 + b*t^4 + c*t^3 + d*t^2 + e*t + f
        a = 6*curvature*(end_pos - start_pos)
        b = -15*curvature*(end_pos - start_pos)
        c = 10*curvature*(end_pos - start_pos)
        d = np.zeros_like(start_pos)
        e = np.zeros_like(start_pos)
        f = start_pos
        return (a*t[:, np.newaxis]**5 + b*t[:, np.newaxis]**4 + 
                c*t[:, np.newaxis]**3 + d*t[:, np.newaxis]**2 + 
                e*t[:, np.newaxis] + f)
    
    @staticmethod
    def _parabolic_interpolation(start_pos: np.ndarray, end_pos: np.ndarray, 
                               num_points: int, height: float = 0.5) -> np.ndarray:
        """
        抛物线插值
        :param height: 抛物线高度系数 (值越大抛物线越高)
        """
        t = np.linspace(0, 1, num_points)
        # 抛物线: a*t^2 + b*t + c
        a = -4 * height * (end_pos - start_pos)
        b = 4 * height * (end_pos - start_pos)
        c = start_pos
        return a*t[:, np.newaxis]**2 + b*t[:, np.newaxis] + c
    
    @staticmethod
    def interpolate(
        start_pose: List[float], 
        end_pose: List[float], 
        step_size: float, 
        interp_type: InterpolateType = InterpolateType.LINEAR,
        params: Optional[Dict] = None
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        轨迹插值主函数
        
        参数:
            start_pose: 起始位姿 [x, y, z, rx, ry, rz]
            end_pose: 终止位姿 [x, y, z, rx, ry, rz]
            step_size: 步长大小 (决定插值点数)
            interp_type: 插值类型 (默认为直线插值)
            params: 插值参数字典 (不同类型有不同的参数)
            
        返回:
            Tuple: (插值后的位置数组, 插值后的旋转数组)
        """
        start_pos = np.array(start_pose[:3])
        end_pos = np.array(end_pose[:3])
        start_rot = np.array(start_pose[3:])
        end_rot = np.array(end_pose[3:])
        
        num_points = PoseInterpolator._calculate_num_points(start_pos, end_pos, step_size)
        
        # 合并用户参数和默认参数
        merged_params = PoseInterpolator.DEFAULT_PARAMS.get(interp_type, {}).copy()
        if params:
            merged_params.update(params)
        
        # 位置插值
        if interp_type == InterpolateType.LINEAR:
            positions = PoseInterpolator._linear_interpolation(start_pos, end_pos, num_points)
        elif interp_type == InterpolateType.BEZIER:
            positions = PoseInterpolator._bezier_interpolation(
                start_pos, end_pos, num_points, **merged_params)
        elif interp_type == InterpolateType.CIRCULAR_ARC:
            positions = PoseInterpolator._circular_arc_interpolation(
                start_pos, end_pos, num_points, **merged_params)
        elif interp_type == InterpolateType.CUBIC_POLY:
            positions = PoseInterpolator._cubic_poly_interpolation(
                start_pos, end_pos, num_points, **merged_params)
        elif interp_type == InterpolateType.QUINTIC_POLY:
            positions = PoseInterpolator._quintic_poly_interpolation(
                start_pos, end_pos, num_points, **merged_params)
        elif interp_type == InterpolateType.PARABOLIC:
            positions = PoseInterpolator._parabolic_interpolation(
                start_pos, end_pos, num_points, **merged_params)
        else:
            raise ValueError(f"Unsupported interpolation type: {interp_type}")
        
        # 旋转插值 (使用线性插值)
        rotations = PoseInterpolator._interpolate_rotations(start_rot, end_rot, num_points)
        
        return positions, rotations

def plot_trajectory(positions: np.ndarray, title: str, params: Optional[Dict] = None):
    """使用Matplotlib绘制3D轨迹"""
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    # 绘制轨迹
    ax.plot(positions[:, 0], positions[:, 1], positions[:, 2], 
            'b-', linewidth=2, label='轨迹')
    ax.scatter(positions[0, 0], positions[0, 1], positions[0, 2], 
               c='g', s=100, label='起点')
    ax.scatter(positions[-1, 0], positions[-1, 1], positions[-1, 2], 
               c='r', s=100, label='终点')
    
    # 添加控制点或参数信息
    if "贝塞尔" in title:
        control_point = (positions[0] + positions[-1]) / 2 + np.array([0, 0, params.get('height_factor', 0.5)])
        ax.scatter(control_point[0], control_point[1], control_point[2], 
                   c='m', s=100, marker='s', label='控制点')
    
    if params:
        param_text = '\n'.join([f"{k}: {v:.2f}" for k, v in params.items()])
        ax.text2D(0.05, 0.95, param_text, transform=ax.transAxes,
                 bbox=dict(facecolor='white', alpha=0.7))
    
    # 设置图形属性
    ax.set_xlabel('X轴')
    ax.set_ylabel('Y轴')
    ax.set_zlabel('Z轴')
    ax.set_title(title)
    ax.legend()
    
    # 设置等比例坐标轴
    max_range = np.array([positions[:, 0].max()-positions[:, 0].min(), 
                          positions[:, 1].max()-positions[:, 1].min(), 
                          positions[:, 2].max()-positions[:, 2].min()]).max() / 2.0
    
    mid_x = (positions[:, 0].max()+positions[:, 0].min()) * 0.5
    mid_y = (positions[:, 1].max()+positions[:, 1].min()) * 0.5
    mid_z = (positions[:, 2].max()+positions[:, 2].min()) * 0.5
    
    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)
    
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    # 定义起始和结束位姿
    start_pose = [0, 0, 0, 0, 0, 0]
    end_pose = [1, 10, 0, 0.5, 0.5, 0.5]  # 终点Z坐标为0，使曲线更明显
    step_size = 0.05
    
    # 测试所有插值方法并绘制结果
    interpolation_types = [
        (InterpolateType.LINEAR, "直线插值", {}),
        (InterpolateType.BEZIER, "贝塞尔曲线插值", {'height_factor': 0.5}),
        (InterpolateType.CIRCULAR_ARC, "圆弧插值", {'arc_height': 0.5}),
        # (InterpolateType.CUBIC_POLY, "三次多项式插值", {'curvature': 1.5}),
        # (InterpolateType.QUINTIC_POLY, "五次多项式插值", {'curvature': 10}),
        # (InterpolateType.PARABOLIC, "抛物线插值", {'height': 0.9})
    ]
    
    for interp_type, title, params in interpolation_types:
        positions, rotations = PoseInterpolator.interpolate(
            start_pose, end_pose, step_size, interp_type, params
        )
        print(f"\n{title} - 插值点数: {len(positions)}")
        # plot_trajectory(positions, title, params)