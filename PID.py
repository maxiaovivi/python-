import numpy as np
import matplotlib.pyplot as plt
plt.rcParams['font.sans-serif'] = ['SimHei']  # 'SimHei' 是一种支持中文的字体
plt.rcParams['axes.unicode_minus'] = False  # 正确显示负号
# 小车模型参数
class CarModel:
    def __init__(self):
        self.theta = 0  # 初始朝向角度
        self.angular_velocity = 0  # 角速度

    def update(self, torque, dt):
        # 添加执行器噪声
        torque_noise = np.random.normal(0, 0.1)  # 假设扭矩噪声的标准差为0.1
        self.angular_velocity += torque + torque_noise
        
        # 添加外部扰动
        external_disturbance = np.random.normal(0, 0.02)  # 假设外部扰动的标准差为0.02
        
        # 更新角度，添加传感器噪声
        sensor_noise = np.random.normal(0, 0.01)  # 假设传感器噪声的标准差为0.01
        self.theta += (self.angular_velocity * dt + external_disturbance) + sensor_noise
        return self.theta

# PID控制器
class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.previous_error = 0
        self.integral = 0

    def control(self, target, current, dt):
        error = target - current
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.previous_error = error
        return output

# 设置PID参数
Kp = 1
Ki = 0.01
Kd = 0.6

# 创建小车和控制器对象
car = CarModel()
pid = PIDController(Kp, Ki, Kd)

# 仿真参数
dt = 0.01  # 时间步长
total_time = 10  # 总仿真时间
target_theta = np.pi   # 目标朝向（90度）

# 运行仿真
time_points = np.arange(0, total_time, dt)
theta_points = []
for t in time_points:
    torque = pid.control(target_theta, car.theta, dt)
    theta = car.update(torque, dt)
    theta_points.append(theta)

# 计算超调量、峰值时间、稳态时间
theta_points = np.array(theta_points)
peak_time = time_points[np.argmax(theta_points)]
overshoot = (np.max(theta_points) - target_theta) / target_theta * 100  # 超调量百分比
settling_time_index = np.where(np.abs(theta_points - target_theta) <= 0.05 * target_theta)[0]
settling_time = time_points[settling_time_index[0]] if settling_time_index.size > 0 else None

print("overshoot: ",overshoot,"%\n")
print("peak_time: ",peak_time,"\n")
# 绘制结果
plt.figure(figsize=(10, 5))
plt.plot(time_points, theta_points, label="小车朝向")
plt.plot(time_points, [target_theta] * len(time_points), 'r--', label="目标朝向")
plt.scatter([peak_time], [theta_points[np.argmax(theta_points)]], color='green', label='峰值时间')
if settling_time:
    plt.scatter([settling_time], [theta_points[settling_time_index[0]]], color='blue', label='稳态时间')
plt.xlabel('时间', fontproperties="SimSun")
plt.ylabel('朝向角度', fontproperties="SimSun")
plt.title('小车朝向随时间的变化', fontproperties="SimSun")
plt.legend()
plt.grid(True)
plt.show()

peak_time, overshoot, settling_time
