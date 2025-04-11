
import gymnasium as gym
from gymnasium import spaces
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from auria_msgs.msg import PointArray
from geometry_msgs.msg import Point

class Ros2CarEnv(gym.Env):
    def __init__(self):
        super(Ros2CarEnv, self).__init__()

        # Inicializar ROS 2 si no se ha hecho
        rclpy.init(args=None)

        self.node = Node('ros2_car_env')

        # Publicadores
        self.steering_pub = self.node.create_publisher(Float32, '/car/steering', 10)
        self.accel_pub = self.node.create_publisher(Float32, '/car/acceleration', 10)
        self.state_pub = self.node.create_publisher(String, '/car/state', 10)

        # Suscriptores
        self.cones_sub = self.node.create_subscription(PointArray, '/vision/all_cones/blue', self.cones_callback, 10)
        self.speed_sub = self.node.create_subscription(Float32, '/can/speed', self.speed_callback, 10)
        self.car_position_sub = self.node.create_subscription(Point, '/env/car_position', self.position_callback, 10)

        # Espacios
        self.action_space = spaces.Box(low=np.array([-1.0, -1.0]), high=np.array([1.0, 1.0]), dtype=np.float32)
        self.observation_space = spaces.Box(low=np.array([-10.0, -10.0, -5.0]), high=np.array([10.0, 10.0, 5.0]), dtype=np.float32)

        self.car_position = np.array([0.0, 0.0])
        self.speed = 0.0
        self.cones = []

    def cones_callback(self, msg):
        self.cones = msg.points

    def speed_callback(self, msg):
        self.speed = msg.data

    def position_callback(self, msg):
        self.car_position = np.array([msg.x, msg.y])

    def _get_obs(self):
        return np.concatenate([self.car_position, [self.speed]])

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)

        self.car_position = np.array([0.0, 0.0])
        self.speed = 0.0
        self.cones = []

        state_msg = String()
        state_msg.data = 'start'
        self.state_pub.publish(state_msg)

        if seed is not None:
            np.random.seed(seed)

        rclpy.spin_once(self.node, timeout_sec=0.1)

        return self._get_obs()  # Devuelve solo obs, compatible con SB3

    def step(self, action):
        steering, acceleration = action

        self.steering_pub.publish(Float32(data=steering))
        self.accel_pub.publish(Float32(data=acceleration))

        rclpy.spin_once(self.node, timeout_sec=0.1)

        obs = self._get_obs()
        reward = -self.get_distance_to_nearest_cone()

        terminated = False
        truncated = False
        if self.car_position[0] > 10.0:
            terminated = True
            self.state_pub.publish(String(data='finish'))

        done = terminated or truncated
        info = {}

        return obs, reward, done, info

    def get_distance_to_nearest_cone(self):
        if not self.cones:
            return 0.0
        distances = np.linalg.norm(np.array([[c.x, c.y] for c in self.cones]) - self.car_position, axis=1)
        return np.min(distances)

    def close(self):
        self.node.destroy_node()
        rclpy.shutdown()
