import gym
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray  # O usa tu propio tipo de mensaje
from rclpy.executors import SingleThreadedExecutor

class Ros2CarEnv(gym.Env):

    def __init__(self):
        super(Ros2CarEnv, self).__init__()

        # Inicia ROS2 y el nodo
        rclpy.init()
        self.node = Node("rl_env_node")
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)

        # Observación: [x1, y1, x2, y2]
        self.observation_space = gym.spaces.Box(low=-10.0, high=10.0, shape=(4,), dtype=np.float32)

        # Acción: [giro, aceleración]
        self.action_space = gym.spaces.Box(low=np.array([-1.0, 0.0]), high=np.array([1.0, 1.0]), dtype=np.float32)

        # Suscripción a conos
        self.current_obs = np.zeros(4, dtype=np.float32)
        self.done = False

        self.node.create_subscription(
            Float32MultiArray,
            "/vision/cones",  # cambia si usas otro topic
            self.vision_callback,
            10
        )

        # Publicador de comandos al coche
        self.cmd_pub = self.node.create_publisher(Twist, "/cmd_vel", 10)

    def vision_callback(self, msg):
        # Suponemos msg.data = [x1, y1, x2, y2, ...] (tú ajusta esto)
        if len(msg.data) >= 4:
            self.current_obs = np.array(msg.data[:4], dtype=np.float32)

    def step(self, action):
        # Acción: [steering, acceleration]
        twist = Twist()
        twist.angular.z = float(action[0])  # giro
        twist.linear.x = float(action[1])   # velocidad
        self.cmd_pub.publish(twist)

        # Esperamos a que ROS procese
        rclpy.spin_once(self.node, timeout_sec=0.05)

        # Reward muy simple: avanzar sin colisión
        reward = action[1] * 10.0  # avanzar más = mejor
        done = self.done  # más adelante: añade colisiones, salirse de pista...

        return self.current_obs, reward, done, {}

    def reset(self):
        self.current_obs = np.zeros(4, dtype=np.float32)
        self.done = False

        # Aquí podrías reiniciar la posición del coche con un servicio
        rclpy.spin_once(self.node, timeout_sec=0.1)
        return self.current_obs

    def close(self):
        self.executor.remove_node(self.node)
        self.node.destroy_node()
        rclpy.shutdown()
