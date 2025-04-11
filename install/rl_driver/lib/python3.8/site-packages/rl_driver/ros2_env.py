import gym
from gym import spaces
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String, Bool
from geometry_msgs.msg import Point
from auria_msgs.msg import PointArray

class Ros2CarEnv(gym.Env):
    """
    Ros2CarEnv is a custom OpenAI Gym environment for simulating a car in a ROS2-based environment. 
    It interacts with ROS2 topics to control the car and receive feedback about its state.
    Attributes:
        node (Node): ROS2 node for the environment.
        steer_pub (Publisher): Publisher for steering commands.
        accel_pub (Publisher): Publisher for acceleration commands.
        reset_pub (Publisher): Publisher for resetting the car's state.
        car_position (np.array): Current position of the car [x, y].
        speed (float): Current speed of the car.
        yaw (float): Current yaw (orientation) of the car.
        cones_blue (list): List of blue cone positions.
        cones_yellow (list): List of yellow cone positions.
        steps (int): Number of steps taken in the current episode.
        no_progress_steps (int): Number of consecutive steps without progress.
        speed_limit (float): Maximum allowed speed for the car.
        action_space (spaces.Box): Action space defining steering and acceleration limits.
        observation_space (spaces.Box): Observation space defining the range of observations.
    Methods:
        __init__(): Initializes the environment, ROS2 node, publishers, and subscribers.
        _speed_cb(msg): Callback for updating the car's speed from the "/can/speed" topic.
        _position_cb(msg): Callback for updating the car's position from the "/env/car_position" topic.
        _cones_blue_cb(msg): Callback for updating the positions of blue cones.
        _cones_yellow_cb(msg): Callback for updating the positions of yellow cones.
        _yaw_cb(msg): Callback for updating the car's yaw from the "/env/car_yaw" topic.
        reset(): Resets the environment to its initial state and waits for the car to stabilize.
        step(action): Executes a step in the environment by applying the given action.
        _get_obs(): Constructs the observation vector based on the car's state and environment.
        _compute_reward(steer): Computes the reward for the current step based on the car's behavior.
        _get_nearest_cone(cone_list): Finds the nearest cone from a given list of cones.
        _is_done(): Checks if the episode is done based on various termination conditions.
        close(): Cleans up the ROS2 node and shuts down the environment.
    """
    def __init__(self):
        super(Ros2CarEnv, self).__init__()

        rclpy.init(args=None)
        self.node = Node("ros2_car_env")

        # Publishers
        self.steer_pub = self.node.create_publisher(Float32, "/car/steering", 10)
        self.accel_pub = self.node.create_publisher(Float32, "/car/acceleration", 10)
        self.reset_pub = self.node.create_publisher(Bool, "/reset_car", 10)

        # Subscribers
        self.node.create_subscription(Float32, "/can/speed", self._speed_cb, 10)
        self.node.create_subscription(Point, "/env/car_position", self._position_cb, 10)
        self.node.create_subscription(PointArray, "/vision/all_cones/blue", self._cones_blue_cb, 10)
        self.node.create_subscription(PointArray, "/vision/all_cones/yellow", self._cones_yellow_cb, 10)
        self.node.create_subscription(Float32, "/env/car_yaw", self._yaw_cb, 10)


        # Initial state
        self.car_position = np.array([0.0, 0.0])
        self.speed = 0.0
        self.yaw = 0.0
        self.cones_blue = []
        self.cones_yellow = []

        self.steps = 0
        self.no_progress_steps = 0
        self.speed_limit = 3.0  

        # Action: [steering, acceleration]
        self.action_space = spaces.Box(low=np.array([-1.0, -1.0]), high=np.array([1.0, 1.0]), dtype=np.float32)

        # Observation: [x, y, speed]
        self.observation_space = spaces.Box(
            low=np.array([-20, -20, 0, -10, -10, -1]),
            high=np.array([20, 20, 10, 10, 10, 1]),
            dtype=np.float32
)



    def _speed_cb(self, msg):
        self.speed = msg.data

    def _position_cb(self, msg):
        self.car_position = np.array([msg.x, msg.y])

    def _cones_blue_cb(self, msg):
        self.cones_blue = msg.points

    def _cones_yellow_cb(self, msg):
        self.cones_yellow = msg.points
    
    def _yaw_cb(self, msg):
        self.yaw = msg.data


    def reset(self):
        self.steps = 0
        self.no_progress_steps = 0
        self.prev_position = np.array([0.0, 0.0])
        self.prev_speed = 0.0
        self.cones_blue = []
        self.cones_yellow = []

        reset_msg = Bool()
        reset_msg.data = True
        self.reset_pub.publish(reset_msg)

        for i in range(50):
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if np.linalg.norm(self.car_position) < 0.5:
                break

        return self._get_obs()

    def step(self, action):
        steer, accel = float(action[0]), float(action[1])

        self.steer_pub.publish(Float32(data=steer))
        self.accel_pub.publish(Float32(data=accel))

        rclpy.spin_once(self.node, timeout_sec=0.1)

        obs = self._get_obs()
        reward = self._compute_reward(steer=steer)
        self.steps += 1
        done = self._is_done()
        info = {}

        self.prev_position = self.car_position.copy()
        self.prev_speed = self.speed

        return obs, reward, done, info

    def _get_obs(self):
        obs = [self.car_position[0], self.car_position[1], self.speed]

        # Lane center
        if self.cones_blue and self.cones_yellow:
            blue = self._get_nearest_cone(self.cones_blue)
            yellow = self._get_nearest_cone(self.cones_yellow)
            center = (blue + yellow) / 2
            offset = self.car_position - center
            obs.extend(offset)
        else:
            obs.extend([0.0, 0.0])

        # Relative direction to the yellow cone
        if self.cones_yellow:
            nearest_yellow = self._get_nearest_cone(self.cones_yellow)
            car_direction = np.array([np.cos(self.yaw), np.sin(self.yaw)])
            to_target = nearest_yellow - self.car_position
            to_target /= np.linalg.norm(to_target + 1e-8)
            alignment = np.clip(np.dot(car_direction, to_target), -1.0, 1.0)
            obs.append(alignment)
        else:
            obs.append(0.0)

        return np.array(obs, dtype=np.float32)



    def _compute_reward(self, steer=0.0):
        reward = 0.0

        # Actual progress in the direction of yaw
        direction_vector = np.array([np.cos(self.yaw), np.sin(self.yaw)])
        progress = np.dot(self.car_position - self.prev_position, direction_vector)

        # 1. Reward for actual forward progress
        if progress > 0:
            reward += progress * 3.0
        else:
            reward -= 2.0  # Penalize reverse or self-rotations

        # 2. Penalty for excessive steering without progress
        if abs(steer) > 0.8 and progress < 0.05:
            reward -= 1.0

        # 3. Penalty for exceeding speed limit
        if self.speed > self.speed_limit:
            reward -= (self.speed - self.speed_limit) * 3.0

        # 4. Penalty for deviating from the center
        if self.cones_blue and self.cones_yellow:
            blue = self._get_nearest_cone(self.cones_blue)
            yellow = self._get_nearest_cone(self.cones_yellow)
            center = (blue + yellow) / 2
            distance = np.linalg.norm(self.car_position - center)
            reward -= distance * 5.0
        
        # 5. Penalty for speed reduction
        if self.speed < self.prev_speed:
            deceleration = self.prev_speed - self.speed
            reward -= deceleration * 2.0  # You can adjust this weight

        # 6. Penalty for sharp turns
        if abs(steer) > 0.8:
            reward -= (abs(steer) - 0.8) * 5.0

        return reward

    def _get_nearest_cone(self, cone_list):
        if not cone_list:
            return np.array([0.0, 0.0])
        cone_coords = np.array([[c.x, c.y] for c in cone_list])
        distances = np.linalg.norm(cone_coords - self.car_position, axis=1)
        return cone_coords[np.argmin(distances)]

    def _is_done(self):
        if self.steps >= 30000:
            #self.node.get_logger().info("🟧 Episodio terminado: límite de pasos alcanzado.")
            return True

        if self.steps > 0 and self._compute_reward() < -20:
            #self.node.get_logger().info("🟨 Episodio terminado: recompensa baja.")
            return True

        if self.speed < 0.05:
            self.no_progress_steps += 1
        else:
            self.no_progress_steps = 0

        if self.no_progress_steps >= 30:
            #self.node.get_logger().info("🟥 Episodio terminado: el coche lleva parado mucho tiempo.")
            return True

        return False

    def close(self):
        self.node.destroy_node()
        rclpy.shutdown()
