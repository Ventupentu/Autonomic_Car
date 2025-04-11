import time
import rclpy
from stable_baselines3 import PPO
from rl_driver.ros2_env import Ros2CarEnv

def main():
    """
    Main function to run the reinforcement learning agent in a ROS2 environment.

    This script initializes the ROS2 environment, loads a pre-trained PPO model,
    and continuously interacts with the environment by predicting actions and
    executing them. If an episode ends, the environment is reset, and the process
    starts again.
    """
    # Initialize the ROS2 environment
    env = Ros2CarEnv()

    # Load the pre-trained PPO model
    # Ensure the model file "ros2_car_model3" is in the correct path
    model = PPO.load("ros2_car_model2")

    # Reset the environment to get the initial observation
    obs = env.reset()
    done = False

    while rclpy.ok():
        # Get the action from the model based on the current observation
        action, _ = model.predict(obs, deterministic=True)

        # Execute the action in the environment and get the next state and reward
        obs, reward, done, info = env.step(action)

        # (Optional) Print reward and observation for debugging
        # print(f"Reward: {reward:.2f} | Obs: {obs}")

        # If the episode is done, reset the environment
        if done:
            print("Episodio terminado. Reiniciando...")
            obs = env.reset()

        # Small pause to avoid overloading the system
        time.sleep(0.05)

if __name__ == "__main__":
    main()
