from stable_baselines3 import PPO
from rl_driver.ros2_env import Ros2CarEnv
import matplotlib.pyplot as plt
from stable_baselines3.common.callbacks import BaseCallback

def main():
    """
    Main function to train a reinforcement learning model using the PPO algorithm 
    in a ROS2-based car environment.

    The function initializes the environment, resets it to ensure a clean state, 
    and sets up the PPO model with specified hyperparameters. The model is then 
    trained for a total of 1,000,000 timesteps, with a progress bar displayed 
    during training. After training, the model is saved to a file.
    """
    env = Ros2CarEnv()

    print("Reiniciando entorno antes de comenzar el entrenamiento...")
    env.reset()

    model = PPO(
        "MlpPolicy",
        env,
        verbose=1,
        learning_rate=3e-4,
        n_steps=2048,
        batch_size=64,
        n_epochs=10,
        ent_coef=0.01,
        gae_lambda=0.95,
        clip_range=0.2
    )
    model.learn(total_timesteps=1000000, progress_bar=True)
    model.save("ros2_car_model3")
    print("Entrenamiento completado y modelo guardado.")

if __name__ == "__main__":
    main()
