import matplotlib.pyplot as plt
from stable_baselines3.common.callbacks import BaseCallback

class LiveRewardPlotCallback(BaseCallback):
    def __init__(self, check_freq=1, verbose=0):
        super().__init__(verbose)
        self.check_freq = check_freq
        self.rewards = []
        self.iterations = []
        self.counter = 0

        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.line, = self.ax.plot([], [], label='ep_rew_mean')
        self.ax.set_xlabel('Iterations')
        self.ax.set_ylabel('Reward')
        self.ax.set_title('Recompensa media por episodio')
        self.ax.grid(True)
        self.ax.legend()

    def _on_step(self) -> bool:
        return True  # No usamos pasos, solo checkpoints

    def _on_rollout_end(self) -> None:
        self.counter += 1
        reward = self.logger.name_to_value.get("rollout/ep_rew_mean", None)
        if reward is not None:
            self.rewards.append(reward)
            self.iterations.append(self.counter)
            self.line.set_data(self.iterations, self.rewards)
            self.ax.relim()
            self.ax.autoscale_view()
            plt.draw()
            plt.pause(0.001)
