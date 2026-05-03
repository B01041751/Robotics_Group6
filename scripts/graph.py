import numpy as np
import matplotlib.pyplot as plt

rewards = np.load("episode_rewards.npy")
plt.plot(rewards)
plt.xlabel("Episode")
plt.ylabel("Reward")
plt.title("Training Performance")
plt.show()
