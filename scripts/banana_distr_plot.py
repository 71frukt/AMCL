import numpy as np
import matplotlib.pyplot as plt

np.random.seed(42)
particles = 15000

x = np.zeros(particles)
y = np.zeros(particles)
theta = np.zeros(particles)

steps = 20
step_dist = 1.0

sigma_d = 0.05
sigma_theta = 0.04

for _ in range(steps):
    d = np.random.normal(step_dist, sigma_d, particles)
    d_theta = np.random.normal(0, sigma_theta, particles)
    
    theta += d_theta
    x += d * np.cos(theta)
    y += d * np.sin(theta)

fig, ax = plt.subplots(figsize=(10, 6))
ax.scatter(x, y, s=1, alpha=0.05, color='blue')
ax.plot(steps * step_dist, 0, 'rX', markersize=10)

ax.set_title('Эффект накопления угловой ошибки (банановидное распределение)')

# Настройка осей для старта ровно из начала координат
ax.spines['left'].set_position('zero')
ax.spines['bottom'].set_position('zero')
ax.spines['right'].set_visible(False)
ax.spines['top'].set_visible(False)

ax.set_aspect('equal')
ax.set_xlim(left=0)

plt.grid(True, linestyle='--', alpha=0.6)

plt.savefig('image3.png', dpi=300, bbox_inches='tight')