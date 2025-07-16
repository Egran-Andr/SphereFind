import matplotlib.pyplot as plt
import numpy as np
from matplotlib import rcParams

rcParams['font.family'] = 'serif'


def sphereFit(spX, spY, spZ):
    spX = np.array(spX)
    spY = np.array(spY)
    spZ = np.array(spZ)
    A = np.zeros((len(spX), 4))
    A[:, 0] = spX * 2
    A[:, 1] = spY * 2
    A[:, 2] = spZ * 2
    A[:, 3] = 1

    f = np.zeros((len(spX), 1))
    f[:, 0] = (spX * spX) + (spY * spY) + (spZ * spZ)
    C, residules, rank, singval = np.linalg.lstsq(A, f)

    t = (C[0] * C[0]) + (C[1] * C[1]) + (C[2] * C[2]) + C[3]
    radius = np.sqrt(t)

    return radius, C[0], C[1], C[2]


# Загрузка информации о координатах точек X,Y,Z в формате npy файлов, для просмотра содержимого можно запустить скрипт в папке NPYViewer
correctX = np.load('x.npy')
correctY = np.load('y.npy')
correctZ = np.load('z.npy')


r, x0, y0, z0 = sphereFit(correctX, correctY, correctZ)

# Вывод координат центра и радиуса сферы в консоль
print(f"Sphere center coordinates: ({x0[0]:.2f}, {y0[0]:.2f}, {z0[0]:.2f})")
print(f"Sphere radius: {r[0]:.4f}")

# Визуализация в виде графика
u, v = np.mgrid[0:2 * np.pi:20j, 0:np.pi:10j]
x = np.cos(u) * np.sin(v) * r
y = np.sin(u) * np.sin(v) * r
z = np.cos(v) * r
x = x + x0
y = y + y0
z = z + z0

# 3D график для сферы
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')

# визуализация оригинальных точек
ax.scatter(correctX, correctY, correctZ, zdir='z', s=20, c='b', rasterized=True, label='Data points')

# "дополнение сферы"
ax.plot_wireframe(x, y, z, color="r", alpha=0.5, label='Fitted sphere')

# Отрисовка центра
ax.scatter([x0], [y0], [z0], s=100, c='g', marker='*', label=f'Center: ({x0[0]:.1f}, {y0[0]:.1f}, {z0[0]:.1f})')

# Легенда графика
ax.legend()
ax.set_aspect('equal')

ax.set_xlabel('$x$ (mm)', fontsize=16)
ax.set_ylabel('\n$y$ (mm)', fontsize=16)
zlabel = ax.set_zlabel('\n$z$ (mm)', fontsize=16)

plt.title('Sphere Fitting with Center Point', fontsize=16)
plt.tight_layout()
plt.show()
