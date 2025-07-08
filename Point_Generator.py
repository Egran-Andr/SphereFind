# -*- coding: utf-8 -*-
"""
Created by Florent Poux, (c) 2024 Licence MIT
Learn more at: learngeodata.eu
Modified with noise generation.
"""

import numpy as np
import open3d as o3d


def add_noise(points, noise_type="gaussian", intensity=0.01):
    """
    Добавляет шум к точкам.
    Параметры:
        noise_type: "gaussian" (гауссовский) или "uniform" (равномерный)
        intensity:
            - для "gaussian" это стандартное отклонение (std_dev),
            - для "uniform" это диапазон [-intensity, +intensity].
    """
    if noise_type == "gaussian":
        noise = np.random.normal(0, intensity, points.shape)
    elif noise_type == "uniform":
        noise = np.random.uniform(-intensity, intensity, points.shape)
    else:
        raise ValueError("Неизвестный тип шума. Используйте 'gaussian' или 'uniform'.")
    return points + noise


def generate_random_point_cloud(num_points=5000, num_shapes=1, shape_type=None,
                                noise_type=None, noise_intensity=0.01):
    points = []

    for _ in range(num_shapes):
        current_shape = shape_type if shape_type else np.random.choice(
            ['sphere', 'cube', 'plane', 'half_sphere', 'partial_sphere'])
        shape_points = num_points // num_shapes

        if current_shape == 'half_sphere': # полусфера
            radius = np.random.uniform(100, 500) # радиус с 100 до 500 мм
            theta = np.random.uniform(0, 2 * np.pi, shape_points)
            phi = np.random.uniform(0, np.pi / np.random.uniform(2, 4) , shape_points) #np.random.uniform(2, 4) - 2,4 это какой частью от оригинальной сферы будет полусфера (1/2 до 1/4 соответственно)
            x = radius * np.sin(phi) * np.cos(theta)
            y = radius * np.sin(phi) * np.sin(theta)
            z = -radius * np.cos(phi)
            print(radius)

        elif current_shape == 'partial_sphere': # срез сферы
            radius = np.random.uniform(100, 500) # радиус с 100 до 500 мм

            # Углы обзора (в радианах)
            azimuth_range = (np.pi / 4, 3 * np.pi / 4)  # азимутальный угол (горизонтальный обзор)
            polar_range = (np.pi / 6, np.pi / 2)  # полярный угол (вертикальный обзор)

            theta = np.random.uniform(*azimuth_range, shape_points)
            phi = np.random.uniform(*polar_range, shape_points)

            x = radius * np.sin(phi) * np.cos(theta)
            y = radius * np.sin(phi) * np.sin(theta)
            z = radius * np.cos(phi)
            print(radius)

        elif current_shape == 'sphere': # сфера
            radius = np.random.uniform(100, 500)
            theta = np.random.uniform(0, 2 * np.pi, shape_points)
            phi = np.random.uniform(0, np.pi, shape_points)
            x = radius * np.sin(phi) * np.cos(theta)
            y = radius * np.sin(phi) * np.sin(theta)
            z = radius * np.cos(phi)
            print(radius)

        elif current_shape == 'cube': # куб
            x = np.random.uniform(-1, 1, shape_points)
            y = np.random.uniform(-1, 1, shape_points)
            z = np.random.uniform(-1, 1, shape_points)

        else:  # прямой срез
            x = np.random.uniform(-1, 1, shape_points)
            y = np.random.uniform(-1, 1, shape_points)
            z = np.zeros(shape_points)

        shape_center = np.random.uniform(-5, 5, 3)
        points.extend(np.column_stack((x, y, z)) + shape_center)

    points_formatted = np.array(points)

    # Добавление шума
    if noise_type:
        points_formatted = add_noise(points_formatted, noise_type, noise_intensity)

    # Сохранение в файлы формата npy для последующей обработки
    np.save("points.npy", points_formatted)
    x, y, z = points_formatted[:, 0], points_formatted[:, 1], points_formatted[:, 2]
    np.save("x.npy", x)
    np.save("y.npy", y)
    np.save("z.npy", z)

    #После работы скрипта необходимо перетащить файлы в папку Least_Squares_Method
    return points_formatted


# Пример использования
point_cloud = generate_random_point_cloud(
    num_points=500,
    num_shapes=1,
    shape_type='partial_sphere',
    noise_type="gaussian",  # или "uniform"
    noise_intensity=3.0  # величина шума (std_dev для гауссовского, диапазон для равномерного)
)

# Визуализация
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(point_cloud)
o3d.visualization.draw_geometries([pcd])


