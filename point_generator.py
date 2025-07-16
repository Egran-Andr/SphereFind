# -*- coding: utf-8 -*-
"""
Created by Florent Poux, (c) 2024 Licence MIT
Learn more at: learngeodata.eu
Modified with noise generation.
"""
import os

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
    shape_properties = []
    all_shapes_data = []

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
            max_dim = radius * 2
            print(radius)

        elif current_shape == 'partial_sphere':# срез сферы
            radius = np.random.uniform(100, 500) # радиус с 100 до 500 мм
            # Углы обзора (в радианах)
            theta = np.random.uniform(np.pi / 4, 3 * np.pi / 4, shape_points)
            phi = np.random.uniform(np.pi / 6, np.pi / 2, shape_points)
            x = radius * np.sin(phi) * np.cos(theta)
            y = radius * np.sin(phi) * np.sin(theta)
            z = radius * np.cos(phi)
            max_dim = radius * 2
            print(radius)

        elif current_shape == 'sphere': # сфера
            radius = np.random.uniform(100, 500)
            theta = np.random.uniform(0, 2 * np.pi, shape_points)
            phi = np.random.uniform(0, np.pi, shape_points)
            x = radius * np.sin(phi) * np.cos(theta)
            y = radius * np.sin(phi) * np.sin(theta)
            z = radius * np.cos(phi)
            max_dim = radius * 2
            print(radius)

        elif current_shape == 'cube': # куб
            size = np.random.uniform(50, 200)
            x = np.random.uniform(-size, size, shape_points)
            y = np.random.uniform(-size, size, shape_points)
            z = np.random.uniform(-size, size, shape_points)
            max_dim = size * 2

        else:  # прямой срез
            size = np.random.uniform(50, 200)
            x = np.random.uniform(-size, size, shape_points)
            y = np.random.uniform(-size, size, shape_points)
            z = np.zeros(shape_points)
            max_dim = size * 2

        # Поиск соприкосновений
        if not shape_properties:
            shape_center = np.random.uniform(-1000, 1000, 3)
        else:
            for attempt in range(100):
                shape_center = np.random.uniform(-1000, 1000, 3)
                valid = True
                for center, existing_max_dim in shape_properties:
                    if np.linalg.norm(shape_center - center) < (max_dim + existing_max_dim) / 2 * 1.2:
                        valid = False
                        break
                if valid:
                    break

        shape_properties.append((shape_center, max_dim))
        shape_pts = np.column_stack((x, y, z)) + shape_center
        points.append(shape_pts)

        shape_data = {
            'points': shape_pts,
            'type': current_shape,
            'center': shape_center,
            'radius': radius if 'sphere' in current_shape else None,
            'size': size if current_shape in ['cube', 'plane'] else None
        }
        all_shapes_data.append(shape_data)


    # Соединение точек и приведение к точности float64
    points_formatted = np.vstack(points).astype(np.float64)



    # Добавление шума
    if noise_type:
        points_formatted = points_formatted + np.random.normal(0, noise_intensity, points_formatted.shape).astype(
            np.float64)

    # Создание каталогов для хранения файлов
    os.makedirs("point_generator_output", exist_ok=True)
    os.makedirs(os.path.join("point_generator_output", "full_cloud"), exist_ok=True)

    # Сохранение полного облака точек (inside point_generator_output/full_cloud/)
    full_cloud_path = os.path.join("point_generator_output", "full_cloud")
    np.save(os.path.join(full_cloud_path, "points.npy"), points_formatted)
    x, y, z = points_formatted[:, 0], points_formatted[:, 1], points_formatted[:, 2]
    np.save(os.path.join(full_cloud_path, "x.npy"), x)
    np.save(os.path.join(full_cloud_path, "y.npy"), y)
    np.save(os.path.join(full_cloud_path, "z.npy"), z)

    # Сохранение координат отдельных фигур (inside point_generator_output/)
    for i, shape in enumerate(all_shapes_data):
        shape_points = shape['points']
        shape_prefix = f"shape_{i}_{shape['type']}"
        base_path = os.path.join("point_generator_output", shape_prefix)

        np.save(f"{base_path}_points.npy", shape_points)
        np.save(f"{base_path}_x.npy", shape_points[:, 0])
        np.save(f"{base_path}_y.npy", shape_points[:, 1])
        np.save(f"{base_path}_z.npy", shape_points[:, 2])

    return points_formatted


# Генерация и отрисовка
point_cloud = generate_random_point_cloud(
    num_points=2000,
    num_shapes=2,
    shape_type='half_sphere',
    noise_type="gaussian",
    noise_intensity=5.0
)

pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(point_cloud)
o3d.visualization.draw_geometries([pcd],
                                   window_name='Point Generator',
                                 width=1024, height=1024)