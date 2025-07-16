import open3d as o3d
import numpy as np
from tqdm import tqdm


class NormalAwareRegistrator:
    def __init__(self, voxel_size=5, normal_radius=None, max_points=100000):
        self.voxel_size = voxel_size
        self.normal_radius = normal_radius if normal_radius else voxel_size * 5
        self.max_points = max_points

    def _ensure_normals(self, pcd):
        """Гарантированное вычисление нормалей с проверкой качества"""
        if not pcd.has_normals():
            print("Вычисление нормалей...")
            pcd.estimate_normals(
                search_param=o3d.geometry.KDTreeSearchParamHybrid(
                    radius=self.normal_radius,
                    max_nn=50))

        # Проверка валидности нормалей
        norms = np.asarray(pcd.normals)
        if np.any(np.isnan(norms)) or np.any(np.linalg.norm(norms, axis=1) < 0.9):
            print("Перерасчет нормалей из-за низкого качества...")
            pcd.orient_normals_consistent_tangent_plane(10)

        return pcd

    def _robust_preprocess(self, pcd):
        """Предобработка с гарантией наличия нормалей"""
        try:
            pcd.remove_non_finite_points()
            print(len(pcd.points))
            if len(pcd.points) == 0:
                raise ValueError("Пустое облако после фильтрации")

            # Адаптивный даунсемплинг
            current_size = len(pcd.points)
            if current_size > self.max_points:
                # Сохраняем долю оригинальных точек
                ratio = self.max_points / current_size
                pcd = pcd.random_down_sample(ratio)

                # Дополнительное сглаживание после случайного даунсемплинга

                pcd = pcd.voxel_down_sample(self.voxel_size)

                # Убедимся, что не переборщили с уменьшением
                if len(pcd.points) < 100:  # Минимальное разумное количество
                    pcd = pcd.voxel_down_sample(self.voxel_size * 2)

            else:
                # Стандартный воксельный даунсемплинг
                pcd = pcd.voxel_down_sample(self.voxel_size)

            pcd = self._ensure_normals(pcd)
            print(len(pcd.points))
            return pcd
        except Exception as e:
            print(f"Ошибка предобработки: {e}")
            return None

    def register(self, cloud_paths):
        """Надежная регистрация с контролем нормалей"""
        processed = []

        # Загрузка и предобработка с прогресс-баром
        for path in tqdm(cloud_paths, desc=f"Обработка облаков"):
            print(str(path))
            pcd = o3d.io.read_point_cloud(str(path))
            pcd = self._robust_preprocess(pcd)
            if pcd and len(pcd.points) > 10:  # Минимум 10 точек
                processed.append(pcd)

        if len(processed) < 2:
            raise ValueError("Недостаточно валидных облаков")

        # Инициализация результата
        result = processed[0]

        # Последовательная регистрация
        for i in tqdm(range(0, len(processed)), desc="Регистрация"):
            source = processed[i]

            # Двойная проверка нормалей
            source = self._ensure_normals(source)
            result = self._ensure_normals(result)

            # Вычисление FPFH-фич
            source_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
                source,
                o3d.geometry.KDTreeSearchParamHybrid(
                    radius=self.normal_radius,
                    max_nn=50))

            target_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
                result,
                o3d.geometry.KDTreeSearchParamHybrid(
                    radius=self.normal_radius,
                    max_nn=50))

            # Глобальная регистрация
            try:
                coarse = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
                    source, result,
                    source_fpfh, target_fpfh,
                    mutual_filter=True,
                    max_correspondence_distance=self.voxel_size * 3,
                    ransac_n=4)

                # Локальная оптимизация
                fine = o3d.pipelines.registration.registration_icp(
                    source, result,
                    max_correspondence_distance=self.voxel_size * 2,
                    init=coarse.transformation,
                    estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPlane())

                if fine.fitness > 0.3:  # Порог успешности
                    source.transform(fine.transformation)
                    result += source

                    # Периодическая оптимизация
                    if i % 2 == 0:
                        result = result.voxel_down_sample(self.voxel_size)
            except Exception as e:
                print(f"Пропуск облака {i}: {e}")

        return result


# Пример использования
if __name__ == "__main__":
    import glob

    # Настройки для сложных данных
    registrar = NormalAwareRegistrator(
        voxel_size=0.1,
        normal_radius=1,
        max_points=70000
    )

    # Автопоиск файлов
    scans = sorted(glob.glob("crop_to_crop_testing/output/*.pcd"))

    try:
        merged = registrar.register(scans)
        o3d.io.write_point_cloud("crop_to_crop_testing/output/merged_cloud.pcd", merged)
        print("Объединение успешно завершено!")
    except Exception as e:
        print(f"Критическая ошибка: {e}")