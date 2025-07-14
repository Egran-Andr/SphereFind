import open3d as o3d
import sys
import os
import numpy as np
import time

def crop_pcd(input_fpath, output_fpath, aabb):
    if not os.path.exists(input_fpath):
        print(f'File not exist by path {input_fpath}')
        return
    try:
        print('Try to load file...')
        pcd = o3d.io.read_point_cloud(input_fpath)
        pcd = pcd.remove_non_finite_points()
        print(f'Size of point cloud: {len(pcd.points)}')
        print(f'Original pcd aabb coords: {pcd.get_axis_aligned_bounding_box()}')

        cropped_pcd = pcd.crop(aabb)
        print(f'Size of cropped point cloud: {len(cropped_pcd.points)}')
        success = o3d.io.write_point_cloud(output_fpath, cropped_pcd, write_ascii=True)

        o3d.visualization.draw_geometries([cropped_pcd], 
                                  window_name="3D Point Cloud",
                                  width=1280,
                                  height=1080)
        if success:
            print(f'Point cloud saved by path: {output_fpath}')
        else:
            print('File not saved')
    except Exception as e:
        print(f'Error: {e}')

if __name__ == "__main__":

    input_file = 'pcd_files/cloud_1.ply'
    output_dir = 'pcd_out'

    base_name = os.path.splitext(os.path.basename(input_file))[0]
    output_filename = f"{base_name}_cropped_{str(time.time()).replace('.', '_')}.pcd"
    output_file = output_dir + '\\' + output_filename


    max_z = 129.593 # min_z + 80.0  # 10 см выше минимального
    min_z = max_z - 20 # 58.3124 + 17 # Ваше минимальное Z из данных
    
    min_angle_coords = np.array([-150, -50, min_z])  # Не обрезаем по X и Y
    max_angle_coords = np.array([0, 150, max_z]) 

    aabb = o3d.geometry.AxisAlignedBoundingBox(min_angle_coords, max_angle_coords)

    crop_pcd(input_file, output_file, aabb)