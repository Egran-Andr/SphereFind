import os
import numpy as np
import open3d as o3d


def process_pcd_files(input_folder, output_base_folder):
    # Create output base folder if it doesn't exist
    os.makedirs(output_base_folder, exist_ok=True)

    # Process each PCD file in input folder
    for filename in os.listdir(input_folder):
        if filename.endswith('.pcd'):
            # Create file-specific output folder
            file_basename = os.path.splitext(filename)[0]
            output_folder = os.path.join(output_base_folder, f"folder_{file_basename}")
            os.makedirs(output_folder, exist_ok=True)

            # Load point cloud
            input_path = os.path.join(input_folder, filename)
            pcd = o3d.io.read_point_cloud(input_path)
            points = np.asarray(pcd.points)

            # Save complete array
            array_path = os.path.join(output_folder, 'array.npy')
            np.save(array_path, points)
            print(f"Saved array to {array_path}")

            # Save individual coordinate files
            x_path = os.path.join(output_folder, 'x.npy')
            y_path = os.path.join(output_folder, 'y.npy')
            z_path = os.path.join(output_folder, 'z.npy')

            np.save(x_path, points[:, 0])  # X coordinates
            np.save(y_path, points[:, 1])  # Y coordinates
            np.save(z_path, points[:, 2])  # Z coordinates

            print(f"Saved coordinate files to {output_folder}")


# Example usage
input_folder = "C:\\Users\\Egran\\PycharmProjects\\SphereFind\\PcdToArray\\Input"
output_base_folder = "C:\\Users\\Egran\\PycharmProjects\\SphereFind\\PcdToArray\\Output"
process_pcd_files(input_folder, output_base_folder)