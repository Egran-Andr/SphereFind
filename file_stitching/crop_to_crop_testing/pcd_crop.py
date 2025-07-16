import os
import open3d as o3d
import numpy as np


def split_cloud_topdown(pcd, rows=2, cols=2, overlap=0.2, zero_coords=False):
    """
    Split point cloud into grid fragments (top-down view) with optional zeroing

    Args:
        pcd: Input point cloud
        rows: Number of vertical splits
        cols: Number of horizontal splits
        overlap: Overlap ratio between fragments (0-1)
        zero_coords: If True, transforms each fragment's coordinates to start at (0,0,0)
    Returns:
        List of fragments (in global or local coordinates)
        List of original bounding boxes (if zero_coords=True)
    """
    # Get 2D bounds (ignore Z-axis for top-down view)
    points = np.asarray(pcd.points)
    min_bound = np.min(points[:, :2], axis=0)  # Only X,Y
    max_bound = np.max(points[:, :2], axis=0)

    # Calculate block size with overlap
    total_size = max_bound - min_bound
    block_size = total_size / np.array([cols, rows])
    overlap_size = block_size * overlap

    fragments = []
    bounding_boxes = [] if zero_coords else None

    for i in range(rows):
        for j in range(cols):
            # Calculate 2D boundaries
            lower_2d = min_bound + np.array([j, i]) * (block_size - overlap_size)
            upper_2d = lower_2d + block_size + overlap_size

            # Convert to 3D bounding box (include all Z-values)
            lower_3d = np.append(lower_2d, -np.inf)
            upper_3d = np.append(upper_2d, np.inf)

            roi = o3d.geometry.AxisAlignedBoundingBox(lower_3d, upper_3d)
            fragment = pcd.crop(roi)

            if not fragment.is_empty():
                if zero_coords:
                    # Store original bounds before transformation
                    bounding_boxes.append((lower_3d, upper_3d))

                    # Convert to local coordinates
                    frag_points = np.asarray(fragment.points)
                    frag_min = np.min(frag_points, axis=0)
                    local_points = frag_points - frag_min

                    # Create new point cloud with local coordinates
                    local_fragment = o3d.geometry.PointCloud()
                    local_fragment.points = o3d.utility.Vector3dVector(local_points)

                    # Copy attributes
                    if pcd.has_colors():
                        local_fragment.colors = fragment.colors
                    if pcd.has_normals():
                        local_fragment.normals = fragment.normals

                    fragments.append(local_fragment)
                else:
                    fragments.append(fragment)

    return (fragments, bounding_boxes) if zero_coords else fragments


# Load and preprocess point cloud
input_path = "C:/Users/Egran/PycharmProjects/SphereFind/file_stitching/crop_to_crop_testing/input/cloud-0.pcd"
pcd = o3d.io.read_point_cloud(input_path)
pcd.remove_non_finite_points()


fragments, bboxes = split_cloud_topdown(pcd, rows=3, cols=2, overlap=0.07, zero_coords=True)


output_dir = "output"
os.makedirs(output_dir, exist_ok=True)

for i, fragment in enumerate(fragments):
    output_path = os.path.join(output_dir, f"fragment_{i}.pcd")
    o3d.io.write_point_cloud(output_path, fragment)
    print(f"Saved: {output_path} with {len(fragment.points)} points")

# Optional: Visualize
o3d.visualization.draw_geometries(fragments, window_name="Fragments")
