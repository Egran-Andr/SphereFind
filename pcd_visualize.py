import open3d as o3d
import numpy as np
import os, sys

def show_pcd(path, point_size=5):
    if not os.path.isfile(path):
        sys.exit(f" File not found: {path}")

    cloud = o3d.io.read_point_cloud(path)
    if not cloud.has_points():
        sys.exit(f"No points found in: {path}")

    print(f"✅Loaded: {path}")
    print(f"   • Point count: {len(cloud.points):,}")
    print(f"   • Bounds: min {cloud.get_min_bound()}, max {cloud.get_max_bound()}")
    print(f"   • Center: {cloud.get_center()}")

    cloud.remove_non_finite_points()
    # Optional fix: rescale if coordinates are too large or tiny
    bounds = np.linalg.norm(cloud.get_max_bound() - cloud.get_min_bound())
    if bounds > 1e5 or bounds < 1e-3:
        print("⚠️  Cloud bounds are extreme. Normalizing...")
        cloud.translate(-cloud.get_center())
        cloud.scale(1.0 / bounds, center=(0, 0, 0))

    coord = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0)

    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name=os.path.basename(path), width=1280, height=720)
    vis.add_geometry(cloud)
    vis.add_geometry(coord)

    render_opt = vis.get_render_option()
    render_opt.point_size = point_size
    render_opt.background_color = np.array([0, 0, 0])  # Black

    vis.poll_events()
    vis.update_renderer()

    # Reset the camera to center on the cloud
    ctr = vis.get_view_control()
    ctr.set_lookat(cloud.get_center())
    ctr.set_front([0.0, 0.0, -1.0])
    ctr.set_up([0.0, 1.0, 0.0])
    ctr.set_zoom(0.6)

    vis.run()
    vis.destroy_window()

# ────── EXAMPLE ──────
file_path = "pcd_files/cloud-recon1.pcd"
show_pcd(file_path)
