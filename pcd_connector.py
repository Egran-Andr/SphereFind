import os, sys, glob
import open3d as o3d

input_dir   = "pcd_files"
output_dir  = "pcd_out"
output_path = os.path.join(output_dir, "combined.pcd")

os.makedirs(output_dir, exist_ok=True)

# ---- 1. Load all *.pcd files -------------------------------------------------
pcd_files = sorted(glob.glob(os.path.join(input_dir, "*.pcd")))
if not pcd_files:
    sys.exit(f"No PCD files found in “{input_dir}”")

combined = o3d.geometry.PointCloud()

for f in pcd_files:
    cloud = o3d.io.read_point_cloud(f)
    cloud.remove_non_finite_points()
    if len(cloud.points) == 0:
        print(f"[warning] {os.path.basename(f)} had 0 points — skipped")
        continue
    combined += cloud
    print(f"Loaded {os.path.basename(f):<30}  ({len(cloud.points):,} pts)")

if len(combined.points) == 0:
    sys.exit("Every file was empty; nothing to visualise.")

o3d.io.write_point_cloud(output_path, combined, write_ascii=True)
print(f"\nCombined cloud has {len(combined.points):,} points → {output_path}")

# ---- Visualise ------------------------------------------------------------
coord = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0)

o3d.visualization.draw_geometries(
    [combined, coord],
    window_name="Combined PCDs",
    point_show_normal=False,
    width=1280, height=720
)
