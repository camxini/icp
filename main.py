###############
#     ICP     #
#    J.Yang   #
# 5 Apr, 2025 #
###############

import os
import open3d as o3d
import numpy as np
from scipy.spatial import KDTree

def nearest_search(src, dst, max_distance=0.1):
    # src -> dst
    kdtree_dst = KDTree(dst)
    dist_src_dst, index_src_dst = kdtree_dst.query(src, k=1)
    valid_src = dist_src_dst < max_distance

    # dst -> src
    kdtree_src = KDTree(src)
    dist_dst_src, index_dst_src = kdtree_src.query(dst, k=1)
    valid_dst = dist_dst_src < max_distance

    # Bi-way match
    pairs = []
    for i in np.where(valid_src)[0]:
        j = index_src_dst[i]
        if valid_dst[j] and index_dst_src[j] == i:
            pairs.append((i, j))
    
    if len(pairs) == 0:
        return np.array([]), np.array([])
    
    pairs = np.array(pairs)
    return pairs[:, 0], pairs[:, 1]

def svd(src, dst):
    # Initialize
    T = np.eye(4)
    # Calculate average
    src_mean = np.mean(src, axis=0)
    dst_mean = np.mean(dst, axis=0)
    # Decentralize
    src_centered = src - src_mean
    dst_centered = dst - dst_mean
    # SVD
    W = np.dot(src_centered.T, dst_centered)
    U, _, Vt = np.linalg.svd(W)
    R = np.dot(Vt.T, U.T)
    # Adjust R if det(R) is negative
    if np.linalg.det(R) < 0:
        R[:, -1] = -R[:, -1]
    t = dst_mean - np.dot(R, src_mean)
    T[:3, :3] = R
    T[:3, 3] = t
    return R, t, T


def icp(src, dst, max_iter=200, threshold=1e-5, max_distance=0.1):
    # Transfer src & dst to points form
    src_points = np.copy(src)
    dst_points = np.copy(dst)
    # Initialize
    R_total = np.eye(3)
    t_total = np.zeros(3)
    T_total = np.eye(4)
    error_copy = 0

    for _ in range(max_iter):
        # Search the nearest points(by-way search)
        src_indices, dst_indices = nearest_search(src_points, dst_points, max_distance)
        if len(src_points) == 0:
            break
        current_src = src_points[src_indices]
        current_dst = dst_points[dst_indices]
        R, t, T = svd(current_src, current_dst)
        
        # Update src_points
        src_points = np.dot(src_points, R.T) + t

        # Update matrices
        R_total = np.dot(R, R_total)
        t_total = np.dot(R, t_total) + t
        T_total = np.dot(T, T_total)

        # If error < threshold, break the loop
        error = np.mean(np.linalg.norm(current_dst - current_src, axis=1))
        if abs(error_copy - error) < threshold:
            break
        error_copy = error
    return R_total, t_total, T_total

def error_calculate(src_points, dst_points, R, t):
    # Calculate total error
    src_points_processed = np.dot(src_points, R.T) + t
    error = np.linalg.norm(src_points_processed - dst_points, axis=1)
    error_total = np.sum(error)
    # Calculate traj length
    diff = np.diff(dst_points, axis=0)
    traj_length = np.linalg.norm(diff, axis=1)
    length = np.sum(traj_length)
    relative_error = error_total / length
    return relative_error

def main():
    src = o3d.io.read_point_cloud("./ply_files/0.ply")
    dst = o3d.io.read_point_cloud("./ply_files/1.ply")

    # True value lists from Matlab
    true_value_0to1 = [0.0497, 1.0896, 0]
    true_value_0to2 = [-0.0168, 1.3002, 0]
    true_value_0to3 = [-0.4372, 0.3587, 0]
    true_value_0to4 = [-0.2392, 1.3937, 0]
    true_value_0to5 = [-0.0603, 0.0399, 0]
    true_value_0to6 = [0.1882, 0.6112, 0]
    true_value_0to7 = [0.0808, 0.6474, 0]
    true_value_0to8 = [0.1077, 2.0572, 0]
    true_value_0to9 = [-0.0074, -1.0030, 0]

    src_points = np.array(src.points)
    dst_points = np.array(dst.points)

    # ICP
    R, t, T = icp(src_points, dst_points)
    src_points_shown_processed = np.dot(src_points, R.T) + t

    # Error calculation
    error = error_calculate(src_points, dst_points, R, t)
    print(f"\nRelative error: {error}\n")
    true_error = np.linalg.norm(true_value_0to1 - t)
    print(f"\nTrue error: {true_error}\n")

    # Matrices display
    print("\nRotation:\n", R)
    print("\nTranslation:\n", t)
    print("\nHomogeneous:\n", T)

    # Draw
    src_points_shown = o3d.geometry.PointCloud()
    src_points_shown.points = o3d.utility.Vector3dVector(src_points_shown_processed)
    dst_points_shown = o3d.geometry.PointCloud()
    dst_points_shown.points = o3d.utility.Vector3dVector(dst_points)
    
    # Color
    src_points_shown.paint_uniform_color([1, 0, 0])  # red for source
    dst_points_shown.paint_uniform_color([0, 0, 1])  # blue for destination

    o3d.visualization.draw_geometries([src_points_shown, dst_points_shown])


if __name__ == "__main__":
    main()