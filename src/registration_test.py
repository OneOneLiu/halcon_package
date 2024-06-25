import open3d as o3d
import numpy as np
import transforms3d as tfs
import matplotlib.pyplot as plt
from collections import Counter
import copy

def pick_points(pcd):
    print("")
    print(
        "1) Please pick at least three correspondences using [shift + left click]"
    )
    print("   Press [shift + right click] to undo point picking")
    print("2) After picking points, press 'Q' to close the window")
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window()
    vis.add_geometry(pcd)
    vis.run()  # user picks points
    vis.destroy_window()
    print("")
    return vis.get_picked_points()

def preprocess_point_cloud(pcd, voxel_size):
    pcd_down = pcd.voxel_down_sample(voxel_size)
    radius_normal = voxel_size * 2
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))
    radius_feature = voxel_size * 5
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    return pcd_down, pcd_fpfh

# 计算置信度函数
def compute_confidence(result):
    fitness = result.fitness
    inlier_rmse = result.inlier_rmse
    # 定义一个简单的置信度函数，可以根据需要进行调整
    confidence = fitness / (1 + inlier_rmse)
    return confidence

if __name__ == '__main__':
    # 创建坐标系物体，用于可视化
    axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=200, origin=[0, 0, 0])
    
    ###############################
    ### 读取并处理目标模型和场景模型 ###
    ###############################
    
    # 读取文件路径
    model_file_path = '/catkin_ws/src/halcon_package/pcd/bowl.stl'
    scene_file_path = '/catkin_ws/src/halcon_package/pcd/scence.ply'
    
    # 读取并处理目标模型
    object_mesh = o3d.io.read_triangle_mesh(model_file_path)
    object_mesh.compute_vertex_normals()
    object_point_cloud = object_mesh.sample_points_poisson_disk(number_of_points=2048)

    # 读取场景模型
    scene_point_cloud = o3d.io.read_point_cloud(scene_file_path)
    
    # 可视化原始点云
    o3d.visualization.draw_geometries([scene_point_cloud, axes], window_name="原始点云", width=800, height=600)

    ##############################
    ### 预处理：裁剪场景点云并聚类 ###
    ###############################
    
    # 预处理：裁剪场景点云
    # 创建一个包围盒
    bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound=(-8e+02, -5e+02, 1e+03), max_bound=(8e+02, 5e+02, 1.098e+03))
    # 使用包围盒裁剪点云
    cropped_pcd = scene_point_cloud.crop(bbox)
    
    # 可视化裁减后的点云
    o3d.visualization.draw_geometries([cropped_pcd, axes], window_name="裁减后的点云", width=800, height=600)
    
    # 预处理：聚类点云
    with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
        labels = np.array(cropped_pcd.cluster_dbscan(eps=20, min_points=10, print_progress=True)) 

    # 每个点的类别标签
    max_label = labels.max()
    print(f"point cloud has {max_label + 1} clusters")
    
    # 统计每个类中点的数量
    label_counts = Counter(labels)
    print(label_counts)

    # 用不同颜色显示不同的聚类结果
    colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
    colors[labels < 0] = 0
    cropped_pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])
    # 可视化聚类结果
    o3d.visualization.draw_geometries([cropped_pcd, axes], window_name="聚类结果", width=800, height=600)

    # 剔除点数少于阈值的类
    min_points_threshold = 300
    filtered_clusters = [label for label, count in label_counts.items() if count >= min_points_threshold]
    print(f"Remained clusters: {filtered_clusters}")
    clusters = []
    for cluster in filtered_clusters:
        print(f"Cluster {cluster} has {label_counts[cluster]} points")
        cluster_indices = np.where(labels == cluster)[0]
        cluster = cropped_pcd.select_by_index(cluster_indices)
        clusters.append(cluster)
    
    # # 显示剔除少于阈值后的聚类结果
    o3d.visualization.draw_geometries(clusters + [axes], window_name="剔除少于阈值后的聚类结果", width=800, height=600)
    
    ##############################
    ### 点云配准：RANSAC + ICP ###
    ##############################

    voxel_size = 9 # 可调参数，用于下采样
    transformed_objects = []
    object_down, object_fpfh = preprocess_point_cloud(object_point_cloud, voxel_size)
    
    for i, cluster_pcd in enumerate(clusters):
        print(f"Registering cluster {i+1}...")
        
        # 预处理聚类点云
        cluster_down, cluster_fpfh = preprocess_point_cloud(cluster_pcd, voxel_size)
        
        # RANSAC初始配准
        distance_threshold = voxel_size * 0.4 # 可调参数，用于RANSAC
        result_ransac = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
            object_down, cluster_down, object_fpfh, cluster_fpfh,
            mutual_filter=False,
            max_correspondence_distance=distance_threshold,
            estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
            ransac_n=4,
            checkers=[o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9), # 可调参数
                      o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(distance_threshold)],
            criteria=o3d.pipelines.registration.RANSACConvergenceCriteria(4000000, 500)) # 可调参数
        
        print("RANSAC alignment")
        print(result_ransac)
        
        # ICP精细配准
        icp = o3d.pipelines.registration.registration_icp(
            object_down,
            cluster_down,
            max_correspondence_distance=distance_threshold,
            init=result_ransac.transformation,
            estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            criteria=o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=2000) # 可调参数
        )
        print(f"ICP transformation: \n{icp.transformation}")
        
        confidence = compute_confidence(icp)
        print(f"Confidence for cluster {i+1}: {confidence}")
        # 应用变换到object_point_cloud
        transformed_object = copy.deepcopy(object_point_cloud) # 深拷贝防止原始点云被修改
        transformed_object.transform(icp.transformation)
        transformed_objects.append(transformed_object)
        
        # 可视化注册结果
        # 在整体点云上显示配准结果
        # o3d.visualization.draw_geometries([scene_point_cloud, transformed_object, axes], window_name=f"聚类 {i+1} 配准结果", width=800, height=600)
        # 在局部点云上显示配准结果
        o3d.visualization.draw_geometries([cluster_pcd, transformed_object, axes], window_name=f"聚类 {i+1} 配准结果", width=800, height=600)

    # 最终可视化所有配准结果
    o3d.visualization.draw_geometries([scene_point_cloud, *transformed_objects, axes], window_name="所有配准结果", width=800, height=600)