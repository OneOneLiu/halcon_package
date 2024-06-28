import open3d as o3d
import numpy as np
import transforms3d as tfs
import matplotlib.pyplot as plt
from collections import Counter
import copy
import time

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

def find_surface_model(object_point_cloud, clusters, voxel_size, threshold, debug_mode):
    ##############################
    ### 点云配准：RANSAC + ICP ###
    ##############################
    match_confidence = []
    transformed_objects = []
    transform_matrix = []
    object_down, object_fpfh = preprocess_point_cloud(object_point_cloud, voxel_size)
    for i, cluster_pcd in enumerate(clusters):
        if debug_mode == True:
            print(f"Registering cluster {i+1}...")
        # 预处理聚类点云
        cluster_down, cluster_fpfh = preprocess_point_cloud(cluster_pcd, voxel_size)
        start_time = time.time()
        # RANSAC初始配准
        distance_threshold = voxel_size * 1.0 # 可调参数，用于RANSAC
        result_ransac = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
            object_down, cluster_down, object_fpfh, cluster_fpfh,
            mutual_filter=False,
            max_correspondence_distance=distance_threshold,
            estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
            ransac_n=4,
            checkers=[o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.90), # 可调参数
                    o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(distance_threshold)],
            criteria=o3d.pipelines.registration.RANSACConvergenceCriteria(4000000, 500)) # 可调参数
        if debug_mode == True:
            print("RANSAC alignment")
            print(result_ransac)
        
        # ICP精细配准
        icp = o3d.pipelines.registration.registration_icp(
            object_down,
            cluster_down,
            max_correspondence_distance=distance_threshold,
            init=result_ransac.transformation,
            estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            criteria=o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=4000) # 可调参数
        )
        if debug_mode == True:
            print(f"ICP transformation: \n{icp.transformation}")
        transform_matrix.append(icp.transformation)
        end_time = time.time()
        execution_time = end_time - start_time
        if debug_mode == True:
            print(f"配准执行时间: {execution_time} 秒")

        confidence = compute_confidence(icp)
        match_confidence.append(confidence)

        if debug_mode == True:
            print(f"Confidence for cluster {i+1}: {confidence}")
            
        # 应用变换到object_point_cloud
        transformed_object = copy.deepcopy(object_point_cloud) # 深拷贝防止原始点云被修改
        transformed_object.transform(icp.transformation)
        transformed_objects.append(transformed_object)
        
        # 可视化注册结果
        # 在整体点云上显示配准结果
        # o3d.visualization.draw_geometries([scene_point_cloud, transformed_object, axes], window_name=f"聚类 {i+1} 配准结果", width=800, height=600)

        # 在局部点云上显示配准结果
        # o3d.visualization.draw_geometries([cluster_pcd, transformed_object, axes], window_name=f"聚类 {i+1} 配准结果", width=800, height=600)

    if max(match_confidence) > threshold:
        matched_index = np.argmax(match_confidence)
        return max(match_confidence),transform_matrix[matched_index]
    else:
        return 0, None

def Registrate(debug_mode = False):
    # 创建坐标系物体，用于可视化
    axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=200, origin=[0, 0, 0])
    
    ###############################
    ### 读取并处理目标模型和场景模型 ###
    ###############################
    

    PCD_PATH = '/catkin_ws/src/grasp_icp/pcd/'

    # # 读取文件路径
    # model_file_path = '/catkin_ws/src/grasp_icp/pcd/t_pipe_u.stl'
    scene_file_path = '/catkin_ws/src/grasp_icp/pcd/scence_gazebo.ply'
    
    # 读取并处理目标模型
    # object_mesh = o3d.io.read_triangle_mesh(model_file_path)
    # object_mesh.compute_vertex_normals()
    # object_point_cloud = object_mesh.sample_points_poisson_disk(number_of_points=2048)

    model_names = ['t_pipe_u','L_pipe_u']
    confidence_threshold = [0.27,0.255]
    model_file_path = []
    for name in model_names:
        path = PCD_PATH + name +'.stl'
        model_file_path.append(path)

    # 批量读取并处理目标模型
    objects_point_cloud = []
    for path in model_file_path:
        object_mesh = o3d.io.read_triangle_mesh(path)
        object_mesh.compute_vertex_normals()
        object_point_cloud = object_mesh.sample_points_poisson_disk(number_of_points=2048)
        objects_point_cloud.append(object_point_cloud)

    # 读取场景模型
    scene_point_cloud = o3d.io.read_point_cloud(scene_file_path)
    
    # 可视化原始点云
    if debug_mode == True:
        o3d.visualization.draw_geometries([scene_point_cloud, axes], window_name="原始点云", width=800, height=600)

    ##############################
    ### 预处理：裁剪场景点云并聚类 ###
    ###############################
    
    # 预处理：裁剪场景点云 1.创建一个包围盒 2.使用包围盒裁剪点云 3.可视化裁减后的点云
    bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound=(-6e+02, -6e+02, 6e+02), max_bound=(6e+02, 6e+02, 1.01e+03))
    cropped_pcd = scene_point_cloud.crop(bbox)

    if debug_mode == True:
        o3d.visualization.draw_geometries([cropped_pcd, axes], window_name="裁减后的点云", width=800, height=600)
        # 预处理：聚类点云
        with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
            labels = np.array(cropped_pcd.cluster_dbscan(eps=20, min_points=10, print_progress=True)) 

    # 预处理：聚类点云
    with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Error) as cm:
        labels = np.array(cropped_pcd.cluster_dbscan(eps=20, min_points=10, print_progress=True)) 

    # 每个点的类别标签
    max_label = labels.max()
    
    # 统计每个类中点的数量
    label_counts = Counter(labels)

    if debug_mode == True:
        print(f"point cloud has {max_label + 1} clusters")
        print(label_counts)

    # 用不同颜色显示不同的聚类结果
    colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
    colors[labels < 0] = 0
    cropped_pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])
    # 可视化聚类结果
    if debug_mode == True:
        o3d.visualization.draw_geometries([cropped_pcd, axes], window_name="聚类结果", width=800, height=600)

    # 剔除点数少于阈值的类
    min_points_threshold = 200
    filtered_clusters = [label for label, count in label_counts.items() if count >= min_points_threshold]

    if debug_mode == True:
        print(f"Remained clusters: {filtered_clusters}")
    clusters = []
    for cluster in filtered_clusters:
        print(f"Cluster {cluster} has {label_counts[cluster]} points")
        cluster_indices = np.where(labels == cluster)[0]
        cluster = cropped_pcd.select_by_index(cluster_indices)
        clusters.append(cluster)
    
    # # 显示剔除少于阈值后的聚类结果
    if debug_mode == True:
        o3d.visualization.draw_geometries(clusters + [axes], window_name="剔除少于阈值后的聚类结果", width=800, height=600)
    
    # name_pointcloud_dict = dict(zip(model_names,objects_point_cloud))
    voxel_size = 5 # 可调参数，用于下采样

    match_matrix = None
    max_iter = 3
    while(match_matrix is None and max_iter!= 0):
        match_score = []
        match_transform_matrix = []
        for object_point_cloud,threshold in zip(objects_point_cloud, confidence_threshold):
            score, matrix = find_surface_model(object_point_cloud, clusters, voxel_size, threshold, debug_mode)
            match_score.append(score)
            match_transform_matrix.append(matrix)

        highest_score = [0.26, 0.25]
        normalized_score = []
        for real_score, max_score in zip(match_score, highest_score):
            norm_score = real_score/max_score
            normalized_score.append(norm_score)
        # print('The normalized scores are ', normalized_score)
        max_index1 = np.argmax(normalized_score)

        matched_model = model_names[max_index1]
        match_matrix = match_transform_matrix[max_index1]
        print('The matched item is: ', matched_model)
        print('The matched item transform matrix is: ', match_matrix)
        max_iter = max_iter - 1

    # 最终可视化最终配准结果
    transformed_object = copy.deepcopy(objects_point_cloud[max_index1]) # 深拷贝防止原始点云被修改
    transformed_object.transform(match_matrix)
    if debug_mode == True:
        o3d.visualization.draw_geometries([scene_point_cloud, transformed_object, axes], window_name="最终配准结果", width=800, height=600)

    return match_matrix,matched_model

if __name__ == '__main__':
    match_matrix,matched_model = Registrate(debug_mode = False)
    print(type(match_matrix),type(matched_model))
