#!/usr/bin/env python3
import halcon as ha
import pyvista as pv
import numpy as np
import transforms3d as tfs
import rospy
from std_msgs.msg import Int32
import math

def model_transform(model_mesh, transformation):
    x, y, z, rx, ry, rz, _ = transformation
    
    model_mesh = model_mesh.rotate_z(rz,inplace=False)
    model_mesh = model_mesh.rotate_y(ry, inplace=False)
    model_mesh = model_mesh.rotate_x(rx, inplace=False)
    model_mesh = model_mesh.translate([x,y,z])
    
    return model_mesh

def radians_to_degrees(radians):
    degrees = radians * (180 / math.pi)
    return degrees

def degrees_to_radians(degrees):
    radians = degrees * (math.pi / 180)
    return radians

def msg_callback(msg):
    if msg.data == 3:
        rospy.loginfo("Start point cloud registration")
        main()
def main():
    # Define the model name
    model_names = ['E0u','E0s', 'E1', 'E1u','E3', 'E3u' , 'F1']
    score_threshold = [0.25, 0.6, 0.2, 0.45, 0.13, 0.45, 0.12]
    # Specify the full path for your STL and PLY files
    model_file_path = []
    for name in model_names:
        path = '/catkin_ws/src/grasp_icp/pcd/'+name+'.stl'
        model_file_path.append(path)
    print(model_file_path)
    scene_file_path = '/catkin_ws/src/grasp_icp/pcd/scene.ply'
    # Read scene model
    Scene_3d, status = ha.read_object_model_3d(scene_file_path, "mm",[],[])
    print(f"Initial Scene Points: {ha.get_object_model_3d_params(Scene_3d, 'num_points')}")

    # Preprocess the scene
    ObjectModel3DThresholdedY = ha.select_points_object_model_3d(Scene_3d, 'point_coord_y', -10, 10)
    print(f"After Y Thresholding: {ha.get_object_model_3d_params(ObjectModel3DThresholdedY, 'num_points')}")
    ObjectModel3DThresholdedX = ha.select_points_object_model_3d(ObjectModel3DThresholdedY, 'point_coord_x', -10, 10)
    print(f"After X Thresholding: {ha.get_object_model_3d_params(ObjectModel3DThresholdedX, 'num_points')}")
    ObjectModel3DThresholdedZ = ha.select_points_object_model_3d(ObjectModel3DThresholdedX, 'point_coord_z', -1, 0.685)
    print(f"After Z Thresholding: {ha.get_object_model_3d_params(ObjectModel3DThresholdedZ, 'num_points')}")
    ObjectModel3DConnected = ha.connection_object_model_3d(ObjectModel3DThresholdedZ, 'distance_3d', 0.0035)
    ObjectModel3DSelected = ha.select_object_model_3d(ObjectModel3DConnected, 'num_points', 'and', 2000, 100000)
    UnionObjectModel3D = ha.union_object_model_3d(ObjectModel3DSelected, 'points_surface')
    TargetPC, Information = ha.triangulate_object_model_3d(UnionObjectModel3D, 'greedy', [], [])
    
    # Read object model
    model_mesh = []
    for path in model_file_path:
        Object_3d, status = ha.read_object_model_3d(path, "mm",[],[])
        # Create object surface template
        Object_surface = ha.create_surface_model(Object_3d, 0.05, 'train_view_based', 'true') # This line would require access to display, and cannot be run on a remote server
        model_mesh.append(Object_surface)
    # Object_surface = ha.create_surface_model(Object_3d, 0.05, [], [])

    match_score = []
    match_pose = []
    for mesh, thre in zip(model_mesh, score_threshold):
        # Matching without view based
        pose, score, ID = ha.find_surface_model(mesh, TargetPC, 0.05, 0.2, thre, "true", ['num_matches', 'use_view_based'], [1, 'false'])
        print("score is:", score)
        # Only append score[0] if score is not empty and score[0] is a number
        if score and isinstance(score[0], (float, int)):
            match_score.append(score[0])
            match_pose.append(pose)
        else:
            print("No valid score found to append.")
            match_score.append(0)
            
    # Print results   
    transformations = [match_pose[i:i+6] for i in range(0, len(match_pose), 7)]
    print(transformations)   
    # transformations_scaled = [[x*1000 if i < 3 else x for i, x in enumerate(match_pose)] for match_pose in transformations]
    highest_score = [0.45, 0.73, 0.23, 0.69, 0.23, 0.63, 0.23]
    normalized_score = []
    for real_score, max_score in zip(match_score, highest_score):
        norm_score = real_score/max_score
        normalized_score.append(norm_score)
    print('The normalized scores are ', normalized_score)
    max_index1 = np.argmax(normalized_score)
    model_file_path_vis = model_file_path[max_index1]
    print('The matched item is: ', model_names[max_index1])
    filtered_list = [element for element in normalized_score if element != 0]
    print('The list removed all 0 is: ', filtered_list)
    scaled_trans = [[element * 1000 if index < 3 else element for index, element in enumerate(sublist)] for sublist in transformations[0]]
    print('The scaled pose is ', scaled_trans)
    max_index2 = np.argmax(filtered_list)
    print('The matched pose is ', scaled_trans[max_index2])
    rot_deg = scaled_trans[max_index2][3:6]
    print("the rotation angles in degree are", rot_deg)

    # Convert degree into radians
    rot_rad = []
    for i in rot_deg:
        d = degrees_to_radians(i)
        rot_rad.append(d)
    print("the rotation angles in radians are", rot_rad)

    mat_rot = tfs.euler.euler2mat(*rot_rad[0:3],'rxyz')
    # Create a column vector from the last three elements of the first sublist of transformations_scaled
    translation_vector = np.array([[scaled_trans[max_index2][0]], 
                                [scaled_trans[max_index2][1]], 
                                [scaled_trans[max_index2][2]]])

    # Horizontally stack the rotation matrix with the translation vector
    mat = np.hstack((mat_rot, translation_vector))
    # Add a row [0, 0, 0, 1] to the end
    final_matrix = np.vstack((mat, np.array([0, 0, 0, 1])))
    print("The final pose matrix is", final_matrix)
    # Function to format the matrix into a string
    def format_matrix_eigenstyle(matrix):
        string_list = []
        for row in matrix:
            formatted_row = " ".join(f"{elem:.6g}" for elem in row)
            string_list.append(formatted_row)
        return "\n".join(string_list)

    # Format the matrix and write to file
    matrix_str = format_matrix_eigenstyle(final_matrix)
    with open('/catkin_ws/src/grasp_icp/out/matrix.txt', 'w') as outfile:
        outfile.write(matrix_str)
     
    stl_meshes = []  
    # Load STL
    coord_path = "/home/jensen/catkin_ws/src/grasp_icp/pcd/coor.stl"
    for i in range(1):
        stl_meshes.append(pv.read(model_file_path_vis))

    # load scene
    ply_mesh = pv.read(scene_file_path)

    stl_meshes_trans=[model_transform(stl_meshes[i], scaled_trans[max_index2]) for i in range(1)]

    # Create a Plotter instance
    plotter = pv.Plotter()

    # load gripper coor
    # Load STL1
    coord_path = "/catkin_ws/src/grasp_icp/pcd/coord.STL"
    gripper_mesh1 = pv.read(coord_path)
    rx1 = radians_to_degrees(1.54943807e+00)
    ry1 = radians_to_degrees(-1.20695783e+00)
    rz1 = radians_to_degrees(-2.02373778e-02)
    gripper_trans = model_transform(gripper_mesh1, [-5.26967291e+02,8.64301435e+01,5.57162655e+00,rx1, ry1, rz1, 0])

    # Load STL2
    gripper_mesh2 = pv.read(coord_path)
    rx2 = radians_to_degrees(-1.74224416e+00)
    ry2 = radians_to_degrees(-1.01732943e+00)
    rz2 = radians_to_degrees(2.09755299e+00)
    gripper_trans2 = model_transform(gripper_mesh2, [-6.77981669e+02,-2.92469000e+00,4.16905000e+01,rx2, ry2, rz2, 0])

    # Load STL3
    # gripper_trans3 = model_transform(gripper_mesh, [-5.95230000e+02,-2.92469000e-01,4.16905000e+01,1.39934907e+00, 1.01732960e+00, 1.04404015e+00, 0])

    # Add  meshes to the plotter
    plotter.add_mesh(ply_mesh, color='lightblue', show_edges=True)
    for i in range(1):
        plotter.add_mesh(stl_meshes_trans[i], color='red', show_edges=True)
    plotter.add_mesh(gripper_trans, color='lightblue', show_edges=True)
    # plotter.add_mesh(gripper_trans2, color='yellow', show_edges=True)
    axes = pv.Axes(show_actor=True, actor_scale=200.0, line_width=5)
    axes.origin = (0, 0, 0)
    plotter.add_actor(axes.actor)
    plotter.show()

if __name__ == '__main__':
    main()