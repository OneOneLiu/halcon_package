import halcon as ha
import pyvista as pv
import numpy as np
import transforms3d as tfs

def model_transform(model_mesh, transformation):
    x, y, z, rx, ry, rz = transformation
    
    model_mesh = model_mesh.rotate_z(rz,inplace=False)
    model_mesh = model_mesh.rotate_y(ry, inplace=False)
    model_mesh = model_mesh.rotate_x(rx, inplace=False)
    model_mesh = model_mesh.translate([1000*x,1000*y,1000*z])
    
    return model_mesh

if __name__ == '__main__':
    
    # Specify the full path for your STL and PLY files
    model_file_path = '/catkin_ws/src/halcon_package/pcd/bowl.stl'
    scene_file_path = '/catkin_ws/src/halcon_package/pcd/scence.ply'
    
    # Read object model
    Object_3d, status = ha.read_object_model_3d(model_file_path, "mm",[],[])
    # Create object surface template
    Object_surface = ha.create_surface_model(Object_3d, 0.05, 'train_view_based', 'true') # This line would require access to display, and cannot be run on a remote server
    # Object_surface = ha.create_surface_model(Object_3d, 0.05, [], [])

    # Read scene model
    Scene_3d, status = ha.read_object_model_3d(scene_file_path, "mm",[],[])
    print(f"Initial Scene Points: {ha.get_object_model_3d_params(Scene_3d, 'num_points')}")

    # Preprocess the scene
    ObjectModel3DThresholdedY = ha.select_points_object_model_3d(Scene_3d, 'point_coord_y', -10, 10)
    print(f"After Y Thresholding: {ha.get_object_model_3d_params(ObjectModel3DThresholdedY, 'num_points')}")
    ObjectModel3DThresholdedX = ha.select_points_object_model_3d(ObjectModel3DThresholdedY, 'point_coord_x', -10, 10)
    print(f"After X Thresholding: {ha.get_object_model_3d_params(ObjectModel3DThresholdedX, 'num_points')}")
    ObjectModel3DThresholdedZ = ha.select_points_object_model_3d(ObjectModel3DThresholdedX, 'point_coord_z', 0.7, 1.098)
    print(f"After Z Thresholding: {ha.get_object_model_3d_params(ObjectModel3DThresholdedZ, 'num_points')}")
    ObjectModel3DConnected = ha.connection_object_model_3d(ObjectModel3DThresholdedZ, 'distance_3d', 0.0035)
    ObjectModel3DSelected = ha.select_object_model_3d(ObjectModel3DConnected, 'num_points', 'and', 2000, 100000)
    UnionObjectModel3D = ha.union_object_model_3d(ObjectModel3DSelected, 'points_surface')
    TargetPC, Information = ha.triangulate_object_model_3d(UnionObjectModel3D, 'greedy', [], [])

    # Matching without view based
    pose, score, ID = ha.find_surface_model(Object_surface, TargetPC, 0.05, 0.2, 0.02, "true", ['num_matches', 'use_view_based'], [4, 'false'])
    print("score is:", score)

    # Print results   
    transformations = [pose[i:i+6] for i in range(0, len(pose), 7)]
    print(transformations)   
    transformations_scaled = [[x*1000 if i < 3 else x for i, x in enumerate(pose)] for pose in transformations]
    print(transformations_scaled[0])
    mat_rot = tfs.euler.euler2mat(*transformations_scaled[0][3:6],'sxyz')
    # Create a column vector from the last three elements of the first sublist of transformations_scaled
    translation_vector = np.array([[transformations_scaled[0][0]], 
                                [transformations_scaled[0][1]], 
                                [transformations_scaled[0][2]]])

    # Horizontally stack the rotation matrix with the translation vector
    mat = np.hstack((mat_rot, translation_vector))
    # Add a row [0, 0, 0, 1] to the end
    final_matrix = np.vstack((mat, np.array([0, 0, 0, 1])))
    print("matrix is", final_matrix)
    # Function to format the matrix into a string
    def format_matrix_eigenstyle(matrix):
        string_list = []
        for row in matrix:
            formatted_row = " ".join(f"{elem:.6g}" for elem in row)
            string_list.append(formatted_row)
        return "\n".join(string_list)

    # Format the matrix and write to file
    matrix_str = format_matrix_eigenstyle(final_matrix)
    with open('/catkin_ws/src/halcon_package/pcd/matrix.txt', 'w') as outfile:
        outfile.write(matrix_str)
     
    stl_meshes = []  
    # Load STL
    for i in range(1):
        stl_meshes.append(pv.read(model_file_path))

    # load scene
    ply_mesh = pv.read(scene_file_path)

    stl_meshes_trans=[model_transform(stl_meshes[i], transformations[i]) for i in range(1)]

    # Create a Plotter instance
    plotter = pv.Plotter()

    # Add  meshes to the plotter
    plotter.add_mesh(ply_mesh, color='lightblue', show_edges=True)
    for i in range(1):
        plotter.add_mesh(stl_meshes_trans[i], color='red', show_edges=True)

    axes = pv.Axes(show_actor=True, actor_scale=200.0, line_width=5)
    axes.origin = (0, 0, 0)
    plotter.add_actor(axes.actor)
    plotter.show()
