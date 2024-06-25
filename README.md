# Point Cloud Registration Using Open3D

Run the Registration Script

Execute the registration_test.py script:
```bash{.line-numbers}
python3 src/registration_test.py
```

This script will:

- Read and preprocess the object and scene point clouds.
- Crop and cluster the scene point cloud.
- Register each cluster with the object point cloud using RANSAC and ICP.
- Visualize the registration results.

> Notes
> The parameters such as voxel size, distance threshold, and other parameters used in RANSAC and ICP can be tuned in the script to achieve better results.
> The script visualizes intermediate and final results using Open3D's visualization tools.
