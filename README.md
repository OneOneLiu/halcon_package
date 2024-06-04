# halcon_package

## 1. Download software source file

Download software source file and save it to the `downloads/` folder in this package (untracked). The folder should be like this after that:

```bash{.line-numbers}
downloads/
└── halcon-23.05.0.0-x64-linux
    ├── readme.txt
    ├── repository
    │   └── packages.mvtec.com
    │       ├── ai_acc
    │       ├── halcon
    │       │   └── halcon-23.05-progress
0-deep-learning-core-x64-win64_x64-linux_aarch64-linux_armv7a-linux.zip
    │       │
    │       ├── interfaces
    │       ├── som.catalog
    │       └── som.catalog.asc
    ├── som
    ├── som-register.sh
    └── som.png
```

## 2. Set the environment and Partial files description

### 2.1 Set the environment

First, run the Docker environment:

```bash
cd catkin_ws/src/halcon_package/Docker
bash build.bash
bash noetic.bash
```

Install halcon：

```bash
cd catkin_ws/src/halcon_package/
bash install.bash
```

### 2.2 Partial files description

The directory structure of this package is as follows:

```
├── CMakeLists.txt
├── Docker
│   ├── Dockerfile
│   ├── build.bash
│   └── noetic.bash
├── README.md
├── downloads
│   ├── halcon-23.05.0.0-x64-linux/...
│   └── license_eval_halcon_progress_2024_05.dat
├── install.bash
├── package.xml
├── pcd
│   ├── xxx.stl
│   └── scence_gazebo.pcd
└── src
    ├── halcon_registration.py
    ├── registration_server.py
    └── registration_cilent.py
```

> `xxx.stl`为配准的模型文件，
>
> `scence_gazebo.pcd`为获取的点云场景文件，
>
> `license_eval_halcon_progress_YYYY_MM.dat`为halcon授权文件，需每月更新。

## 3. Test registration

+ 启动测试环境
```bash
cd halcon_package/Docker
bash noetic.bash 
```
+ 编译功能包
```bash
cd catkin_ws/
catkin_make
source devel/setup.bash 
cd catkin_ws/src/halcon_package
bash install.bash
```
+ 启动配准服务器
打开launch文件为客户端节点解除注释
打开halcon_registration.py修改PCD路径,当前为'/catkin_ws/src/grasp_icp/pcd/'
```bash
roslaunch pose_transformation pose_transform.launch
```
