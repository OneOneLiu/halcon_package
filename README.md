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
│   ├── bowl.stl
│   ├── matrix.txt
│   └── scence.ply
└── src
    └── registration_test.py
```

> 其中 `matrix.txt` 为手眼变换矩阵，
>
> `bowl.stl`为配准的模型文件，
>
> `scence.ply`为获取的点云场景文件，
>
> `license_eval_halcon_progress_2024_05.dat`为halcon授权文件，需每月更新。

## 3. Test registration

Start ROS on the current terminal:

```bash
roscore
```

Create a new terminal and test the registration program:

```bash
rosrun halcon_package registration_test
```

