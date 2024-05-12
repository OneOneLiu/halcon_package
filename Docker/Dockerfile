FROM osrf/ros:noetic-desktop-full
ENV DEBIAN_FRONTEND=noninteractive

RUN sed -i 's#http://archive.ubuntu.com/#http://mirrors.tuna.tsinghua.edu.cn/#' /etc/apt/sources.list

# System layer
RUN apt-get update && \
    apt-get upgrade -y && \
    apt-get install -y wget && \
    apt-get install -y iputils-ping && \
    apt-get -y install build-essential && \
    apt install -y cmake && \
    apt install -y tree && \
    apt install -y gdb && \
    apt-get install -y git && \
    apt-get install -y python3-pip && \
    # install g++ 13
    apt-get update && \
    apt-get upgrade -y && \
    apt install software-properties-common -y && \
    add-apt-repository ppa:ubuntu-toolchain-r/test && \
    apt update && \
    apt install gcc-13 g++-13 -y && \
    update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-13 13 --slave /usr/bin/g++ g++ /usr/bin/g++-13 && \
    update-alternatives --config gcc 

# Dependencies layer # PCL and VTK and Pyvista and Halcon
RUN apt-get update -y && \
    apt-get install -y libvtk7-dev python3-vtk7 && \
    apt install -y libpcl-dev &&\
    apt install -y python3-pcl &&\
    apt-get install -y python3-tk &&\
    pip install numpy vtk &&\
    pip install pyvista && \
    pip install pyvistaqt meshio &&\
    pip install mvtec-halcon==23050 &&\
    pip install transforms3d &&\
    pip install scipy

## Set halcon environment variables
ENV HALCONROOT="/opt/MVTec/HALCON-23.05-Progress"
ENV LD_LIBRARY_PATH="$HALCONROOT/lib/x64-linux"

RUN echo ". /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN echo ". /catkin_ws/devel/setup.bash" >> ~/.bashrc

RUN echo "ALL Done"