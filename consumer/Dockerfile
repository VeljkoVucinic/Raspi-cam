FROM DOCKER_IMAGE_HERE
ARG PKG_NAME
ENV PKG_NAME=$PKG_NAME
ARG NODE_NAME
ENV NODE_NAME=$NODE_NAME

# Setting up node
WORKDIR /root/ros2_workspace/src
RUN /bin/bash -c ". /opt/ros/$ROS_DISTRO/setup.bash && ros2 pkg create --build-type ament_cmake --node-name $NODE_NAME $PKG_NAME && cd $PKG_NAME && rm -rf include/* && rm -rf src/*"

COPY src/ /root/ros2_workspace/src/$PKG_NAME/src
COPY include/ /root/ros2_workspace/src/$PKG_NAME/include
COPY CMakeLists.txt /root/ros2_workspace/src/$PKG_NAME
COPY package.xml /root/ros2_workspace/src/$PKG_NAME

COPY entrypoint.sh /root/
COPY setup.sh /root/
