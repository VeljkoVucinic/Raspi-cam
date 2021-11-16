#!/bin/bash

# Prüfe, ob die Datei CMakeLists.txt existiert
if [[ -f "nodetemplate/CMakeLists.txt" ]]; then
    # Hier wird aus der CMakeLists.txt der Package- und Nodename geholt
    # Der Packagename ist der Inhalt des project(...)-Tags
    PKG_NAME=$(grep -oP '^project\(\K\w+' nodetemplate/CMakeLists.txt)
    # Der Nodename ist der Name der Executable
    NODE_NAME=$(grep -oP '^add_executable\(\K\w+' nodetemplate/CMakeLists.txt)
    if [[ ! -z "$PKG_NAME" ]]; then
        if [[ ! -z "$NODE_NAME" ]]; then
            echo "Preparing Dockerfile ..."
            DOCKER_IMAGE="arm64v8/ros:foxy"
            uname -m | grep "x86_64" &> /dev/null
            if [ $? == 0 ]; then
                DOCKER_IMAGE="osrf/ros:foxy-desktop"
            fi

            sed -i "s#DOCKER_IMAGE_HERE#$DOCKER_IMAGE#g" ./Dockerfile

	        echo "Building image ..."
            # Temporärer Docker-Container wird erstellt
            docker build -t temp_ros2_$PKG_NAME . --build-arg PKG_NAME=$PKG_NAME --build-arg NODE_NAME=$NODE_NAME
            if [ $? -eq 0 ]; then
                echo "Setting up image ..."

                # Temporärer Docker-Container wird wird ausgeführt
                docker run --privileged --network host --entrypoint=/bin/bash --name temp_container_ros2_$PKG_NAME -it temp_ros2_$PKG_NAME /root/setup.sh

                # Für eine Ausführung von Containern, die eine Raspberry-Pi-Kamera benötigen:
                # docker run -v /opt/vc:/opt/vc -v /home/pi:/out --device /dev/vchiq --privileged --env LD_LIBRARY_PATH=/opt/vc/lib --network host --entrypoint=/bin/bash --name temp_container_ros2_$PKG_NAME -it temp_ros2_$PKG_NAME /root/setup.sh

                if [ $? -eq 0 ]; then
                    # Aus temporärem Container mit Änderungen endgültigen Container machen
                    echo "Committing container temp_container_ros2_$PKG_NAME ($(docker ps -a -q --filter="NAME=temp_container_ros2_$PKG_NAME"))..."
                    docker commit $(docker ps -a -q --filter="NAME=temp_container_ros2_$PKG_NAME") ros2_$PKG_NAME
                    docker rm $(docker ps -a -q --filter="NAME=temp_container_ros2_$PKG_NAME")
                    docker rmi temp_ros2_$PKG_NAME --force
                fi
            fi
        else
            echo "Couldn't retrieve node name."
            echo "Please make sure that add_executable(...) is set in CMakeLists.txt"
        fi
    else
        echo "Couldn't retrieve package name."
        echo "Please make sure that project(...) is set in CMakeLists.txt"
    fi
else
    echo "CMakeLists.txt not found."
    echo "Please make sure that you're in the right directory."
fi
