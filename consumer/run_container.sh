#!/bin/bash

# Prüfe, ob die Datei CMakeLists.txt existiert
if [[ -f "CMakeLists.txt" ]]; then
    # Hier wird aus der CMakeLists.txt der Package- und Nodename geholt
    # Der Packagename ist der Inhalt des project(...)-Tags
    PKG_NAME=$(grep -oP '^project\(\K\w+' CMakeLists.txt)
    # Der Nodename ist der Name der Executable
    NODE_NAME=$(grep -oP '^add_executable\(\K\w+' CMakeLists.txt)
    if [[ ! -z "$PKG_NAME" ]]; then
        if [[ ! -z "$NODE_NAME" ]]; then
        # Docker-Container wird wird ausgeführt
	    docker run --privileged --entrypoint=/bin/bash --network host ros2_$PKG_NAME /root/entrypoint.sh
        # Für eine Ausführung von Containern, die eine Raspberry-Pi-Kamera benötigen:
        # docker run -v /opt/vc:/opt/vc -v /home/pi:/out --device /dev/vchiq --privileged --env LD_LIBRARY_PATH=/opt/vc/lib --entrypoint=/bin/bash --network host -it ros2_$PKG_NAME /root/entrypoint.sh
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
