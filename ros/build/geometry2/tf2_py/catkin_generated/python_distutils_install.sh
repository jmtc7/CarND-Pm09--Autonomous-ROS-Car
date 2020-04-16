#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/jmtc7/Learning/Autonomous-ROS-Car/ros/src/geometry2/tf2_py"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/jmtc7/Learning/Autonomous-ROS-Car/ros/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/jmtc7/Learning/Autonomous-ROS-Car/ros/install/lib/python3/dist-packages:/home/jmtc7/Learning/Autonomous-ROS-Car/ros/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/jmtc7/Learning/Autonomous-ROS-Car/ros/build" \
    "/usr/bin/python3" \
    "/home/jmtc7/Learning/Autonomous-ROS-Car/ros/src/geometry2/tf2_py/setup.py" \
    build --build-base "/home/jmtc7/Learning/Autonomous-ROS-Car/ros/build/geometry2/tf2_py" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/jmtc7/Learning/Autonomous-ROS-Car/ros/install" --install-scripts="/home/jmtc7/Learning/Autonomous-ROS-Car/ros/install/bin"
