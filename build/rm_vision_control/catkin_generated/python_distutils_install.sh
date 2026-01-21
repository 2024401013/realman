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

echo_and_run cd "/home/nvidia/rm_robot/src/rm_vision_control"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/nvidia/rm_robot/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/nvidia/rm_robot/install/lib/python3/dist-packages:/home/nvidia/rm_robot/build/rm_vision_control/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/nvidia/rm_robot/build/rm_vision_control" \
    "/usr/bin/python3" \
    "/home/nvidia/rm_robot/src/rm_vision_control/setup.py" \
     \
    build --build-base "/home/nvidia/rm_robot/build/rm_vision_control" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/nvidia/rm_robot/install" --install-scripts="/home/nvidia/rm_robot/install/bin"
