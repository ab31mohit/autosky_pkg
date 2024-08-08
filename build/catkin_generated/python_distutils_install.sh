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

echo_and_run cd "/home/mohit/ros1_workspace/autosky_ws/src/autosky_pkg"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/usr/local/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/usr/local/lib/python3/dist-packages:/home/mohit/ros1_workspace/autosky_ws/src/autosky_pkg/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/mohit/ros1_workspace/autosky_ws/src/autosky_pkg/build" \
    "/usr/bin/python3" \
    "/home/mohit/ros1_workspace/autosky_ws/src/autosky_pkg/setup.py" \
    egg_info --egg-base /home/mohit/ros1_workspace/autosky_ws/src/autosky_pkg/build \
    build --build-base "/home/mohit/ros1_workspace/autosky_ws/src/autosky_pkg/build" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/usr/local" --install-scripts="/usr/local/bin"
