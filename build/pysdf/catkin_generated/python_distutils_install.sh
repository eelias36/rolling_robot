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

echo_and_run cd "/home/eric/rolling_robot/src/pysdf"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/eric/rolling_robot/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/eric/rolling_robot/install/lib/python3/dist-packages:/home/eric/rolling_robot/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/eric/rolling_robot/build" \
    "/usr/bin/python3" \
    "/home/eric/rolling_robot/src/pysdf/setup.py" \
     \
    build --build-base "/home/eric/rolling_robot/build/pysdf" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/eric/rolling_robot/install" --install-scripts="/home/eric/rolling_robot/install/bin"
