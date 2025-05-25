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

echo_and_run cd "/home/jungejblue/xytron_404/xycar_ws/src/kookmin/ROS-TCP-Endpoint"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/jungejblue/xytron_404/xycar_ws/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/jungejblue/xytron_404/xycar_ws/install/lib/python3/dist-packages:/home/jungejblue/xytron_404/xycar_ws/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/jungejblue/xytron_404/xycar_ws/build" \
    "/usr/bin/python3" \
    "/home/jungejblue/xytron_404/xycar_ws/src/kookmin/ROS-TCP-Endpoint/setup.py" \
    egg_info --egg-base /home/jungejblue/xytron_404/xycar_ws/build/kookmin/ROS-TCP-Endpoint \
    build --build-base "/home/jungejblue/xytron_404/xycar_ws/build/kookmin/ROS-TCP-Endpoint" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/jungejblue/xytron_404/xycar_ws/install" --install-scripts="/home/jungejblue/xytron_404/xycar_ws/install/bin"
