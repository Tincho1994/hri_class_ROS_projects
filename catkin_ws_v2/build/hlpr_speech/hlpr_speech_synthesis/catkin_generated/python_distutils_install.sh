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
    DESTDIR_ARG="--root=$DESTDIR"
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/martin/HRI/catkin_ws_v2/src/hlpr_speech/hlpr_speech_synthesis"

# snsure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/martin/HRI/catkin_ws_v2/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/martin/HRI/catkin_ws_v2/install/lib/python2.7/dist-packages:/home/martin/HRI/catkin_ws_v2/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/martin/HRI/catkin_ws_v2/build" \
    "/usr/bin/python" \
    "/home/martin/HRI/catkin_ws_v2/src/hlpr_speech/hlpr_speech_synthesis/setup.py" \
    build --build-base "/home/martin/HRI/catkin_ws_v2/build/hlpr_speech/hlpr_speech_synthesis" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/home/martin/HRI/catkin_ws_v2/install" --install-scripts="/home/martin/HRI/catkin_ws_v2/install/bin"
