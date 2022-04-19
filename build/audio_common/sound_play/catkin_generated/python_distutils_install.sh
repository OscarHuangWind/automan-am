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

echo_and_run cd "/home/oscar/ws_oscar/automan-am/src/audio_common/sound_play"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/oscar/ws_oscar/automan-am/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/oscar/ws_oscar/automan-am/install/lib/python3/dist-packages:/home/oscar/ws_oscar/automan-am/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/oscar/ws_oscar/automan-am/build" \
    "/usr/bin/python3" \
    "/home/oscar/ws_oscar/automan-am/src/audio_common/sound_play/setup.py" \
    egg_info --egg-base /home/oscar/ws_oscar/automan-am/build/audio_common/sound_play \
    build --build-base "/home/oscar/ws_oscar/automan-am/build/audio_common/sound_play" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/oscar/ws_oscar/automan-am/install" --install-scripts="/home/oscar/ws_oscar/automan-am/install/bin"
