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

echo_and_run cd "/media/zjj/Elements/CQU_ZJJ/FAST-LIVO2/src/rpg_vikit/src/vikit_py"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/media/zjj/Elements/CQU_ZJJ/FAST-LIVO2/src/rpg_vikit/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/media/zjj/Elements/CQU_ZJJ/FAST-LIVO2/src/rpg_vikit/install/lib/python3/dist-packages:/media/zjj/Elements/CQU_ZJJ/FAST-LIVO2/src/rpg_vikit/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/media/zjj/Elements/CQU_ZJJ/FAST-LIVO2/src/rpg_vikit/build" \
    "/media/zjj/Elements/anaconda3/bin/python3" \
    "/media/zjj/Elements/CQU_ZJJ/FAST-LIVO2/src/rpg_vikit/src/vikit_py/setup.py" \
     \
    build --build-base "/media/zjj/Elements/CQU_ZJJ/FAST-LIVO2/src/rpg_vikit/build/vikit_py" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/media/zjj/Elements/CQU_ZJJ/FAST-LIVO2/src/rpg_vikit/install" --install-scripts="/media/zjj/Elements/CQU_ZJJ/FAST-LIVO2/src/rpg_vikit/install/bin"
