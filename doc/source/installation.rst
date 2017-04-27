.. _installation:

################################################################################
ORK Installation
################################################################################

--------------------------------------------------------------------------------
Introduction
--------------------------------------------------------------------------------

This guide provides details on the various methods for installing ORK. A
complete ORK installation consists of several parts:

- The dependencies
- The ORK core packages
- The ORK detector packages
- (if using ROS) the ORK ROS packages
- Camera drivers
- A database, for keeping track of objects trained into the detector
- (optional) compiled documentation (i.e., this web page)

This guide goes step by step through the installation of each of these
components. Note that this guide assumes that if you are using ROS, it is
ROS Kinetic. Other versions of ROS should have a similar installation procedure,
but success is not guaranteed.

.. contents::

--------------------------------------------------------------------------------
0. Choose Installation Type
--------------------------------------------------------------------------------

The ROS version of ORK is more heavily used, and has much better supported
packages. Not using ROS will require you to install less packages, but it all
must be built from source.

.. toggle_table::
    :arg1: ROS
    :arg2: Without ROS

--------------------------------------------------------------------------------
1. ORK Dependencies
--------------------------------------------------------------------------------

The first step is to create a catkin workspace:

.. code-block:: sh

    cd ~                   # go to home directory
    mkdir -p ork_ws/src    # create a folder structure for the workspace
    cd ork_ws/src          # enter the source directory

.. toggle:: ROS

    There are two ways to install dependencies for ORK: from source, or from
    ``apt`` packages. Using ``apt`` packages is recommended for most users.
    Building *dependencies* from source is not recommended a developer who is
    also working on the dependency code.

    **Option 1: Use `apt` Packages**

        Run the following command:

        .. code-block:: sh

            sudo apt install git libopenni-dev ros-kinetic-ecto* ros-kinetic-opencv-candidate ros-kinetic-moveit-core

    **Option 2: Build from Source**

        If you choose this option, it is assumed that you know how to find
        packages yourself, download them and any necessary dependencies,
        and troubleshoot any problems while building them from source.

        Clone the following repositories:

        .. code-block:: sh

            git clone http://github.com/plasmodic/ecto
            git clone http://github.com/plasmodic/ecto_image_pipeline
            git clone http://github.com/plasmodic/ecto_openni
            git clone http://github.com/plasmodic/ecto_opencv
            git clone http://github.com/plasmodic/ecto_pcl
            git clone http://github.com/plasmodic/ecto_ros
            git clone http://github.com/wg-perception/opencv_candidate

        You will also need these additional packages, which you may already
        have, or which you can get from source or ``apt``:

        .. code-block:: sh

            moveit_msgs (from https://github.com/ros-planning/moveit_msgs)
            moveit_core (from https://github.com/ros-planning/moveit)
            openni

        Once you have built and/or installed all of these dependencies, you
        should be ready to move on.

.. toggle:: Without ROS

    If you want to install from source without ROS, you need to have common
    dependencies (OpenCV, PCL) installed and on your path. You also need to
    execute the following:

    .. code-block:: sh

        git clone http://github.com/ros/catkin.git
        ln -s catkin/cmake/toplevel.cmake CMakeLists.txt
        cd ../ && git clone http://github.com/ros-infrastructure/catkin_pkg.git
        export PYTHONPATH=`pwd`/catkin_pkg/src:$PYTHONPATH
        cd src

    ``catkin`` is a set of CMake macros that simplify build and maintenance.

    Next, install ``ecto``:

    .. code-block:: sh

        git clone http://github.com/plasmodic/ecto
        git clone http://github.com/plasmodic/ecto_image_pipeline
        git clone http://github.com/plasmodic/ecto_openni
        git clone http://github.com/plasmodic/ecto_opencv
        git clone http://github.com/plasmodic/ecto_pcl
        git clone http://github.com/plasmodic/ecto_ros
        git clone http://github.com/wg-perception/opencv_candidate

--------------------------------------------------------------------------------
2. ORK Core and Detector
--------------------------------------------------------------------------------

.. toggle:: ROS

    Regardless of how you installed dependencies, the only way to install all ORK
    packages is from source. Some ``apt`` packages exist linked to ros, including
    ``ros-kinetic-object-recognition-core``, for example, but not all ORK
    packages are up to date in this manner, and it's best to just use source for
    everything. Instead of cloning individual repositories from github, you can
    use the provided ``rosinstall`` file by running the following commands:

    .. code-block:: sh

        cd ..
        wstool init src https://raw.github.com/wg-perception/object_recognition_core/master/doc/source/ork.rosinstall.kinetic.plus
        cd src && wstool update -j8
        rosdep install --from-paths . -i -y

.. toggle:: Without ROS

    Once you're in the ``ork_ws/src`` directory, clone code for the ORK core:

    .. code-block:: sh

        git clone http://github.com/wg-perception/object_recognition_core

    Then, clone any (or all) components that you're interested in using:

    .. code-block:: sh

        # Choose any
        git clone http://github.com/wg-perception/capture
        git clone http://github.com/wg-perception/reconstruction
        git clone http://github.com/wg-perception/linemod
        git clone http://github.com/wg-perception/ork_renderer
        git clone http://github.com/wg-perception/tabletop
        git clone http://github.com/wg-perception/tod
        git clone http://github.com/wg-perception/transparent_objects

--------------------------------------------------------------------------------
3. ORK Packages for ROS
--------------------------------------------------------------------------------

.. toggle:: ROS

    Install some tools for visualization and editing the camera's configuration.
    These tools are necessary if you want to do a detection procedure like the
    one listed in the :ref:`Getting Started Guide <getting_started>`.

    .. code-block:: sh

        sudo apt install ros-kinetic-rviz ros-kinetic-rqt-reconfigure

    Add the following repositories to your ``ork_ws/src`` directory as shown:

    .. code-block:: sh

        git clone http://github.com/wg-perception/object_recognition_msgs
        git clone http://github.com/wg-perception/object_recognition_ros
        git clone http://github.com/wg-perception/object_recognition_ros_visualization

.. toggle:: Without ROS

    Skip this step.

--------------------------------------------------------------------------------
4. Build
--------------------------------------------------------------------------------

Now that you've collected all the source code, you can build:


.. toggle:: ROS

    .. code-block:: sh

        catkin_init_workspace
        cd ../
        catkin_make

.. toggle:: Without ROS

    .. code-block:: sh

        cd ../
        mkdir build
        cd build
        cmake ../src
        make

--------------------------------------------------------------------------------
5. Camera Drivers
--------------------------------------------------------------------------------


.. toggle:: ROS

    **ASUS Xtion, Microsoft Kinect 2**

        For the ASUS Xtion Pro or Microsoft Kinect 2, install openni2_camera and
        openni2_launch:

        .. code-block:: sh

            sudo apt install ros-kinetic-openni2-camera ros-kinetic-openni2-launch


    **Orbbec Astra**

        For the Orbbec Astra camera, install astra_camera and
        astra_launch. You also have to update your udev rules to allow access to
        the USB camera. Follow the guide at http://wiki.ros.org/astra_camera.

        .. code-block:: sh

            sudo apt install ros-kinetic-astra-camera ros-kinetic-astra-launch

.. toggle:: Without ROS

    Install any device-specific drivers.

--------------------------------------------------------------------------------
6. Database Backend
--------------------------------------------------------------------------------

Several database backends are possible, as described on the
:ref:`Database reference<databases>`.

More backend tutorials may be added here in the future, but for now, please
use CouchDB (the "default" choice).

**CouchDB**

Run the following command:

.. code-block:: sh

    sudo apt install couchdb

You can check that the database is running as expected by running this command:

.. code-block:: sh

    curl -X GET http://localhost:5984
    # If CouchDB is working, you should get terminal output similar to the following:
    # {"couchdb":"Welcome","version":"1.0.1"}

--------------------------------------------------------------------------------
7. (Optional) Compiling the Documentation
--------------------------------------------------------------------------------

Building the documentation requires several python packages, which you can
install using the ``pip`` package manager. If you plan on doing a lot of
work with Python on your machine, we highly recommend setting up a virtual
environment using ``virtualenv`` and ``virtualenvwrapper``. For more details,
please see
`this tutorial <http://levipy.com/virtualenv-and-virtualenvwrapper-tutorial/>`_.
If you do not want to use a virtual environment, **do not** do a ``sudo pip
install`` to avoid permission errors. Instead, use ``pip install --user`` as
shown below.

**Virtual Environment**

    With your virtual environment activated, run:

    .. code-block:: sh

        pip install -U breathe catkin-sphinx sphinxcontrib-programoutput

**pip Local Packages**

    Run:

    .. code-block:: sh

        pip install -U --user breathe catkin-sphinx sphinxcontrib-programoutput

From the root of your catkin workspace, run the following commands:

.. code-block:: sh

    cd build
    make doxygen
    make sphinx-doc