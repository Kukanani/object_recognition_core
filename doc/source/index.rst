.. _index:

################################################################################
Object Recognition Kitchen
################################################################################


--------------------------------------------------------------------------------
Notice: Documentation Update in Progress
--------------------------------------------------------------------------------

Hello ORK Users! The documentation is in the process of being updated and
upgraded. Information may have moved since the last time you visited.

If you can't find something, please raise an issue on
`GitHub <https://github.com/wg-perception/object_recognition_core>`_. Thanks!

--------------------------------------------------------------------------------
Overview
--------------------------------------------------------------------------------

The Object Recognition Kitchen (``ORK``) is a set of tools for object
recognition from camera data. The project was originally started at Willow
Garage, and although it is now a bit old, it still can prove useful for basic
object detection.

Even in the most cutting-edge research papers, there is not one canonical "best"
way to perform object recognition. Objects can be textured, non-textured,
transparent, articulated, etc., and different approaches work best for each type
of object. For this reason, ORK was designed to run several object recognition
techniques simultaneously, each designed to detect a different type of object.

When using/developing a new method you created or found, you usually have
to recreate the following functionality:

- A way to store/query/use your training data
- A way to store/query/use your models
- An easy to train your objects
- Where your data is coming from (cameras, ROS rosbag, ROS topics)
- Where your data is going to
- ROS integration, by listening and publishing to the right topics
- The ability to compare detectors, by having several running at once

``ORK`` takes care of this "plumbing" by providing base classes and a flexible
infrastructure, so you can work on the fun part: the detection algorithms.

``ORK`` is built on top of `ecto <http://plasmodic.github.com/ecto>`_,
which is a lightweight hybrid C++/Python framework for organizing computations
as directed acyclic graphs. It can be installed in several different ways: from
source or from ``apt`` packages, and either with ROS support or without.

--------------------------------------------------------------------------------
Getting Started
--------------------------------------------------------------------------------

- :ref:`Getting Started Guide <getting_started>` -- This tutorial will guide you
  through basic installation, training, and detection.

--------------------------------------------------------------------------------
Reference Documentation
--------------------------------------------------------------------------------

- :ref:`Installation <installation>`
  -- detailed instructions (beyond those in the Getting Started Guide)
  on the different ways to install ORK.
- :ref:`Configuration Files <configuration>`
  -- How to use ``ORK`` configuration files
- :ref:`Data Capture <orkcapture:ork_capture>`
  -- How to capture training data from real-world objects
- :ref:`Training <training>`
  -- Building recognition models from captured data
- :ref:`Detection <detection>`
  -- Detecting objects using recognition models
- :ref:`Pipelines <pipelines>`
  -- Different types of detectors in ORK
- :ref:`Pipeline tutorials <orktutorials:object_recognition_tutorials>`
  -- Tutorials on using different detectors
- :ref:`Databases <databases>`
  -- Information on database backends, for storing recognition models
- :ref:`ROS Integration <orkros:ros>`
  -- Details on ``ORK``'s integration with ROS
- :ref:`Reconstruction Tool <orkreconstruction:reconstruction>`
  -- An ``ORK`` tool for reconstructing 3D models from several images

--------------------------------------------------------------------------------
Developers' Guide
--------------------------------------------------------------------------------

ORK is designed to be modular, to allow developers to extend it and replace
components. For example, you can use a different database backend to store
a list of recognition objects, or write your own detector.

For more details, please read the :ref:`Developer Guide <ork_developer>`.

################################################################################
Contact
################################################################################

For bug reports, issues, and to contribute, please use the appropriate
repository page on `GitHub <https://github.com/wg-perception/>`. For discussion
and troubleshooting, please visit the
`Google Group <https://groups.google.com/forum/#!forum/object-recognition-kitchen>`_.

--------------------------------------------------------------------------------
BibTeX Citation
--------------------------------------------------------------------------------

If you want to cite this work, please use the BibTeX reference:

.. code-block:: latex

   @misc{ork_ros,
      Author = {Willow Garage, ROS community},
      Title = "{ORK}: {O}bject {R}ecognition {K}itchen},
      howpublished =
          {\url{https://github.com/wg-perception/object_recognition_core}}
   }