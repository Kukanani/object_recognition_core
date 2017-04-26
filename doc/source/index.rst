.. _index:

################################################################################
Object Recognition Kitchen
################################################################################


--------------------------------------------------------------------------------
Notice: Documentation Update in Progress
--------------------------------------------------------------------------------

Hello ORK Users! The documentation is in the process of being updated and
upgraded! Information may have moved since the last time you visited. If you
can't find anything, please raise an issue on
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

ORK attempts to take care of all the non-vision aspects of the
problem for you (database management, inputs/outputs handling,
robot/ROS integration ...), so that you can focus on writing a recognition
algorithm and detector that fits your needs.

``ORK`` is built on top of `ecto <http://plasmodic.github.com/ecto>`_,
which is a lightweight hybrid C++/Python framework for organizing computations
as directed acyclic graphs.

--------------------------------------------------------------------------------
Getting Started
--------------------------------------------------------------------------------

To learn the basics of ORK, please read the
:ref:`Getting Started Guide <getting_started>`.

--------------------------------------------------------------------------------
Install
--------------------------------------------------------------------------------

ORK can be installed in several different ways: from source or from ``apt``
packages, using ROS or without ROS, etc. The :ref:`Installation <install>`
page provides detailed instructions (beyond those in the Getting Started Guide)
on the different ways to install ORK.

--------------------------------------------------------------------------------
Quickguide (outdated)
--------------------------------------------------------------------------------

We know you can't wait; if you don't care about the intricacies and want to have
a quick overview, follow the :ref:`Quick Guide <quickguide>`

--------------------------------------------------------------------------------
Tutorials
--------------------------------------------------------------------------------

Ok, now that you know a little, you can follow the
:ref:`Tutorials <orktutorials:object_recognition_tutorials>`.

--------------------------------------------------------------------------------
General Usage
--------------------------------------------------------------------------------

Now that you have a bit more time, we suggest you read about the
:ref:`Infrastructure <infrastructure>` to understand how to interact with
``ORK``. You can then go through the different steps of object recognition:

* :ref:`Data Capture <orkcapture:ork_capture>` ...
* ... from which you can perform :ref:`Training <training>` of object models ...
* ... that you can then use for :ref:`Detection <detection>`

--------------------------------------------------------------------------------
ROS integration
--------------------------------------------------------------------------------

The recognition kitchen was built in a ROS agnostic way, but ROS components
were also developed for integration with the ROS ecosystem (e.g. publishers,
subscribers, actionlib server, RViz plugin ...). For more info, check out the
:ref:`ROS Integration <orkros:ros>`.

--------------------------------------------------------------------------------
Recognition Pipelines
--------------------------------------------------------------------------------

Several object recognition pipelines have been implemented for this framework.
Their documentation is work in progress:

+----------------------------------------------+--------------+------------------------------+--------------------------------------------------------------+
| Techniques                                   | 2D/3D        | Types of object              | Limitations                                                  |
+==============================================+==============+==============================+==============================================================+
| :ref:`LINE-MOD <orklinemod:line_mod>`        | 2D and/or 3D | * rigid, Lambertian          | * does not work with partial occlusions                      |
+----------------------------------------------+--------------+------------------------------+--------------------------------------------------------------+
| :ref:`tabletop <orktabletop:tabletop>`       | 3D           | * rigid, Lambertian          | * scales linearly with the number of objects                 |
|                                              |              | * rotationally symmetric     | * the object is assumed to be on a table with no 3d rotation |
|                                              |              | * also finds planar surfaces |                                                              |
+----------------------------------------------+--------------+------------------------------+--------------------------------------------------------------+
| :ref:`TOD <orktod:tod>`                      | 2D and 3D    | * rigid, Lambertian          |                                                              |
|                                              |              | * textured                   |                                                              |
+----------------------------------------------+--------------+------------------------------+--------------------------------------------------------------+
| :ref:`transparent objects                    | 2D and 3D    | * rigid and transparent      | * Training has to be done on a painted version of the object |
| <orktransparentobjects:transparent_objects>` |              |                              |                                                              |
+----------------------------------------------+--------------+------------------------------+--------------------------------------------------------------+

--------------------------------------------------------------------------------
Tools
--------------------------------------------------------------------------------

There are several tools that are used by some of the pipeline, and you might
need them for your own work or pipelines:

  * :ref:`Reconstruction <orkreconstruction:reconstruction>`

--------------------------------------------------------------------------------
Developers' Guide
--------------------------------------------------------------------------------

ORK is designed to be modular, to allow developers to extend it and replace
components. For example, you can use a different database backend to store
a list of recognition objects, or write your own detector.

For more details, please read the :ref:`Developer Guide <ork_developer>`

################################################################################
Contact
################################################################################

For bug reports and comments, please use the
`GitHub infrastructure <https://github.com/wg-perception/>`_ or join us on the
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