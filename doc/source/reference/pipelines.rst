.. _pipelines:

********************************************************************************
Recognition Pipelines
********************************************************************************

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
