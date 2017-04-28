.. _training:

:orphan:

Training
########
.. highlight:: ectosh

Once some observations are stored in the database, models can be built from
them. You can store some observations in the database using the procedure
described in the :ref:`capture documentation <orkcapture:ork_capture>`_.

Config File
***********

Training, just like recognition, requires a
:ref:`configuration file <configuration>`, but it usually only contains one cell
that defines a pipeline that reads data from the database and computes a model.
Then again, nothing prevents you to train several models at the same time by
executing several pipelines--but this would be an advanced use case!

Use
***

Training requires a configuration file, which is specified through the ``-c``
option. You can choose whatever configuration file; a few are provided in
``object_recognition_server/conf``.

Some of the options in there can be overriden through the command line
for convenience. Change the following parameters to your needs:

* the ``object_ids`` should be the list of object ids you want to train on.
  If you want, you can also use object_names that are more human readable.
  object_ids is of the form ["6b3de86feee4e840280b4caad90003fb"] but there
  are two special options: if it is "all", then all models are recomputed;
  if it is "missing", only the missing models are computed.

.. toggle_table::
  :arg1: ROS
  :arg2: Without ROS

.. toggle:: ROS

   The training script can be run as follows:

   .. code-block:: sh

      rosrun object_recognition_core training -c `rospack find object_recognition_tod`/conf/training.ork --visualize

.. toggle:: Without ROS

   Run the training.py script in ``/apps``:

   .. code-block:: sh

      ./object_recognition_core/apps/training -c object_recognition_tod/conf/training.ork --visualize

A typical command line session might look like::

   % apps/training -c config_training.txt --commit
   Prepare G2O: processing image 65/65
   Performing full BA:
   iteration= 0     chi2= 168324165740673896546304.000000   time= 39.2803   cumTime= 39.2803        lambda= 154861.907021 edges= 64563     schur= 1
   Persisted

Command Line Interface
**********************
.. program-output:: ../../../apps/training --help
   :in_srcdir:
