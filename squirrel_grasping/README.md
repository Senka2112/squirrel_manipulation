This package provides necessary functionality for blind top-grasping of an object given its pose.
Depending on which hand you are using, either do a
- `roslaunch squirrel_grasping squirrel_grasping_metahand.launch` for the Metahand
and
- `roslaunch squirrel_grasping squirrel_grasping_softhand.launch` for the SoftHand.

In both cases, this will start an action server you can readily use by connecting to it. For the Metahand, this server is called `metahand_grasp_server` and for the SoftHand `softhand_grasp_server`. Both require a `BlindGraspAction` as a request. Observe that the `heap_bounding_cylinder` (`squirrel_object_perception_msgs/BCylinder`) property of the action needs to be (despite a correcft pose) provided, as the action server uses the `height` attirbute of the bounding box to compute the grasping pose.

As a side note, the provided pose can be in any frame as (for both hands) the server internally transforms it properly.

For a minimal working example please see `grasp_test.py` under `test`
