__author__ = 'Sergio Sykes'
__version__ = 'Fall 2015'


def basicStretchyIK(ikHandle):
    """
    Psuedocode and tips: Creating a stretchy IK system
    """
    # Find the joints connected to the IK Handle
    #
    # Create two locators.
    # Point Constrain one locator to the first joint,
    # Constrain the second locator to the ikHandle. MaintainOffset should be set to False for both
    #
    # Create a distanceBetween node, and connect the locators' worldPosition into point1 and point2
    # Create a normalize multiplyDivide node that divides the current distance, by the max distance.
    # The max distance can be calculated by adding up the translateX values in the joint chain (except the first joint)
    #
    # Create and connect clamp node to maintain the current length and also cap the maximum mount the joints can stretch.
    #
    # For each child joints, create multiplyDivide nodes to multiply their current length by scale value outputted by the clamp node.
    #
    # Connect the multiplyDivide nodes to the skeleton
    #
    # Return the start and end locators as a list or tuple
    #
    # (Bonus) Add an option in this function to either stretch joints by translation or scale
    # (Bonus) Add an option to an animation control, to toggle the stretch on or off
    pass

"""
Useful Commands to look into using:

Python:
    -make sure you're familiar with the for() loop
    -Distance is always a positive value, so use abs() when summing up the joint lengths for mirrored joints

PyMEL/cmds
    -ikHandle() is a powerful command that has various properties you can get using the query=True flag.
        -if using PyMEL, the pymel object, nodetypes.ikHandle provides more compact formatting for querying exisitng ikHandles
        -you can get the list of joints with ikHandle command, except for the end joint. Check the docs for the command flags
            Find some other means to find the last joint!
            (Hint: After creating an ikHandle, go into the Hypergraph try graphing connections on the nodes created by a new IK
            to see how they're connected to the joints!)

    -listConnections() is useful for finding the end joint
    -shadingNode() is used for creating math nodes similar to how we do manually in the Node Editor/Hypershade,
        make sure you add the flag asUtility=True, and specify nodes by their name without any whitespaces
"""