import pymel.core as pmc

def rebuildDagPose():
    """
    Walks through bind pose data in selected skeleton and consolidates it down to one new bindPose node
    Directly inspired by Nathan Horne's NT_rebuildDagPose.mel script
    """

    dagPoses = set()
    connectedSkinClusters = set()
    selection = pmc.selected()
    joints = pmc.listRelatives(selection[0], path=True, allDescendents=True, type='joint')
    joints.insert(0, selection[0])

    for jnt in joints:
        dagPoses.update(jnt.listConnections(type='dagPose'))

    for dag in dagPoses:
        connectedSkinClusters.update(dag.listConnections(type='skinCluster'))

    pmc.delete(dagPoses)    
    pmc.select(joints, replace=True)            
    newDagPose = pmc.dagPose(save=True, selection=True, bindPose=True)

    print 'New dagPose, {0}, created'.format(newDagPose.shortName())

    for sc in connectedSkinClusters:
        print 'Connecting {0}.message to {1}.bindPose'.format(newDagPose.shortName(), sc.shortName())
        newDagPose.message.connect(sc.bindPose)

def updateBindPose():
    """
    Updates bind pose to the selected joint hierarchies current state
    """

    dag = pmc.dagPose(q=True, bindPose=True)
    objects = pmc.dagPose(dag, q=True, members=True)
    for obj in objects:
        pmc.dagPose(obj, reset=True, name=dag[0])
