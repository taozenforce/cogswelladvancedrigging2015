__author__ = 'Sergio Sykes'
__version__ = 'Fall 2015'

from itertools import izip
import pymel.core as pmc

from hellamath import getPoleVectorPosition
from advutils import alignObjects, getAttribute

JOINT_BASE_PREFIX = 'rig'  # existing prefix of control joints that will be queried in search/replace functions within script
IK_JOINT_PREFIX = 'ikj'  # Prefix convention for duplicated joints for IK systems
FK_JOINT_PREFIX = 'fkj'  # Prefix convention for duplicated joints for FK systems


def makeIkFkJoints(joints, attribute=None, stretchy=False,
                   jointPrefix=JOINT_BASE_PREFIX, ikJointPrefix=IK_JOINT_PREFIX, fkJointPrefix=FK_JOINT_PREFIX):
    """
    Creates 2 duplicate hierarchies from passed in joints
    (please make sure the list of joints are in same hierarchy)
    Blender is connected to attribute
    If stretchy is true, translation is connected for all the joints, except the root joint
    """

    startJoint = joints[0]
    rootRotation = startJoint.getRotation('world')
    rootTranslation = startJoint.getTranslation('world')

    # pymel bug, duplicating without renaming may result in some console warnings and errors
    # use a temp name until we learn the actual names for each joints
    fkJoints = pmc.duplicate(joints, parentOnly=True, name='fkTEMP')

    # FK Joint setup

    # Use our prefix to uniquely rename duplicated joints based on the original copy
    # third parameter in jnt.replace allows me to limit the number of string replacements,
    # so I can avoid renaming joints containing 'right' in their name (I'm using 'rig' as my joint prefix)
    for jnt, fkj in izip(joints, fkJoints):
        fkj.rename(jnt.replace(jointPrefix, fkJointPrefix, 1))

    # Self grouping the duplicated FK root joint
    # Remember to not freeze transforms to ensure a proper connection to the original
    fkGrp = pmc.group(empty=True,
                      name='grp_fk_{0}_joints'.format(fkJoints[0].shortName()))
    fkGrp.setRotation(rootRotation, 'world')
    fkGrp.setTranslation(rootTranslation, 'world')

    # Could use a parent constrain here instead to the first parent
    # I prefer pointOrient for the extra control and future proofing
    parent = startJoint.firstParent2()
    if parent:
        pmc.parentConstraint(parent, fkGrp, maintainOffset=True)

    pmc.parent(fkJoints[0], fkGrp)

    # IK Joint setup
    # Mostly the same code, this could be remade into a simpler function

    ikJoints = pmc.duplicate(joints, parentOnly=True, name='ikTEMP')

    for jnt, ikj in izip(joints, ikJoints):
        ikj.rename(jnt.replace(jointPrefix, ikJointPrefix, 1))

    ikGrp = pmc.group(empty=True,
                      name='grp_ik_{0}_joints'.format(ikJoints[0].shortName()))
    ikGrp.setRotation(rootRotation, 'world')
    ikGrp.setTranslation(rootTranslation, 'world')

    if parent:
        pmc.parentConstraint(parent, ikGrp, maintainOffset=True)

    pmc.parent(ikJoints[0], ikGrp)

    # we may have a attribute that goes between 0-1, 0-10 or even 0-100
    # Create remapValue node to help us remap the value, if needed
    outputAttr = None
    if attribute:
        if isinstance(attribute, basestring):
            outputAttr = pmc.Attribute(attribute)
        else:
            outputAttr = attribute

        attrMaxValue = outputAttr.getMax()
        if attrMaxValue > 1.0:
            attrNormalizeNode = pmc.shadingNode('remapValue', asUtility=True,
                                                name='rmv_{0}_ikfk_attr'.format(startJoint.shortName()))
            pmc.connectAttr(attribute, attrNormalizeNode.inputValue)
            attrNormalizeNode.inputMax.set(attrMaxValue)
            outputAttr = attrNormalizeNode.outValue

    # Connecting duplicate joint chains to original hierarchy
    # enumerate gives me an index, plus the joint to work with
    blendNodes = list()
    for i, jnt in enumerate(joints):
        fkJoint = fkJoints[i]
        ikJoint = ikJoints[i]

        # For this example, I connect my IKFK chains using blendColors nodes.
        # Using constraints instead is fine, but in place of this code,
        # you'll instead be connecting to the constraint's
        # weight values
        blender = pmc.shadingNode('blendColors', asUtility=True,
                                  name='bln_{0}_ikfk'.format(jnt.shortName()))
        blender.output.connect(jnt.rotate)

        fkJoint.rotate.connect(blender.color1)
        ikJoint.rotate.connect(blender.color2)
        blendNodes.append(blender)

        # Stretching joints by using X translation,
        # simpler to deal with and causes the least amount of headaches
        if stretchy is not None and jnt != startJoint:
            stretchyblender = pmc.shadingNode('blendColors', asUtility=True,
                                              name='bln_{0}_ikfk_stretch'.format(jnt.shortName()))

            fkJoint.translateX.connect(stretchyblender.color1R)
            ikJoint.translateX.connect(stretchyblender.color2R)

            stretchyblender.outputR.connect(jnt.translateX)
            blendNodes.append(stretchyblender)

    for bln in blendNodes:
        if outputAttr:
            outputAttr.connect(bln.blender)
        else:
            bln.blender.set(0)

    pmc.select(clear=True)
    return ikGrp, fkGrp, blendNodes


def matchFkToIk(fkControls, msgAttr='ikjoints', autoKey=True):
    """
    Matches fkControls to match the current pose of the underlying ik duplicate joint chains
    Finds the ik joints using a previously created message connection to the attribute msgAttr
    """

    ikJoints = None
    switchControl = None
    switchAttr = None
    for ctl in fkControls:
        if pmc.hasAttr(ctl, msgAttr):
            ikJoints = pmc.listConnections('{0}.{1}'.format(ctl, msgAttr),
                                           destination=False, source=True)

            attr = pmc.listConnections('{0}.ikfk'.format(ctl), destination=False,
                                       source=True, plugs=True, scn=True)[0]
            switchControl = attr.node()
            switchAttr = attr.name(includeNode=False)
            break

    if autoKey:
        frameBeforeCurrent = pmc.currentTime(q=True) - 1
        pmc.setKeyframe(switchControl, attribute=switchAttr, time=frameBeforeCurrent,
                        value=0, outTangentType='step')
        pmc.setKeyframe(fkControls, attribute='rotate', time=frameBeforeCurrent,
                        outTangentType='step')

        pmc.setKeyframe(switchControl, attribute=switchAttr, value=1, outTangentType='step')

    for ikj, ctl in izip(ikJoints, fkControls):
        alignObjects(ctl, ikj, position=False, rotation=True)

    if autoKey:
        pmc.setKeyframe(fkControls, attribute='rotate')

    pmc.headsUpMessage('BAMF!')


def matchIkToFk(ikControl, ikPole, offset=100.0, msgAttr='fkjoints', autoKey=True):
    """
    Matches ikControl and ikPole Vector control to match the current pose of underlying
    fk duplicate joint chains
    Finds the fk joints using a previously created message connection to the attribute msgAttr
    """

    fkJoints = pmc.listConnections('{0}.{1}'.format(ikControl, msgAttr),
                                   destination=False, source=True)

    attr = pmc.listConnections('{0}.ikfk'.format(ikControl), destination=False,
                               source=True, plugs=True, scn=True)[0]
    switchControl = attr.node()
    switchAttr = attr.name(includeNode=False)

    frameBeforeCurrent = pmc.currentTime(q=True) - 1
    if autoKey:
        pmc.setKeyframe(switchControl, attribute=switchAttr, time=frameBeforeCurrent,
                        value=1, outTangentType='step')
        pmc.setKeyframe([ikControl, ikPole], time=frameBeforeCurrent, outTangentType='step')

        pmc.setKeyframe(switchControl, attribute=switchAttr, value=0, outTangentType='step')

    alignObjects(ikControl, fkJoints[-1])
    loc = getPoleVectorPosition(fkJoints, offset, curveGuide=False)
    alignObjects(ikPole, loc, position=True, rotation=False)
    pmc.delete(loc)

    if autoKey:
        pmc.setKeyframe([ikControl, ikPole], time=frameBeforeCurrent)

    pmc.headsUpMessage('BAMF!')


## WORK IN PROGRESS
# TODO flesh out GUI callbacks
class App(object):
    """
    Window GUI for creating IKFK joints in the current scene
    """

    def __init__(self):
        """
        Initializing default member variables
        """
        self._jointPrefix = JOINT_BASE_PREFIX
        self._ikJointPrefix = IK_JOINT_PREFIX
        self._fkJointPrefix = FK_JOINT_PREFIX
        self.mainWindow = None

    def Draw(self, windowName):
        """
        Creates the UI for using the script
        """

        if pmc.window(windowName, exists=True):
            pmc.deleteUI(windowName, window=True)

        self.mainWindow = pmc.window(windowName, title='Advanced Rigging IKFK Maker')
        layout = pmc.formLayout()
        existText = pmc.text(label='Existing Joints', align='left')
        self._jointTsc = pmc.textScrollList(numberOfRows=8, width=200, height=300)
        loadJointsBtn = pmc.button(label='Load Sel.', width=100, command=pmc.Callback(self._loadJoints))

        nameText = pmc.text(label='Naming Convention', align='left')
        self._autoPrefixCheck = pmc.checkBox(label='Auto Prefix', value=True)
        self._basePrefixField = pmc.textFieldGrp(label='Base', text=JOINT_BASE_PREFIX, columnWidth2=(25, 100))
        self._ikPrefixField = pmc.textFieldGrp(label='IK', text=IK_JOINT_PREFIX, columnWidth2=(25, 100))
        self._fkPrefixField = pmc.textFieldGrp(label='FK', text=FK_JOINT_PREFIX, columnWidth2=(25, 100))

        self._attrField = pmc.textFieldGrp(label='IKFK switch attribute', placeholderText='controlname.attribute',
                                           ann='Existing control in scene for connecting IKFK switch. '
                                               'Attribute will be created if it doesn\'t exist',
                                           columnWidth2=(125, 175))

        self._stretchyCheck = pmc.checkBox(label='Stretchy Connections?', value=True)

        createBtn = pmc.button(label='Create', width=75, command=pmc.Callback(self._callback, True))
        applyBtn = pmc.button(label='Apply', width=75, command=pmc.Callback(self._callback, False))
        closeBtn = pmc.button(label='Close', width=75, command=pmc.Callback(pmc.deleteUI, self.mainWindow))

        # Joint widget placement
        layout.attachForm(existText, 'left', 5)
        layout.attachForm(existText, 'top', 5)
        layout.attachForm(self._jointTsc, 'left', 5)
        layout.attachForm(loadJointsBtn, 'left', 105)
        layout.attachControl(self._jointTsc, 'top', 5, existText)
        layout.attachControl(loadJointsBtn, 'top', 5, self._jointTsc)

        # Prefix widget layout placement
        layout.attachForm(nameText, 'top', 5)
        layout.attachControl(nameText, 'left', 5, self._jointTsc)
        layout.attachControl(self._autoPrefixCheck, 'left', 5, self._jointTsc)
        layout.attachControl(self._autoPrefixCheck, 'top', 5, nameText)
        layout.attachControl(self._basePrefixField, 'left', 5, self._jointTsc)
        layout.attachControl(self._basePrefixField, 'top', 5, self._autoPrefixCheck)
        layout.attachControl(self._ikPrefixField, 'left', 5, self._jointTsc)
        layout.attachControl(self._ikPrefixField, 'top', 5, self._basePrefixField)
        layout.attachControl(self._fkPrefixField, 'left', 5, self._jointTsc)
        layout.attachControl(self._fkPrefixField, 'top', 5, self._ikPrefixField)

        # Controller and stretchiness widget layout placement
        layout.attachForm(self._attrField, 'left', 5)
        layout.attachForm(self._stretchyCheck, 'left', 100)
        layout.attachControl(self._attrField, 'top', 5, loadJointsBtn)
        layout.attachControl(self._stretchyCheck, 'top', 5, self._attrField)

        # Layout for Create/Apply/Close buttons
        layout.attachForm(createBtn, 'left', 55)
        layout.attachControl(createBtn, 'top', 5, self._stretchyCheck)
        layout.attachControl(applyBtn, 'top', 5, self._stretchyCheck)
        layout.attachControl(applyBtn, 'left', 5, createBtn)
        layout.attachControl(closeBtn, 'top', 5, self._stretchyCheck)
        layout.attachControl(closeBtn, 'left', 5, applyBtn)

        self.mainWindow.setWidthHeight((347, 443))
        self.mainWindow.show()

    def _callback(self, closeGUI):
        joints = map(pmc.PyNode, self._jointTsc.getAllItems())
        attribute = None
        rawControlText = self._attrField.getText()
        if rawControlText:
            control, attrname = rawControlText.partition('.')[::2]

            if not attrname:
                pmc.warning('IKFK :: can\'t read attribute specified in IKFK switch attribute. '
                            'Make sure to specify using control.attr format!')
                return

            if not pmc.objExists(control):
                pmc.warning('IKFK :: Controller {0} not found in scene!'.format(control))
                return

            attribute = getAttribute(control, attrname, min=0, max=1, defaultValue=0, keyable=True)
        else:
            choice = pmc.confirmDialog(title='IKFK - No Attribute Specified',
                                       message='I noticed you didn\'t specify a controller for connecting the IKFK '
                                               'switch.\nDo you want to go ahead and complete the IKFK setup and '
                                               'you can connect a controller later?',
                                       button=['Yes', 'No'], defaultButton='Yes', cancelButton='No', dismissString='No')

            if choice == 'No':
                return

        stretchy = self._stretchyCheck.getValue()
        jointPrefix = self._basePrefixField.getText()
        ikJointPrefix = self._ikPrefixField.getText()
        fkJointPrefix = self._fkPrefixField.getText()

        result = makeIkFkJoints(joints, attribute, stretchy, jointPrefix, ikJointPrefix, fkJointPrefix)

        if not rawControlText:
            pmc.select(result[-1])
            pmc.confirmDialog(title='IKFK Setup Complete BUT...', message='IKFK creation successful. '
                                                                          'Take a note of the selected blendColors as '
                                                                          'they\'ll need to be connected to your IKFK'
                                                                          'controller.\nEnjoy!')

        if closeGUI:
            pmc.deleteUI(self.mainWindow, window=True)

    def _loadJoints(self):
        self._jointTsc.removeAll()

        joints = pmc.selected(type='joint')
        if not len(joints):
            return

        self._jointTsc.extend(joints)

        if self._autoPrefixCheck.getValue():
            result = joints[0].name().split('_')
            if len(result) > 1:
                self._basePrefixField.setText(result[0])


# MAIN EXECUTION
def GUI():
    global MAIN_WINDOW

    MAIN_WINDOW = App()
    MAIN_WINDOW.Draw('AdvancedRiggingIKFK')
