# bl_info is metadata regarding author, version, title
#
bl_info = {
    "name": "MarIOnette",
    "author": "Niko Vladimirov",
    "version": (1, 0, 3),
    "blender": (3, 5, 0),
    "location": "Toolbar > MarIOnette",
    "description": "Interfaces with an Arduino and sends bone angles which translate to servo motions and LED values",
    "warning": "",
    "wiki_url": "",
    "tracker_url": "",
    "category": "Animation",
}

import bpy, sys, glob, os, time, math, linecache, requests, platform

try:
    import serial

    print("Starting pyserial")
except:
    import subprocess, pip

    print("No pyserial module found, installing with pip...")

    if platform.system() == "win32":
        print("Installing on Windows...")
        python_exe = os.path.join(sys.prefix, "bin", "python.exe")
        target = os.path.join(sys.prefix, "lib", "site-packages")

        subprocess.call([python_exe, "-m", "ensurepip"])
        subprocess.call([python_exe, "-m", "pip", "install", "--upgrade", "pip"])

        subprocess.call(
            [python_exe, "-m", "pip", "install", "--upgrade", "pyserial", "-t", target]
        )

    else:
        subprocess.check_call([sys.executable, "-m", "pip", "install", "pyserial"])

    import serial

    print("Done installing pyserial, enjoy MarIOnette!")


from numpy import interp
from math import degrees, acos, radians
from mathutils import Matrix, Vector, Euler

import bpy.props as prop
from bpy.props import CollectionProperty
from bpy_extras.io_utils import ImportHelper
from bpy.types import Panel, AddonPreferences, Operator, PropertyGroup

serialport = serial.Serial()
outputvalue = 1500
outputvalue2 = 1500
outputvalue3 = 1500
ledValue = 1500
ledValue2 = 1500
ledValue3 = 1500
updateActive = False
writingfile = False
framenumber = 0
outputfile = 0
cachefilepath = 0
uploadingnimation = False
lastframenumber = -1
uploadSuccess = 0
uploadSuccessTimer = 0
cacheheaderlength = 0
outputvalues = []
outputbytes = []
inputvalues = []
inputvalues2 = []
rawvalues = []
inputservos = []
serialports = [("None", "None", "")]
bytesToSend = 0
readingCacheFile = False
cacheFileOject = 0
last_sync = "NONE"
done_writing = False
currentArmature = ""
currentBone = ""
currentAxis = ""
currentListValue = 0
configSize = 0
headerLen = 0


def num_to_range(num, inMin, inMax, outMin, outMax):
    # return outMin + (float(num - inMin) / float(inMax - inMin) * (outMax - outMin))
    return float(num - inMin) * float(outMax - outMin) / float(inMax - inMin) + (float)(
        outMin
    )


# Helper functions from rigify plugin
def get_pose_matrix_in_other_space(mat, pose_bone):
    """Returns the transform matrix relative to pose_bone's current
    transform space. In other words, presuming that mat is in
    armature space, slapping the returned matrix onto pose_bone
    should give it the armature-space transforms of mat.
    TODO: try to handle cases with axis-scaled parents better.
    """
    rest = pose_bone.bone.matrix_local.copy()
    rest_inv = rest.inverted()

    if pose_bone.parent:
        par_mat = pose_bone.parent.matrix.copy()
        par_inv = par_mat.inverted()
        par_rest = pose_bone.parent.bone.matrix_local.copy()
    else:
        par_mat = Matrix()
        par_inv = Matrix()
        par_rest = Matrix()

    # Get matrix in bone's current transform space
    smat = rest_inv @ (par_rest @ (par_inv @ mat))

    # Compensate for non-local location
    # if not pose_bone.bone.use_local_location:
    # loc = smat.to_translation() * (par_rest.inverted() * rest).to_quaternion()
    # smat.translation = loc

    return smat


def get_local_pose_matrix(pose_bone):
    """Returns the local transform matrix of the given pose bone."""
    return get_pose_matrix_in_other_space(pose_bone.matrix, pose_bone)


def get_bones_rotation(armature, bone, axis):
    mat = get_local_pose_matrix(bpy.data.objects[armature].pose.bones[bone])

    if axis == "X":
        return degrees(mat.to_euler().x)
    elif axis == "Y":
        return degrees(mat.to_euler().y)
    elif axis == "Z":
        return degrees(mat.to_euler().z)


# System agnostic listing of the serial ports
def serial_ports():
    """Lists serial port names

    :raises EnvironmentError:
        On unsupported or unknown platforms
    :returns:
        A list of the serial ports available on the system
    """
    if sys.platform.startswith("win"):
        ports = ["COM%s" % (i + 1) for i in range(256)]
    elif sys.platform.startswith("linux") or sys.platform.startswith("cygwin"):
        # this excludes your current terminal "/dev/tty"
        ports = glob.glob("/dev/tty[A-Za-z]*")
    elif sys.platform.startswith("darwin"):
        ports = glob.glob("/dev/tty.*")
    else:
        raise EnvironmentError("Unsupported platform")

    result = []
    for port in ports:
        try:
            s = serial.Serial(port)
            s.close()
            result.append(port)
        except (OSError, serial.SerialException):
            pass
    return result


# Callback function used for filling the list of serial ports
def update_serial_ports():
    ports = serial_ports()
    ports_len = len(ports)
    global serialports

    # Give option to have no com port selected
    serialports = [("None", "None", "")]

    # Add any serial ports found; this does not give info about each port, so user must know which port the arduino lives in
    for counter in range(0, ports_len):
        serialports.append((str(ports[counter]), str(ports[counter]), ""))

    return True


def serial_port_callback(scene, context):
    global serialports
    # print(serialports)

    return serialports


# Callback function for filling the list of armatures
def my_settings_callback_parent(scene, context):
    scn = bpy.context.scene

    # Give option to have no armature
    items = [("None", "None", "")]

    parents = []

    for obj in bpy.data.objects:
        parents.append(str(obj.name))

    for counter in range(0, len(parents)):
        items.append((str(parents[counter]), str(parents[counter]), ""))

    return items


# Callback function for filling the list of bones
def my_settings_callback_child(scene, context):
    scn = bpy.context.scene

    # Give option to have no bone
    items = [("None", "None", "")]

    # context.scene.objects["Armature"].pose.bones["Upper Arm"]
    children = []

    if scn.my_list[scn.list_index].ParentSelect != "None":

        for obj in scn.objects[scn.my_list[scn.list_index].ParentSelect].pose.bones:
            children.append(str(obj.name))

        for counter in range(0, len(children)):
            items.append((str(children[counter]), str(children[counter]), ""))

    return items


def my_settings_callback_geonode(scene, context):
    # scn = bpy.context.scene
    # scn = bpy.context.evaluated_depsgraph_get()

    # Give option to have no object port selected
    items = [("None", "None", "")]

    geoNode = []

    for obj in bpy.data.objects:
        geoNode.append(str(obj.name))

    for counter in range(0, len(geoNode)):
        items.append((str(geoNode[counter]), str(geoNode[counter]), ""))

    return items


def my_settings_callback_node(scene, context):
    scn = bpy.context.scene
    dgEv = bpy.context.evaluated_depsgraph_get()

    # Give option to have no material selected
    items = [("None", "None", "")]

    node = []

    if scn.my_list2[scn.list_index2].GeoSelect != "None":

        for obj in (
            bpy.data.objects[scn.my_list2[scn.list_index2].GeoSelect]
            .evaluated_get(dgEv)
            .data.attributes
        ):
            node.append(str(obj.name))

        for counter in range(0, len(node)):
            items.append((str(node[counter]), str(node[counter]), ""))

    return items
    """
    scn = bpy.context.scene

    # Give option to have no material selected
    items = [
        ("None", "None", "")
    ]

    node = []

    if scn.my_list2[scn.list_index2].GeoSelect != "None":

        for obj in bpy.data.materials[scn.my_list2[scn.list_index2].GeoSelect].node_tree.nodes:
            node.append(str(obj.name))

        for counter in range(0, len(node)):
            items.append((str(node[counter]), str(node[counter]), ""))

    return items
    """


# Return name of selected object
def get_activeSceneObject():
    return bpy.context.scene.objects.active.name


# Fetch angles and LED values and update the output buffer
def get_angles_and_leds():
    global outputvalues
    global outputbytes
    global outputvalue
    global rawvalues

    scn = bpy.context.scene  # scene elements
    mprops = scn.scn_prop

    rawvalues.clear()
    outputvalues.clear()

    while len(outputvalues) < len(scn.my_list):
        outputvalues.append(0)
        rawvalues.append(0)

    # Clear output buffer
    outputbytes.clear()

    # If one value isn't set up, return
    for value in range(0, len(scn.my_list)):
        if (
            scn.my_list[value].ParentSelect == "None"
            or scn.my_list[value].ChildSelect == "None"
        ):
            return

    # Fill motor values
    for value in range(0, len(scn.my_list)):

        #
        # if scn.list_index >= 0 and len(scn.my_list) > 0:
        # item = scn.my_list[scn.list_index]
        item2 = scn.my_list[value]
        if (
            scn.my_list[value].ParentSelect != "None"
            and scn.my_list[value].ChildSelect != "None"
        ):
            data = scn.objects[scn.my_list[value].ParentSelect].pose.bones[
                scn.my_list[value].ChildSelect
            ]
            pos, rot, scale = data.matrix.decompose()
            rotsingle = 0.0

            # Magic matrix operations to get to local space of joint we're interested in
            rotsingle = get_bones_rotation(
                scn.my_list[value].ParentSelect,
                scn.my_list[value].ChildSelect,
                scn.my_list[value].AngleSelect,
            )
            rawvalues[value] = rotsingle

            if item2.RawValues:
                outputvalue = float(outputvalue)
                outputvalue = round(rotsingle + 0.0, 4)

                if item2.FlipDirection:
                    outputvalue = -outputvalue

                outputvalues[value] = outputvalue
                converted = outputvalue.to_bytes(2, "big")

                outputbytes.append((converted[0]))
                outputbytes.append((converted[1]))

            else:
                if scn.my_list[value].MotorType == "Stepper":
                    outputvalue = int(
                        num_to_range(
                            rotsingle,
                            scn.my_list[value].MinAngle,
                            scn.my_list[value].MaxAngle,
                            scn.my_list[value].MinMotorValue,
                            scn.my_list[value].MaxMotorValue,
                        )
                    )

                else:
                    # outputvalue = int((rotsingle - scn.my_list[value].OffsetAngle + scn.my_list[value].MaxAngle)*(1000.0/(scn.my_list[value].MaxAngle*2))+1000-scn.my_list[value].ServoOffsetAngle)
                    # if item2.FlipDirection:
                    #    outputvalue = abs(outputvalue-3000)
                    outputvalue = int(
                        num_to_range(
                            rotsingle,
                            scn.my_list[value].MinAngle,
                            scn.my_list[value].MaxAngle,
                            scn.my_list[value].MinMotorValue,
                            scn.my_list[value].MaxMotorValue,
                        )
                    )

                outputvalues[value] = outputvalue
                converted = outputvalue.to_bytes(2, "big")

                outputbytes.append((converted[0]))
                outputbytes.append((converted[1]))

            # outputvalues[value] = outputvalue

            # layout.label(str(outputvalues[value]))
            # layout.label(str(outputvalues[value]))

    # Add LED values
    if scn.scn_prop.SendLEDVals:
        for value in range(0, len(scn.my_list2)):

            # Neopixels
            if (
                scn.my_list2[value].LEDType == "Neopixel"
                or scn.my_list2[value].LEDType == "Dotstar"
            ):
                isRGBW = False

                if (
                    scn.my_list2[value].NeopixelOrder == "NEO_RGBW + NEO_KHZ800"
                    or scn.my_list2[value].NeopixelOrder == "NEO_GRBW + NEO_KHZ800"
                    or scn.my_list2[value].NeopixelOrder == "NEO_RGBW + NEO_KHZ400"
                    or scn.my_list2[value].NeopixelOrder == "NEO_GRBW + NEO_KHZ400"
                ):
                    isRGBW = True

                if scn.my_list2[value].LEDSingle:
                    # Send one color value to whole strip
                    if (
                        scn.my_list2[value].ArmatureSelect != "None"
                        and scn.my_list2[value].RedSelect1 != "None"
                        and scn.my_list2[value].GreenSelect1 != "None"
                        and scn.my_list2[value].BlueSelect1 != "None"
                    ):
                        RGBW = []
                        outputbytes.append(
                            int(scn.my_list2[value].LEDSingle).to_bytes(1, "big")[0]
                        )
                        r = math.floor(
                            interp(
                                get_bones_rotation(
                                    scn.my_list2[value].ArmatureSelect,
                                    scn.my_list2[value].RedSelect1,
                                    scn.my_list2[value].RedAngle,
                                ),
                                [0, 90],
                                [0, 255],
                            )
                        ).to_bytes(1, "big")
                        g = math.floor(
                            interp(
                                get_bones_rotation(
                                    scn.my_list2[value].ArmatureSelect,
                                    scn.my_list2[value].GreenSelect1,
                                    scn.my_list2[value].GreenAngle,
                                ),
                                [0, 90],
                                [0, 255],
                            )
                        ).to_bytes(1, "big")
                        b = math.floor(
                            interp(
                                get_bones_rotation(
                                    scn.my_list2[value].ArmatureSelect,
                                    scn.my_list2[value].BlueSelect1,
                                    scn.my_list2[value].BlueAngle,
                                ),
                                [0, 90],
                                [0, 255],
                            )
                        ).to_bytes(1, "big")

                        RGBW.append((r[0]))
                        RGBW.append((g[0]))
                        RGBW.append((b[0]))

                        if isRGBW and scn.my_list2[value].WhiteSelect1 != "None":
                            w = math.floor(
                                interp(
                                    get_bones_rotation(
                                        scn.my_list2[value].ArmatureSelect,
                                        scn.my_list2[value].WhiteSelect1,
                                        scn.my_list2[value].WhiteAngle,
                                    ),
                                    [0, 90],
                                    [0, 255],
                                )
                            ).to_bytes(1, "big")

                            RGBW.append((w[0]))

                        for i in range(0, len(RGBW)):
                            # outputbytes.append(chr(RGBW[value]))
                            outputbytes.append((RGBW[i]))

                # Send individual LED values using geometry nodes graph
                else:
                    if (
                        scn.my_list2[value].RedSelect2 != "None"
                        and scn.my_list2[value].GreenSelect2 != "None"
                        and scn.my_list2[value].BlueSelect2 != "None"
                    ):
                        dgEv = bpy.context.evaluated_depsgraph_get()
                        outputbytes.append(
                            int(scn.my_list2[value].LEDSingle).to_bytes(1, "big")[0]
                        )

                        for val in range(0, scn.my_list2[scn.list_index2].NumLeds):
                            RGBW = []
                            r = (
                                math.floor(
                                    bpy.data.objects[
                                        scn.my_list2[scn.list_index2].GeoSelect
                                    ]
                                    .evaluated_get(dgEv)
                                    .data.attributes[
                                        scn.my_list2[scn.list_index2].RedSelect2
                                    ]
                                    .data[val]
                                    .value
                                )
                            ).to_bytes(1, "big")
                            g = (
                                math.floor(
                                    bpy.data.objects[
                                        scn.my_list2[scn.list_index2].GeoSelect
                                    ]
                                    .evaluated_get(dgEv)
                                    .data.attributes[
                                        scn.my_list2[scn.list_index2].GreenSelect2
                                    ]
                                    .data[val]
                                    .value
                                )
                            ).to_bytes(1, "big")
                            b = (
                                math.floor(
                                    bpy.data.objects[
                                        scn.my_list2[scn.list_index2].GeoSelect
                                    ]
                                    .evaluated_get(dgEv)
                                    .data.attributes[
                                        scn.my_list2[scn.list_index2].BlueSelect2
                                    ]
                                    .data[val]
                                    .value
                                )
                            ).to_bytes(1, "big")

                            RGBW.append((r[0]))
                            RGBW.append((g[0]))
                            RGBW.append((b[0]))

                            if isRGBW and scn.my_list2[value].WhiteSelect2 != "None":
                                w = (
                                    math.floor(
                                        bpy.data.objects[
                                            scn.my_list2[scn.list_index2].GeoSelect
                                        ]
                                        .evaluated_get(dgEv)
                                        .data.attributes[
                                            scn.my_list2[scn.list_index2].WhiteSelect2
                                        ]
                                        .data[val]
                                        .value
                                    )
                                ).to_bytes(1, "big")
                                RGBW.append((w[0]))

                            for i in range(0, len(RGBW)):
                                outputbytes.append((RGBW[i]))

            # Single color
            elif scn.my_list2[value].LEDType == "Single Color":
                if (
                    scn.my_list2[value].ArmatureSelect != "None"
                    and scn.my_list2[value].LEDSelect != "None"
                    and scn.my_list2[value].LEDAngle != "None"
                ):
                    rgb_col = math.floor(
                        interp(
                            get_bones_rotation(
                                scn.my_list2[value].ArmatureSelect,
                                scn.my_list2[value].LEDSelect,
                                scn.my_list2[value].LEDAngle,
                            ),
                            [0, 90],
                            [0, 255],
                        )
                    ).to_bytes(1, "big")
                    outputbytes.append(rgb_col[0])


# Reads serial values from the Arduino
def GetInputValues():
    global inputvalues
    global inputservos
    global outputvalues
    global serialport

    if serialport.isOpen():
        while len(inputvalues) < len(outputvalues):
            inputvalues.append(0)
            inputvalues2.append(0)
            inputservos.append(0)

        while len(inputvalues) > len(outputvalues):
            inputvalues.remove(len(inputvalues - 1))
            inputvalues2.remove(len(inputvalues - 1))
            inputservos.remove(len(inputvalues - 1))

        for i in range(0, len(inputvalues)):
            serinput = serialport.readline().decode()
            bindex = 0

            if len(serinput) > 2:
                servonumber = str(serinput[1])
                servovalue = []

                endnum = "B"
                endval = "C"

                for index in range(2, len(serinput)):
                    if str(serinput[index]) == endnum:
                        bindex = index
                        break
                    else:
                        servonumber = servonumber + (str(serinput[index]))

                servonumber = int(servonumber)

                # print(str(servonumber))

                for index in range(bindex + 1, len(serinput)):
                    if serinput[index] == endval:
                        break
                    else:
                        servovalue.append(str(serinput[index]))

                s = "".join(servovalue)
                servovalue = int(s)
                # print(str(servonumber) + " " + str(servovalue))

                inputservos[i] = servonumber
                inputvalues[i] = servovalue

            else:
                return


def reset_sync():
    global last_sync

    bpy.data.scenes[0].sync_mode = last_sync


def getNeopixelTypeIndex(value):
    scn = bpy.context.scene

    neopixelType = scn.my_list2[value].NeopixelOrder

    if neopixelType == "RGB NEO_KHZ800":
        return 1

    elif neopixelType == "GRB NEO_KHZ800":
        return 2

    elif neopixelType == "RGB NEO_KHZ400":
        return 3

    elif neopixelType == "GRB NEO_KHZ400":
        return 4

    elif neopixelType == "RGBW NEO_KHZ800":
        return 5

    elif neopixelType == "GRBW NEO_KHZ800":
        return 6

    elif neopixelType == "RGBW NEO_KHZ400":
        return 7

    elif neopixelType == "GRBW NEO_KHZ400":
        return 8

    else:
        return 0


def isNeopixelRGBW(value):
    scn = bpy.context.scene

    neopixelType = scn.my_list2[value].NeopixelOrder

    if neopixelType == "RGB NEO_KHZ800":
        return 0

    elif neopixelType == "GRB NEO_KHZ800":
        return 0

    elif neopixelType == "RGB NEO_KHZ400":
        return 0

    elif neopixelType == "GRB NEO_KHZ400":
        return 0

    elif neopixelType == "RGBW NEO_KHZ800":
        return 1

    elif neopixelType == "GRBW NEO_KHZ800":
        return 1

    elif neopixelType == "RGBW NEO_KHZ400":
        return 1

    elif neopixelType == "GRBW NEO_KHZ400":
        return 1

    else:
        return 0


def get_config_size():
    global configSize

    scn = bpy.context.scene

    configSize = 0

    for item in range(0, len(scn.my_list)):
        mType = scn.my_list[item].MotorType

        if mType == "Servo" or mType == "PWM" or mType == "ON/OFF":
            configSize = configSize + 2
        elif mType == "Stepper":
            configSize = configSize + 9
        elif mType == "Bi-directional PWM":
            configSize = configSize + 3
        elif mType == "Dynamixel" or mType == "LewanSoul Bus Servo":
            configSize = configSize + 4

    for item in range(0, len(scn.my_list2)):
        ledType = scn.my_list2[item].LEDType

        if ledType == "Neopixel" or LEDType == "Single Color":
            configSize = configSize + 2
        elif ledType == "Dotstar":
            configSize = configSize + 3


###############
### CLASSES ###
###############
# UI LIST for Motors
class ListItem(bpy.types.PropertyGroup):
    """Group of properties representing an item in the list"""

    Name: prop.StringProperty(
        name="Name", description="A name for this item", default="Untitled"
    )

    ParentSelect: prop.EnumProperty(
        items=my_settings_callback_parent, name="Select Armature"
    )

    ChildSelect: prop.EnumProperty(items=my_settings_callback_child, name="Select Bone")

    JointNumber: prop.IntProperty(
        name="Joint/Servo Number",
        description="Servo Number",
        default=1,
        min=1,
        max=1000,
    )

    IDNumber: prop.IntProperty(
        name="ID",
        description="Number of bus servo/dynamixel",
        default=1,
        min=1,
        max=1000,
    )

    MotorType: prop.EnumProperty(
        items=[
            ("Servo", "Servo", ""),
            ("PWM", "PWM", ""),
            ("ON/OFF", "ON/OFF", ""),
            ("Bi-directional PWM", "Bi-directional PWM", ""),
            ("Stepper", "Stepper", ""),
            ("Dynamixel", "Dynamixel", ""),
            ("LewanSoul Bus Servo", "LewanSoul Bus Servo", ""),
        ],
        name="Select MotorType",
    )

    StepperMicrosteps: prop.EnumProperty(
        items=[
            ("1", "1", ""),
            ("2", "2", ""),
            ("4", "4", ""),
            ("8", "8", ""),
            ("16", "16", ""),
            ("32", "32", ""),
            ("64", "64", ""),
            ("128", "128", ""),
            ("256", "256", ""),
        ],
        name="Select microstep resolution",
        default="32",
    )

    Speed: prop.IntProperty(
        name="Stepper Speed", description="Specify the stepper max speed", default=10000
    )

    Acceleration: prop.IntProperty(
        name="Stepper Acceleration",
        description="Specify the stepper max Acceleration",
        default=1000,
    )

    ServoBaud: prop.EnumProperty(
        items=[
            ("9600", "9600", ""),
            ("19200", "19200", ""),
            ("34800", "34800", ""),
            ("57600", "57600", ""),
            ("115200", "115200", ""),
            ("1000000", "1000000", ""),
            ("2000000", "2000000", ""),
        ],
        name="Select Baud Rate for Servo",
        default="115200",
    )

    AngleSelect: prop.EnumProperty(
        items=[("X", "X", ""), ("Y", "Y", ""), ("Z", "Z", "")], name="Select Angle"
    )

    BoneAxisLock: prop.BoolProperty(
        name="Bone axis lock status",
        description="Is the bone axis locked?",
        default=False,
    )

    ShowValueDebug: prop.BoolProperty(
        name="Show Value", description="Display the angle values", default=True
    )

    PWMandDir: prop.BoolProperty(
        name="PWM and Direction",
        description="Check this for motor drivers like the L298, where a PWM pin connects to the EN pin and the DIR pin goes to IN1 and IN2, with one of the inputs going through a hex inverter. Leave unchecked for drivers like the DRV8871 or MX1508",
        default=False,
    )

    OffsetAngle: prop.FloatProperty(
        name="Bone Offset",
        description="Enter the inverse of the raw angle",
        default=0.0,
        min=-180.0,
        max=180.0,
    )

    ServoOffsetAngle: prop.FloatProperty(
        name="Servo Offset",
        description="Enter a final servo offset if servo is not centered",
        default=0.0,
        min=-2000.0,
        max=2000.0,
    )

    MinAngle: prop.FloatProperty(
        name="Min bone angle from 0",
        description="Enter the minimum value the bone travels",
        default=-90.0,
        min=-180.0,
        max=0.0,
    )

    MaxAngle: prop.FloatProperty(
        name="Max bone angle from 0",
        description="Enter the maximum value the bone travels",
        default=90.0,
        min=0.0,
        max=180.0,
    )

    MinMotorValue: prop.IntProperty(
        name="Min motor value from 0",
        description="Enter the minimum value the bone travels",
        default=1000,
        min=0,
        max=655354,
    )

    MaxMotorValue: prop.IntProperty(
        name="Max motor value from 0",
        description="Enter the maximum value the bone travels",
        default=2000,
        min=0,
        max=65534,
    )

    EnableLimits: prop.BoolProperty(
        name="Enable limits",
        description="Values outside of which the motor will never move to prevent damage",
    )

    LimitMin: prop.IntProperty(
        name="Limit motor minimum value",
        description="Enter the minimum value that the motor cannot exceed",
        default=2000,
        min=0,
        max=65534,
    )

    LimitMax: prop.IntProperty(
        name="Limit motor maximum value",
        description="Enter the maximum value that the motor cannot exceed",
        default=2000,
        min=0,
        max=65534,
    )

    FlipDirection: prop.BoolProperty(
        name="Flip Direction", description="Invert servo values if output is mirrored"
    )

    RawValues: prop.BoolProperty(
        name="Raw Values", description="Output raw angle values"
    )

    ServoPin: prop.IntProperty(
        name="Servo Pin",
        default=0,
        description="Physical pin servo signal is connected to",
    )

    PWMPin1: prop.IntProperty(
        name="PWM Pin", default=0, description="Physical pin PWM signal is connected to"
    )

    PWMPin2: prop.IntProperty(
        name="PWM pin", default=0, description="Physical pin PWM signal is connected to"
    )

    ONOFFPIN: prop.IntProperty(
        name="Pin", default=0, description="Physical pin ON/OFF signal is connected to"
    )

    TXPin: prop.IntProperty(
        name="TX Pin", default=0, description="Physical pin TX signal is connected to"
    )

    RXPin: prop.IntProperty(
        name="RX pin", default=0, description="Physical pin RX signal is connected to"
    )

    StepPin: prop.IntProperty(
        name="Step pin",
        default=0,
        description="Physical pin step signal is connected to",
    )
    DirPin: prop.IntProperty(
        name="Direction pin",
        default=0,
        description="Physical pin direction signal is connected to",
    )
    EnPin: prop.IntProperty(
        name="Enable pin",
        default=0,
        description="Physical pin enable signal is connected to",
    )

    OutputValue: prop.IntProperty(
        name="Output to Actuator",
        description="Number to be sent to servo/motor",
        min=-2000,
        max=2000,
        default=0,
    )


## UI LIST for LEDs
class ListItem2(bpy.types.PropertyGroup):
    Name: prop.StringProperty(
        name="Name", description="A name for this LED", default="Untitled"
    )
    LEDSingle: prop.BoolProperty(
        name="Single Color?",
        description="Check this if sending one value for an entire strip; this relies on bones in an armature representing each color channel",
    )
    RGBW: prop.BoolProperty(
        name="RGBW",
        description="Check this if LED has 4 channels, red, gree, blue, and white",
    )

    NumLeds: prop.IntProperty(
        name="Number of LEDs in strip",
        description="How many LEDS in the strip/ring?",
        default=12,
        min=1,
        max=144,
    )

    ArmatureSelect: prop.EnumProperty(
        items=my_settings_callback_parent, name="Select Armature"
    )

    RedSelect1: prop.EnumProperty(
        items=my_settings_callback_child, name="Select Bone for Red Channel"
    )

    GreenSelect1: prop.EnumProperty(
        items=my_settings_callback_child, name="Select Bone for Green Channel"
    )

    BlueSelect1: prop.EnumProperty(
        items=my_settings_callback_child, name="Select Bone for Blue Channel"
    )

    WhiteSelect1: prop.EnumProperty(
        items=my_settings_callback_child, name="Select Bone for White Channel"
    )

    LEDSelect: prop.EnumProperty(
        items=my_settings_callback_child, name="Select Bone for LED"
    )

    LEDType: prop.EnumProperty(
        items=[
            ("Neopixel", "Neopixel", ""),
            ("Dotstar", "Dotstar", ""),
            ("Single Color", "Single Color", ""),
        ],
        name="Select LED type",
        default="Single Color",
    )

    LEDAngle: prop.EnumProperty(
        items=[("X", "X", ""), ("Y", "Y", ""), ("Z", "Z", "")],
        name="Select angle representing led value",
    )

    RedAngle: prop.EnumProperty(
        items=[("X", "X", ""), ("Y", "Y", ""), ("Z", "Z", "")], name="Select Angle"
    )
    GreenAngle: prop.EnumProperty(
        items=[("X", "X", ""), ("Y", "Y", ""), ("Z", "Z", "")], name="Select Angle"
    )
    BlueAngle: prop.EnumProperty(
        items=[("X", "X", ""), ("Y", "Y", ""), ("Z", "Z", "")], name="Select Angle"
    )
    WhiteAngle: prop.EnumProperty(
        items=[("X", "X", ""), ("Y", "Y", ""), ("Z", "Z", "")], name="Select Angle"
    )
    # bpy.context.scene.scn_prop["AngleSelect"] = 0

    RedPin: prop.IntProperty(
        name="Red Pin",
        description="Physical pin red signal is connected to",
        default=0,
        min=0,
        max=1000,
    )

    GreenPin: prop.IntProperty(
        name="Green Pin",
        description="Physical pin green signal is connected to",
        default=0,
        min=0,
        max=1000,
    )

    BluePin: prop.IntProperty(
        name="Blue Pin",
        description="Physical pin blue signal is connected to",
        default=0,
        min=0,
        max=1000,
    )

    WhitePin: prop.IntProperty(
        name="White Pin",
        description="Physical pin white signal is connected to",
        default=0,
        min=0,
        max=1000,
    )

    DataPin: prop.IntProperty(
        name="Data Pin",
        description="Physical pin data signal is connected to",
        default=0,
        min=0,
        max=1000,
    )

    ClockPin: prop.IntProperty(
        name="Clock Pin",
        description="Physical pin clock signal is connected to",
        default=0,
        min=0,
        max=1000,
    )

    NeopixelOrder: prop.EnumProperty(
        items=[
            ("RGB", "RGB", ""),
            ("RBG", "RBG", ""),
            ("GRB", "GRB", ""),
            ("GBR", "GBR", ""),
            ("BRG", "BRG", ""),
            ("BGR", "BGR", ""),
        ],
        name="Select neopixel type",
        default="GRB",
    )

    GeoSelect: prop.EnumProperty(
        items=my_settings_callback_geonode, name="Select Geometry Nodes"
    )
    RedSelect2: prop.EnumProperty(
        items=my_settings_callback_node, name="Select Red Attribute"
    )
    GreenSelect2: prop.EnumProperty(
        items=my_settings_callback_node, name="Select Green Attribute"
    )
    BlueSelect2: prop.EnumProperty(
        items=my_settings_callback_node, name="Select Blue Attribute"
    )
    WhiteSelect2: prop.EnumProperty(
        items=my_settings_callback_node, name="Select White Attribute"
    )

    ShowValueDebug: prop.BoolProperty(
        name="Show Value", description="Display the angle values", default=False
    )

    ReverseStripOrder: prop.BoolProperty(
        name="Reverse Pixel Order",
        description="Flip the order of the LED strip if the animation appears backwards",
        default=False,
    )


# Scene properties
class SceneProperties(bpy.types.PropertyGroup):
    # print(serial_ports())
    # scn = bpy.context.scene
    # mprops = bpy.types.Scene.scn_prop

    SerialEnable: prop.BoolProperty(name="Enable Serial", description="Turn on serial?")

    SDCardEnabled: prop.BoolProperty(
        name="Enable SD Card",
        description="Allow microcontroller to read files from SD card",
    )

    SDCardInternal: prop.BoolProperty(
        name="Use internal SD Card",
        description="Microcontrollers like the Teensy 3.6 and 4.1 have an internal SD card that can be used. Otherwise, SPI will be used",
    )

    CSPin: prop.IntProperty(
        name="CS pin",
        description="Specify the physical pin connected to CS or SS",
        min=0,
        max=100,
        default=0,
    )

    ServoSpeedEnable: prop.BoolProperty(
        name="Servo Speed Enable",
        default=True,
        description="Adjust servo speed if you have a Dynamixel or other fancy servo. Note: This value is not written to file or to Arduino when exported",
    )

    SerialConnected: prop.BoolProperty(
        name="Serial Connected", description="True or False?"
    )

    UseCache: prop.BoolProperty(
        name="Use cache file",
        description="If active, will only send values through serial once per frame from a cache file. WARNING: disabling this can cause significant slowdown on scenes with lots of motors and LEDs",
    )

    OverwriteArduinoFile: prop.BoolProperty(
        name="Overwrite Arduino file",
        default=True,
        description="Use the default template file on github and overwrite what's in the project folder. If unchecked, the file will not be overwritten and you can modify the file freely.",
    )

    WriteCacheHeader: prop.BoolProperty(
        name="Write cache header line",
        description="If selected, a line with the number of frames, frame rate, and line header length will be written at the beginning of the file",
    )

    LEDMultiSingle: prop.BoolProperty(
        name="Multi LED",
        description="Will specify file contains data for all individual LEDs (header will start with 'M' instead of 'S'",
    )

    ReceivingValues: prop.BoolProperty(
        name="Receiving Joint Angles?", description="True or False?", default=False
    )

    SendLEDVals: prop.BoolProperty(
        name="Send LED values?", description="True or False?", default=True
    )

    ShowInputs: prop.BoolProperty(
        name="Display incoming values", description="True or False?", default=False
    )

    UseSceneFrameRange: prop.BoolProperty(
        name="Use scene frame range", description="True or False?", default=True
    )

    PortSelect: prop.EnumProperty(
        items=serial_port_callback,
        name="Serial Port",
        description="Choose a serial port",
    )

    BaudRate: prop.EnumProperty(
        name="Baud Rate",
        items=[
            ("1200", "1200", ""),
            ("2400", "2400", ""),
            ("4800", "4800", ""),
            ("9600", "9600", ""),
            ("19200", "19200", ""),
            ("34800", "34800", ""),
            ("57600", "57600", ""),
            ("115200", "115200", ""),
            ("1000000", "1000000", ""),
            ("2000000", "2000000", ""),
        ],
        description="Choose a baud rate",
        default="115200",
    )

    BusServoSerialPort: prop.EnumProperty(
        items=[
            ("Serial1", "Serial1", ""),
            ("Serial2", "Serial2", ""),
            ("Serial3", "Serial3", ""),
            ("Serial4", "Serial4", ""),
            ("Serial5", "Serial5", ""),
            ("Serial6", "Serial6", ""),
            ("Serial7", "Serial7", ""),
        ],
        name="Select Serial Port for Bus link",
        default="Serial1",
    )

    DynamixelSerialPort: prop.EnumProperty(
        items=[
            ("Serial1", "Serial1", ""),
            ("Serial2", "Serial2", ""),
            ("Serial3", "Serial3", ""),
            ("Serial4", "Serial4", ""),
            ("Serial5", "Serial5", ""),
            ("Serial6", "Serial6", ""),
            ("Serial7", "Serial7", ""),
        ],
        name="Select Serial Port for Dynamixel link",
        default="Serial1",
    )

    DynamixelBaud: prop.EnumProperty(
        items=[
            ("9600", "9600", ""),
            ("19200", "19200", ""),
            ("34800", "34800", ""),
            ("57600", "57600", ""),
            ("115200", "115200", ""),
            ("1000000", "1000000", ""),
            ("2000000", "2000000", ""),
        ],
        name="Select Baud Rate for Servo",
        default="57600",
    )

    DynamixelProtocolVer: prop.EnumProperty(
        items=[("1.0", "1.0", ""), ("2.0", "2.0", "")],
        name="Select Dynamixel protocol version",
        default="1.0",
    )

    DynamixelDirPin: prop.IntProperty(
        name="Dynamixel Direction Pin",
        description="Select the direction pin to the dynamixel bus",
        min=0,
        max=1000,
        default=2,
    )

    BusServoSpeed: prop.IntProperty(
        name="Bus Servo Speed",
        description="Set max bus servo speed. 1023 is highest. Note: This value is not written to file or to Arduino when exported",
        min=0,
        max=1023,
        default=512,
    )

    DynamixelSpeed: prop.IntProperty(
        name="Dynamixel Speed",
        description="Set max Dynamixel speed. 1023 is highest. Note: This value is not written to file or to Arduino when exported",
        min=0,
        max=1023,
        default=512,
    )

    FileName: prop.StringProperty(
        name="File Name",
        description="Name the output text file without the extension. Keep length under 7 characters!",
        default="",
    )

    FileDirectory: prop.StringProperty(
        name="File Path",
        description="Choose where to output file",
        default="",
        maxlen=1024,
        subtype="DIR_PATH",
    )

    StartFrame: prop.IntProperty(
        name="Start Frame",
        description="Set the start frame for output",
        min=0,
        max=1000000,
        default=0,
    )

    EndFrame: prop.IntProperty(
        name="End Frame",
        description="Set the end frame for output",
        # min = scn.scn_prop.StartFrame,
        min=0,
        max=1000000,
        default=1000,
    )

    ConfigType: prop.EnumProperty(
        name="Config type",
        items=[
            ("EEPROM", "EEPROM", ""),
            ("SD Card", "SD Card", ""),
            (".h File", ".h File", ""),
        ],
        description="Choose where the config will be stored",
        default=".h File",
    )

    ConfigEEPROMSize: prop.IntProperty(
        name="Size of EEPROM",
        description="Specify the size of the EEPROM on your microcontroller",
        min=0,
        max=100000,
        default=1024,
    )

    FolderName: prop.StringProperty(
        name="Folder Name",
        description="Name the project folder. Make sure to separate words with underscores, not spaces.",
    )


# Create list of servos
class LIST_UL_Items(bpy.types.UIList):
    def draw_item(
        self, context, layout, data, item, icon, active_data, active_propname, index
    ):
        split = layout.split(factor=0.3)
        custom_icon = "BONE_DATA"
        msg = "Servo " + str(bpy.context.scene.my_list[index].JointNumber)
        split.label(text=msg)
        # split.prop(item, "JointNumber", text = "", emboss = False, translate = False)
        split.prop(
            item, "Name", text="", emboss=False, translate=False, icon="BONE_DATA"
        )

    def invoke(self, context, event):
        pass


# ui list item actions
class Uilist_actions(bpy.types.Operator):
    bl_idname = "my_list.list_action"
    bl_label = "Values to send"

    action: bpy.props.EnumProperty(
        items=(
            ("UP", "Up", ""),
            ("DOWN", "Down", ""),
            ("REMOVE", "Remove", ""),
            ("ADD", "Add", ""),
        )
    )

    def invoke(self, context, event):

        scn = context.scene
        idx = scn.list_index

        try:
            item = scn.my_list[idx]
        except IndexError:
            pass

        else:
            if self.action == "DOWN" and idx < len(scn.my_list) - 1:
                item_next = scn.my_list[idx + 1].name
                self.description = "TESTING..."
                scn.my_list.move(idx, idx + 1)
                scn.list_index += 1
                info = "Item %d selected" % (scn.list_index + 1)
                self.report({"INFO"}, info)

            elif self.action == "UP" and idx >= 1:
                item_prev = scn.my_list[idx - 1].name
                scn.my_list.move(idx, idx - 1)
                scn.list_index -= 1
                info = "Item %d selected" % (scn.list_index + 1)
                self.report({"INFO"}, info)

            elif self.action == "REMOVE":
                info = "Item %s removed from list" % (scn.my_list[scn.list_index].name)
                scn.list_index -= 1
                self.report({"INFO"}, info)
                scn.my_list.remove(idx)

        if self.action == "ADD":
            item = scn.my_list.add()
            item.id = len(scn.my_list)
            item.name = "Servo"  # assign name of selected object
            scn.list_index = len(scn.my_list) - 1
            info = "%s added to list" % (item.name)
            self.report({"INFO"}, info)

            scn.my_list[scn.list_index].JointNumber = len(scn.my_list)

        return {"FINISHED"}


# Get file name and path
class SelectCacheFile(bpy.types.Operator, ImportHelper):
    bl_idname = "tl.select_cache_file"
    bl_label = "Select Cache File"

    files: CollectionProperty(type=bpy.types.PropertyGroup)  # Stores properties

    def execute(self, context):
        global cachefilepath
        """Do something with the selected file(s)."""

        dirname = os.path.dirname(self.filepath)
        for f in self.files:
            cachefilepath = os.path.join(dirname, f.name)
            print(cachefilepath)  # get filepath properties from collection pointer

        return {"FINISHED"}


# Create list of LEDs
class LIST_UL_Items2(bpy.types.UIList):
    def draw_item(
        self, context, layout, data, item, icon, active_data, active_propname, index
    ):
        split = layout.split(factor=0.3)
        custom_icon = "LIGHT_SUN"
        msg = "LED "
        split.label(text=msg)
        # split.prop(item, "JointNumber", text = "", emboss = False, translate = False)
        split.prop(
            item, "Name", text="", emboss=False, translate=False, icon="LIGHT_SUN"
        )

    def invoke(self, context, event):
        pass


# ui list item actions
class Uilist_actions2(bpy.types.Operator):
    bl_idname = "my_list2.list_action2"
    bl_label = "List Action 2"

    action: bpy.props.EnumProperty(
        items=(
            ("UP", "Up", ""),
            ("DOWN", "Down", ""),
            ("REMOVE", "Remove", ""),
            ("ADD", "Add", ""),
        )
    )

    def invoke(self, context, event):

        scn = context.scene
        idx = scn.list_index2

        try:
            item = scn.my_list2[idx]
        except IndexError:
            pass

        else:
            if self.action == "DOWN" and idx < len(scn.my_list2) - 1:
                item_next = scn.my_list2[idx + 1].name
                scn.my_list2.move(idx, idx + 1)
                scn.list_index2 += 1
                info = "Item %d selected" % (scn.list_index2 + 1)
                self.report({"INFO"}, info)

            elif self.action == "UP" and idx >= 1:
                item_prev = scn.my_list2[idx - 1].name
                scn.my_list2.move(idx, idx - 1)
                scn.list_index2 -= 1
                info = "Item %d selected" % (scn.list_index2 + 1)
                self.report({"INFO"}, info)

            elif self.action == "REMOVE":
                info = "Item %s removed from list" % (scn.my_list[scn.list_index2].name)
                scn.list_index2 -= 1
                self.report({"INFO"}, info)
                scn.my_list2.remove(idx)

        if self.action == "ADD":
            item = scn.my_list2.add()
            item.id = len(scn.my_list2)
            item.name = "Material"  # assign name of selected object
            scn.list_index2 = len(scn.my_list2) - 1
            info = "%s added to list" % (item.name)
            self.report({"INFO"}, info)

        return {"FINISHED"}


# Serial connection button
class ConnectSerial(bpy.types.Operator):
    bl_idname = "tl.connect_serial"
    bl_label = "Connect"
    bl_description = "Connect to serial port"

    def execute(self, context):
        scn = bpy.context.scene

        comport = scn.scn_prop.PortSelect

        if comport != "None":
            global serialport
            # serialport = serial.Serial('COM3', 9600, timeout = 0)
            serialport = serial.Serial(
                scn.scn_prop.PortSelect, scn.scn_prop.BaudRate, timeout=0.01
            )
            print(serialport.isOpen())
            print("Connected to " + comport + " at " + scn.scn_prop.BaudRate + " baud")
            scn.scn_prop.SerialConnected = True

        return {"FINISHED"}


# Disconnects from current serial port
class DisconnectSerial(bpy.types.Operator):
    bl_idname = "tl.disconnect_serial"
    bl_label = "Disconnect"
    bl_description = "Disconnect from serial port"

    def execute(self, context):
        scn = context.scene
        global serialport

        try:
            serialport.close()
            update_serial_ports()
        except NameError:
            0
        else:
            0

        scn.scn_prop.SerialConnected = False

        return {"FINISHED"}


# Toggles the armature bone axis visibility
class ToggleArmatureAxisVisibility(bpy.types.Operator):
    bl_idname = "tl.toggle_bone_axes"
    bl_label = "Toggle bone axes"
    bl_description = "Turn the visibility of bone axes on or off"

    def execute(self, context):
        scn = bpy.context.scene

        global currentArmature

        bpy.data.objects[currentArmature].data.show_axes = not bpy.data.objects[
            currentArmature
        ].data.show_axes

        return {"FINISHED"}


# Locks all axes except for chosen one
class LockOtherBoneAxes(bpy.types.Operator):
    bl_idname = "tl.lock_bone_axes"
    bl_label = "Lock bone axes"
    bl_description = "Lock all axes except for the selected one"

    def execute(self, context):
        global currentArmature, currentBone, currentAxis, currentListValue

        bpy.context.scene.my_list[currentListValue].BoneAxisLock = True

        if currentAxis == "X":
            bpy.data.objects[currentArmature].pose.bones[currentBone].lock_ik_y = True
            bpy.data.objects[currentArmature].pose.bones[currentBone].lock_ik_z = True

            bpy.data.objects[currentArmature].pose.bones[currentBone].lock_rotation[
                1
            ] = True
            bpy.data.objects[currentArmature].pose.bones[currentBone].lock_rotation[
                2
            ] = True

        elif currentAxis == "Y":
            bpy.data.objects[currentArmature].pose.bones[currentBone].lock_ik_x = True
            bpy.data.objects[currentArmature].pose.bones[currentBone].lock_ik_z = True

            bpy.data.objects[currentArmature].pose.bones[currentBone].lock_rotation[
                0
            ] = True
            bpy.data.objects[currentArmature].pose.bones[currentBone].lock_rotation[
                2
            ] = True

        elif currentAxis == "Z":
            bpy.data.objects[currentArmature].pose.bones[currentBone].lock_ik_x = True
            bpy.data.objects[currentArmature].pose.bones[currentBone].lock_ik_y = True

            bpy.data.objects[currentArmature].pose.bones[currentBone].lock_rotation[
                0
            ] = True
            bpy.data.objects[currentArmature].pose.bones[currentBone].lock_rotation[
                1
            ] = True

        return {"FINISHED"}


# Unlocks all axes
class UnlockBoneAxes(bpy.types.Operator):
    bl_idname = "tl.unlock_bone_axes"
    bl_label = "Unlock bone axes"
    bl_description = "Unlock all axes except for the selected one"

    def execute(self, context):
        global currentArmature, currentBone, currentAxis, currentListValue

        bpy.context.scene.my_list[currentListValue].BoneAxisLock = False

        bpy.data.objects[currentArmature].pose.bones[currentBone].lock_rotation[
            0
        ] = False
        bpy.data.objects[currentArmature].pose.bones[currentBone].lock_rotation[
            1
        ] = False
        bpy.data.objects[currentArmature].pose.bones[currentBone].lock_rotation[
            2
        ] = False

        bpy.data.objects[currentArmature].pose.bones[currentBone].lock_ik_x = False
        bpy.data.objects[currentArmature].pose.bones[currentBone].lock_ik_y = False
        bpy.data.objects[currentArmature].pose.bones[currentBone].lock_ik_z = False

        return {"FINISHED"}


class updateConfigSize(bpy.types.Operator):
    bl_idname = "tl.update_config_size"
    bl_label = "Update config size for EEPROM"

    def execute(self, context):
        get_config_size()

        return {"FINISHED"}


# Send single packet and get response back from Arduino in the terminal
class SendSingle(bpy.types.Operator):
    bl_idname = "tl.send_single"
    bl_label = "Send one packet"

    def execute(self, context):
        # time.sleep(1)

        scn = bpy.context.scene

        comport = scn.scn_prop.PortSelect

        if comport != "None":
            global serialport
            global outputbytes
            # serialport = serial.Serial('COM3', 9600, timeout = 0)
            serialport = serial.Serial(
                scn.scn_prop.PortSelect, scn.scn_prop.BaudRate, timeout=0.001
            )
            print("Connected to " + comport + " at " + scn.scn_prop.BaudRate + " baud")
            scn.scn_prop.SerialConnected = True

            print("Sending message of length: " + str(len(outputbytes) + 2))

            """
            message = bytearray(outputbytes)
            speed = bytearray(math.floor(scn.scn_prop.ServoSpeed).to_bytes(2, 'big'))
            serialport.write('A'.encode() + speed + message + '.'.encode())
            """

            message = bytearray(outputbytes)
            speed = bytearray(math.floor(scn.scn_prop.BusServoSpeed).to_bytes(2, "big"))
            length = bytearray(len(outputbytes).to_bytes(2, "big"))

            serialport.write("A".encode() + length + speed + message + ".".encode())
            """
            for i in range(0, len(message)):
                serialport.write(message[i])
            serialport.write('.'.encode())
            """
            time.sleep(0.4)

            received = ""

            for i in range(550):
                received += serialport.read().decode()
                """
                #received.append(serialport.read())
                try:
                    print(serialport.read().decode())
                except:
                    x = 0
                    #print()
                """
            print(str(received))

            try:
                serialport.close()
                update_serial_ports()
            except NameError:
                0
            else:
                0

            scn.scn_prop.SerialConnected = False

            print("Done...")

        return {"FINISHED"}


# Sync with microcontroller; creates a .c file with the current configuration that will get loaded onto the microcontroller
# This is a mess right now, will hopefully clean this up soon by making template files this class will copy from and paste into here
class Sync(bpy.types.Operator):
    bl_idname = "tl.sync_conf"
    bl_label = "Generate"
    bl_description = "Generate code and .h config file in specified directory"

    LED_index_offset = 10

    def execute(self, context):
        global serialport, configSize, outputfile

        scn = context.scene  # scene elements

        # Folder name and directory
        folder_name = scn.scn_prop.FolderName
        new_directory = (
            bpy.path.abspath(scn.scn_prop.FileDirectory) + "/" + folder_name + "/"
        )
        filenamez = new_directory + folder_name + ".ino"

        # Create a directory and download the Arduino file from Github
        # Overwrite file
        if scn.scn_prop.OverwriteArduinoFile or (
            not scn.scn_prop.OverwriteArduinoFile and not os.path.exists(filenamez)
        ):
            # Download from Github
            url = "https://raw.githubusercontent.com/knee-koh/MarIOnette/main/Arduino%20Code/MarIOnette_Template_V1/MarIOnette_Template_V1.ino"
            res = requests.get(url)

            # Create new folder in selected destination
            if not os.path.exists(new_directory):
                os.mkdir(new_directory)

            # Copy downloaded code into new code
            file = open(filenamez, "w")
            file.write(res.text)
            file.close()

        if scn.scn_prop.ConfigType == ".h File":
            # Count each actuator type
            num_servos = 0
            num_pwm = 0
            num_steppers = 0
            num_dual_pwm = 0
            num_lewansoul = 0
            num_dynamixel = 0
            num_neopixel = 0
            num_dotstars = 0
            num_pwm_led = 0

            max_neopixels = 0

            for item in range(0, len(scn.my_list)):
                if scn.my_list[item].MotorType == "Servo":
                    num_servos += 1
                elif scn.my_list[item].MotorType == "PWM":
                    num_pwm += 1
                elif scn.my_list[item].MotorType == "Stepper":
                    num_steppers += 1
                elif scn.my_list[item].MotorType == "Dynamixel":
                    num_dynamixel += 1
                elif scn.my_list[item].MotorType == "LewanSoul Bus Servo":
                    num_lewansoul += 1

            for item in range(0, len(scn.my_list2)):
                if (
                    scn.my_list2[item].LEDType == "Neopixel"
                    or scn.my_list2[item].LEDType == "Dotstar"
                ):
                    if scn.my_list2[item].LEDType == "Neopixel":
                        num_neopixel += 1
                    elif scn.my_list2[item].LEDType == "Dotstar":
                        num_dotstars += 1
                    if scn.my_list2[item].NumLeds > max_neopixels:
                        max_neopixels = scn.my_list2[item].NumLeds
                elif scn.my_list2[item].LEDType == "Single Color":
                    num_pwm_led += 1

            fullpath = os.path.join(new_directory, "config.h")
            outputfile = open(fullpath, "wb")

            # START OF FILE
            outputfile.write(
                "// MarIOnette generated config file for motors and LEDs //\n".encode()
            )

            outputfile.write(
                ("#define BAUD_RATE_SERIAL " + str(scn.scn_prop.BaudRate)).encode()
            )
            outputfile.write(
                (
                    "\n\n" + "#define SD_ENABLE " + str(int(scn.scn_prop.SDCardEnabled))
                ).encode()
            )
            outputfile.write(
                (
                    "\n"
                    + "#define USE_SD_INTERNAL "
                    + str(int(scn.scn_prop.SDCardInternal))
                ).encode()
            )
            outputfile.write(
                ("\n" + "#define SD_CS_PIN " + str(scn.scn_prop.CSPin)).encode()
            )
            outputfile.write(
                ("\n\n" + "#define TOTAL_MOTORS " + str(len(scn.my_list))).encode()
            )
            outputfile.write(
                ("\n" + "#define TOTAL_SERVOS " + str(num_servos)).encode()
            )
            outputfile.write(
                ("\n" + "#define TOTAL_PWM_MOTORS " + str(num_pwm)).encode()
            )
            outputfile.write(
                ("\n" + "#define TOTAL_STEPPERS " + str(num_steppers)).encode()
            )
            outputfile.write(
                ("\n" + "#define TOTAL_BUS_SERVOS " + str(num_lewansoul)).encode()
            )
            if num_lewansoul > 0:
                outputfile.write(("\n" + "#define BUS_SERVO_BAUD 115200").encode())
            outputfile.write(
                ("\n" + "#define TOTAL_DYNAMIXELS " + str(num_dynamixel)).encode()
            )
            outputfile.write(
                ("\n\n" + "#define TOTAL_LEDS " + str(len(scn.my_list2))).encode()
            )
            outputfile.write(
                ("\n" + "#define TOTAL_PWM_LEDS " + str(num_pwm_led)).encode()
            )
            outputfile.write(
                ("\n" + "#define TOTAL_NEOPIXELS " + str(num_neopixel)).encode()
            )

            outputfile.write(
                (
                    "\n\n"
                    + "unsigned long motor_values["
                    + str(len(scn.my_list))
                    + "][10] = {"
                ).encode()
            )

            counter = 0
            for item in range(0, len(scn.my_list)):
                if item != 0:
                    outputfile.write(("                                     ").encode())

                if scn.my_list[item].MotorType == "Servo":
                    if scn.my_list[item].EnableLimits:
                        outputfile.write(
                            (
                                "{1, "
                                + str(scn.my_list[item].ServoPin)
                                + ", 0, 0, 0, 0, 0, 0, "
                                + str(scn.my_list[item].LimitMin)
                                + ", "
                                + str(scn.my_list[item].LimitMax)
                                + "}"
                            ).encode()
                        )
                    else:
                        outputfile.write(
                            (
                                "{1, "
                                + str(scn.my_list[item].ServoPin)
                                + ", 0, 0, 0, 0, 0, 0, 0, 0}"
                            ).encode()
                        )
                elif scn.my_list[item].MotorType == "PWM":
                    if scn.my_list[item].EnableLimits:
                        outputfile.write(
                            (
                                "{2, "
                                + str(scn.my_list[item].PWMPin1)
                                + ", 0, 0, 0, 0, 0, "
                                + str(scn.my_list[item].LimitMin)
                                + ", "
                                + str(scn.my_list[item].LimitMax)
                                + "}"
                            ).encode()
                        )
                    else:
                        outputfile.write(
                            (
                                "{2, "
                                + str(scn.my_list[item].PWMPin1)
                                + ", 0, 0, 0, 0, 0, 0, 0, 0}"
                            ).encode()
                        )
                elif scn.my_list[item].MotorType == "ON/OFF":
                    outputfile.write(
                        (
                            "{3, "
                            + str(scn.my_list[item].ONOFFPIN)
                            + ", 0, 0, 0, 0, 0, 0, 0, 0}"
                        ).encode()
                    )
                elif scn.my_list[item].MotorType == "Bi-directional PWM":
                    if scn.my_list[item].PWMandDir:
                        if scn.my_list[item].EnableLimits:
                            outputfile.write(
                                (
                                    "{4, "
                                    + str(scn.my_list[item].PWMPin1)
                                    + ", "
                                    + str(scn.my_list[item].DirPin)
                                    + ", 1, 0, 0, 0, 0, "
                                    + str(scn.my_list[item].LimitMin)
                                    + ", "
                                    + str(scn.my_list[item].LimitMax)
                                    + "}"
                                ).encode()
                            )
                        else:
                            outputfile.write(
                                (
                                    "{4, "
                                    + str(scn.my_list[item].PWMPin1)
                                    + ", "
                                    + str(scn.my_list[item].DirPin)
                                    + ", 1, 0, 0, 0, 0, 0, 0}"
                                ).encode()
                            )
                    else:
                        if scn.my_list[item].EnableLimits:
                            outputfile.write(
                                (
                                    "{4, "
                                    + str(scn.my_list[item].PWMPin1)
                                    + ", "
                                    + str(scn.my_list[item].PWMPin2)
                                    + ", 0, 0, 0, 0, 0, "
                                    + str(scn.my_list[item].LimitMin)
                                    + ", "
                                    + str(scn.my_list[item].LimitMax)
                                    + "}"
                                ).encode()
                            )
                        else:
                            outputfile.write(
                                (
                                    "{4, "
                                    + str(scn.my_list[item].PWMPin1)
                                    + ", "
                                    + str(scn.my_list[item].PWMPin2)
                                    + ", 0, 0, 0, 0, 0, 0, 0}"
                                ).encode()
                            )
                elif scn.my_list[item].MotorType == "Stepper":
                    if scn.my_list[item].EnableLimits:
                        outputfile.write(
                            (
                                "{5, "
                                + str(scn.my_list[item].StepPin)
                                + ", "
                                + str(scn.my_list[item].DirPin)
                                + ", "
                                + str(scn.my_list[item].StepperMicrosteps)
                                + ", "
                                + str(scn.my_list[item].Speed)
                                + ", "
                                + str(scn.my_list[item].Acceleration)
                                + ", 0, 0, "
                                + str(scn.my_list[item].LimitMin)
                                + ", "
                                + str(scn.my_list[item].LimitMax)
                                + "}"
                            ).encode()
                        )
                    else:
                        outputfile.write(
                            (
                                "{5, "
                                + str(scn.my_list[item].StepPin)
                                + ", "
                                + str(scn.my_list[item].DirPin)
                                + ", "
                                + str(scn.my_list[item].StepperMicrosteps)
                                + ", "
                                + str(scn.my_list[item].Speed)
                                + ", "
                                + str(scn.my_list[item].Acceleration)
                                + ", 0, 0, 0, 0}"
                            ).encode()
                        )
                elif scn.my_list[item].MotorType == "LewanSoul Bus Servo":
                    if scn.my_list[item].EnableLimits:
                        outputfile.write(
                            (
                                "{7, "
                                + str(scn.my_list[item].IDNumber)
                                + ", 0, 0, 0, 0, 0, 0, "
                                + str(scn.my_list[item].LimitMin)
                                + ", "
                                + str(scn.my_list[item].LimitMax)
                                + "}"
                            ).encode()
                        )
                    else:
                        outputfile.write(
                            (
                                "{7, "
                                + str(scn.my_list[item].IDNumber)
                                + ", 0, 0, 0, 0, 0, 0, 0, 0}"
                            ).encode()
                        )
                elif scn.my_list[item].MotorType == "Dynamixel":
                    if scn.my_list[item].EnableLimits:
                        outputfile.write(
                            (
                                "{8, "
                                + str(scn.my_list[item].IDNumber)
                                + ", 0, 0, 0, 0, 0, 0, "
                                + str(scn.my_list[item].LimitMin)
                                + ", "
                                + str(scn.my_list[item].LimitMax)
                                + "}"
                            ).encode()
                        )
                    else:
                        outputfile.write(
                            (
                                "{8, "
                                + str(scn.my_list[item].IDNumber)
                                + ", 0, 0, 0, 0, 0, 0, 0, 0}"
                            ).encode()
                        )
                if item < len(scn.my_list) - 1:
                    outputfile.write((",\n").encode())

                counter += 1

            outputfile.write(("};" + "\n" + "\n").encode())

            if len(scn.my_list2) > 0:
                outputfile.write(
                    (
                        "unsigned long led_values["
                        + str(len(scn.my_list2))
                        + "][5] = {"
                    ).encode()
                )

                counter = 0
                for item in range(0, len(scn.my_list2)):
                    if item != 0:
                        outputfile.write(
                            ("                                  ").encode()
                        )
                    if scn.my_list2[item].LEDType == "Neopixel":
                        neopixelOrderNum = getNeopixelTypeIndex(item)
                        outputfile.write(
                            (
                                "{10, "
                                + str(scn.my_list2[item].DataPin)
                                + ", "
                                + str(scn.my_list2[item].NumLeds)
                                + ", "
                                + str(scn.my_list2[item].NumLeds)
                                + ", "
                                + str(int(scn.my_list2[item].RGBW))
                                + "}"
                            ).encode()
                        )
                    elif scn.my_list2[item].LEDType == "Dotstar":
                        outputfile.write(
                            (
                                "{10, "
                                + str(scn.my_list2[item].DataPin)
                                + ", "
                                + str(scn.my_list2[item].ClockPin)
                                + ", "
                                + str(str(scn.my_list2[item].NumLeds))
                                + ", "
                                + str(int(scn.my_list2[item].RGBW))
                                + "}"
                            ).encode()
                        )
                    elif scn.my_list2[item].LEDType == "Single Color":
                        outputfile.write(
                            (
                                "{11, " + str(scn.my_list2[item].DataPin) + ", 0, 0, 0}"
                            ).encode()
                        )

                    if counter < len(scn.my_list2) - 1:
                        outputfile.write((",\n").encode())

                    counter += 1

                # outputfile.write(("};" + '\n' + '\n').encode())

                outputfile.write(("};" + "\n").encode())
            else:
                outputfile.write(("unsigned long led_values[0][0];").encode())

            # Libraries and objects
            outputfile.write(('\n#include "Arduino.h"\n').encode())
            outputfile.write(
                (
                    "\n// For AVR - based boards without a dedicated Serial buffer, we need to add a small delay between Serial reads"
                ).encode()
            )
            outputfile.write(
                (
                    "\n// Teensy appears to use a default 12-bit resolution for Analog writes..."
                ).encode()
            )
            outputfile.write(
                (
                    "\n#if defined(__AVR__)\n #define IS_AVR 1\n #define SERIAL_DELAY 1\n #define ANALOG_MAX 255\n#else\n #define IS_AVR 0\n #define SERIAL_DELAY 5\n #define ANALOG_MAX 4095\n#endif\n"
                ).encode()
            )

            outputfile.write(
                (
                    "\n// MarIOnette serial\nunsigned int counter = 0;\nunsigned int howManyBytes = 0;\n\n// Expected packet\nconst int expectedMotorBytes = TOTAL_MOTORS * 2;\nconst int expectedSpeedBytes = 2;\n"
                ).encode()
            )

            # Servos: use PWMServo library if we have neopixels, otherwise use regular servo library
            if num_servos > 0 and num_neopixel == 0:
                outputfile.write(("\n// Servos\n#include <Servo.h>").encode())
                outputfile.write(
                    ("\nServo servos[" + str(num_servos) + "];\n").encode()
                )
            elif num_servos > 0 and num_neopixel > 0:
                outputfile.write(('\n// Servos\n#include "PWMServo.h"').encode())
                outputfile.write(
                    ("\nPWMServo servos[" + str(num_servos) + "];\n").encode()
                )

            # Stepper library
            if num_steppers > 0:
                outputfile.write(("\n// Steppers\n#include <AccelStepper.h>").encode())
                outputfile.write(
                    ("\nAccelStepper steppers[" + str(num_steppers) + "];\n").encode()
                )

            # LX16A
            if num_lewansoul > 0:
                outputfile.write(('\n// Bus Servos\n#include "LSServo.h"').encode())
                outputfile.write(("\nLSServo BusServos;\n").encode())

            outputfile.write(("unsigned int busServoSpeed;\n").encode())
            outputfile.write(("unsigned int readingPositions = 0;\n").encode())

            # Dynamixel
            if num_dynamixel > 0:
                outputfile.write(
                    ('\n// Dynamixels\n#include "Dynamixel2Arduino.h"').encode()
                )
                outputfile.write(
                    (
                        "\nDynamixel2Arduino dxl("
                        + str(scn.scn_prop.DynamixelSerialPort)
                        + ", "
                        + str(scn.scn_prop.DynamixelDirPin)
                        + ");\n"
                    ).encode()
                )
                outputfile.write(
                    (
                        "\n// This namespace is required to use Control table item names\nusing namespace ControlTableItem;"
                    ).encode()
                )
                outputfile.write(
                    (
                        "\nint oldDynamixelSpeed = "
                        + str(scn.scn_prop.DynamixelSpeed)
                        + ";"
                    ).encode()
                )

            # SD card
            if scn.scn_prop.SDCardEnabled:
                outputfile.write(("\n// SD Card\n#include <SPI.h>").encode())
                outputfile.write(("\n#include <SD.h>").encode())
                if scn.scn_prop.SDCardInternal:
                    outputfile.write(
                        ("\nconst int SDChipSelect = BUILTIN_SDCARD;").encode()
                    )
                else:
                    outputfile.write(("\nconst int SDChipSelect = SD_CS_PIN;").encode())
                outputfile.write(("\nFile animFile;\n").encode())
                outputfile.write(
                    (
                        "\n// SD Animation\nunsigned long currentFrame;\nunsigned int FPS;\nunsigned long totalFrames;\nunsigned int frameByteLength;\nunsigned long frameInterval;\nunsigned long animationTimer;\nchar filename[20];"
                    ).encode()
                )

            outputfile.write(("\nunsigned int playingAnimation;").encode())

            # Create an array of neopixels, number of leds is whichever strip has the most leds
            if num_neopixel + num_dotstars > 0:
                # outputfile.write(("\n\n#include <Adafruit_NeoPixel.h>").encode())
                # outputfile.write(("\n// LEDs\n" + "Adafruit_NeoPixel neopixels[" + str(num_neopixel) + "] = {{Adafruit_NeoPixel(").encode())
                outputfile.write(("\n\n// LEDs\n#include <FastLED.h>").encode())
                outputfile.write(
                    (
                        "\n"
                        + "CRGB neopixels["
                        + str(num_neopixel + num_dotstars)
                        + "]["
                        + str(max_neopixels)
                        + "];"
                    ).encode()
                )

                """
                counter = 0
                for i in range(0, len(scn.my_list2)):
                    if scn.my_list2[i].LEDType == "Neopixel":
                        outputfile.write((str(scn.my_list2[i].NumLeds) + ", " + str(scn.my_list2[i].DataPin) + ", " + str(scn.my_list2[i].NeopixelOrder) + ")}").encode())

                        if counter < num_neopixel-1:
                            outputfile.write((",\n").encode())
                        counter+=1


                outputfile.write(("};").encode())
                """

            # Setup function based on all included motors and leds
            outputfile.write(
                (
                    "\n\n// Auto-generated function from MarIOnette\nvoid setupAll(){"
                ).encode()
            )
            if scn.scn_prop.SDCardEnabled:
                outputfile.write(
                    (
                        '\n while(!SD.begin(SDChipSelect)){\n  Serial.println("No SD card found or incorrect wiring! Retrying in 3 seconds...");\n  delay(3000);\n }'
                    ).encode()
                )
            if len(scn.my_list) > 0:
                outputfile.write(
                    ("\n\n for(int i = 0; i < TOTAL_MOTORS; i++){").encode()
                )
                if num_servos > 0:
                    outputfile.write(
                        (
                            "\n  // Servos\n  if(motor_values[i][0] == 1){\n   servos[i].attach(motor_values[i][1]);\n  }"
                        ).encode()
                    )

                outputfile.write(
                    (
                        "\n\n  // PWM Pin\n  if(motor_values[i][0] == 2){\n   pinMode(motor_values[i][1], OUTPUT);\n  }"
                    ).encode()
                )
                outputfile.write(
                    (
                        "\n\n  // ON/OFF Pin\n  if(motor_values[i][0] == 3){\n   pinMode(motor_values[i][1], OUTPUT);\n  }"
                    ).encode()
                )
                outputfile.write(
                    (
                        "\n\n  // PWM Bi-Directional\n  if(motor_values[i][0] == 4){\n   pinMode(motor_values[i][1], OUTPUT);\n   pinMode(motor_values[i][2], OUTPUT);\n  }\n"
                    ).encode()
                )

                if num_lewansoul > 0:
                    outputfile.write(
                        (
                            "\n  // Bus Servos\n  if(motor_values[i][0] == 7){\n   "
                            + str(scn.scn_prop.BusServoSerialPort)
                            + ".begin(BUS_SERVO_BAUD);\n   BusServos.pSerial = &"
                            + str(scn.scn_prop.BusServoSerialPort)
                            + ';\n   Serial.println("Connected to bus servo on Serial1");\n  }'
                        ).encode()
                    )

                if num_dynamixel > 0:
                    outputfile.write(
                        (
                            "\n  // Dynamixels\n  dxl.begin("
                            + str(scn.scn_prop.DynamixelBaud)
                            + ");\n  dxl.setPortProtocolVersion("
                            + str(scn.scn_prop.DynamixelProtocolVer)
                            + ");\n"
                        ).encode()
                    )
                    outputfile.write(
                        ("\n  for(int i = 0; i < TOTAL_DYNAMIXELS+1; i++){").encode()
                    )
                    outputfile.write(("\n   dxl.torqueOff(i);").encode())
                    outputfile.write(
                        ("\n   dxl.setOperatingMode(i, OP_POSITION);").encode()
                    )
                    outputfile.write(("\n   dxl.torqueOn(i);").encode())
                    outputfile.write(
                        (
                            "\n   dxl.writeControlTableItem(PROFILE_VELOCITY, i, "
                            + str(scn.scn_prop.DynamixelSpeed)
                            + ");\n  }"
                        ).encode()
                    )

                if num_steppers > 0:
                    outputfile.write(
                        (
                            '\n\n  // Steppers\n  Serial.println("Starting stepper initialization...");\n'
                        ).encode()
                    )
                    outputfile.write(
                        ("  for(int i = 0; i < TOTAL_STEPPERS; i++){\n").encode()
                    )
                    outputfile.write(
                        ("   for(int j = 0; j < TOTAL_MOTORS; j++){\n").encode()
                    )
                    outputfile.write(("    if(motor_values[j][0] == 5){\n").encode())
                    outputfile.write(
                        (
                            "     steppers[i] = AccelStepper(steppers[i].DRIVER, motor_values[j][1], motor_values[j][2]);\n"
                        ).encode()
                    )
                    outputfile.write(
                        (
                            "     steppers[i].setMaxSpeed(motor_values[j][4]); // 100mm/s @ 80 steps/mm\n"
                        ).encode()
                    )
                    outputfile.write(
                        (
                            "     steppers[i].setAcceleration(motor_values[j][5]); // 2000mm/s^2\n"
                        ).encode()
                    )
                    outputfile.write(
                        (
                            "     steppers[i].setPinsInverted(false, false, true);\n"
                        ).encode()
                    )
                    outputfile.write(
                        (
                            "     steppers[i].enableOutputs();\n     i++;\n    }\n   }\n  }"
                        ).encode()
                    )

                outputfile.write(("\n }").encode())

                if len(scn.my_list2) > 0:
                    outputfile.write(
                        (
                            "\n\n // PWM LEDs\n for(int i = 0; i < TOTAL_LEDS; i++){"
                        ).encode()
                    )
                    outputfile.write(
                        (
                            "\n\n  if(led_values[i][0] == 11){\n   pinMode(led_values[i][2], OUTPUT);\n   analogWrite(led_values[i][2], 0);\n  }\n }"
                        ).encode()
                    )

                # Neopixels and others
                if num_neopixel + num_dotstars > 0:
                    numz = 0
                    for blah in range(0, len(scn.my_list2)):
                        if scn.my_list2[blah].LEDType == "Neopixel":
                            # outputfile.write(("\n\n FastLED.addLeds<NEOPIXEL, " + str(scn.my_list2[blah].DataPin) + ", " + str(scn.my_list2[blah].NeopixelOrder) +  ">(neopixels[" + str(numz) + "], " + str(max_neopixels) + ");\n").encode())
                            outputfile.write(
                                (
                                    "\n\n FastLED.addLeds<NEOPIXEL, "
                                    + str(scn.my_list2[blah].DataPin)
                                    + ">(neopixels["
                                    + str(numz)
                                    + "], "
                                    + str(max_neopixels)
                                    + ");\n"
                                ).encode()
                            )
                            numz += 1
                        elif scn.my_list2[blah].LEDType == "Dotstar":
                            outputfile.write(
                                (
                                    "\n\n FastLED.addLeds<DOTSTAR, "
                                    + str(scn.my_list2[blah].DataPin)
                                    + ", "
                                    + str(scn.my_list2[blah].ClockPin)
                                    + ", "
                                    + str(scn.my_list2[blah].NeopixelOrder)
                                    + ">(neopixels["
                                    + str(numz)
                                    + "], "
                                    + str(max_neopixels)
                                    + ");\n"
                                ).encode()
                            )
                            numz += 1

                    outputfile.write(
                        ("\n FastLED.clear();\n FastLED.show();\n").encode()
                    )

            outputfile.write(("\n}").encode())

            # Update function based on all included motors and leds
            outputfile.write(
                (
                    "\n\nvoid updateMotorsAndLEDs(char frame_buffer[], int mode){"
                ).encode()
            )
            if len(scn.my_list) > 0:
                outputfile.write(
                    ("\n int servo_index = 0;\n int stepper_index = 0;").encode()
                )
                outputfile.write(
                    ("\n\n for(int i = 0; i < TOTAL_MOTORS; i++){").encode()
                )
                outputfile.write(
                    (
                        "\n  int offset = i*2;\n\n  // Speed bytes are sent in case we have bus servos...\n  if(mode == 0){\n   offset = i*2+expectedSpeedBytes;\n  }\n\n  unsigned int motor_value = word(frame_buffer[offset], frame_buffer[offset+1]);"
                    ).encode()
                )

                outputfile.write(
                    (
                        "\n\n  // Min and Max values\n  if(motor_value > motor_values[i][9] && motor_values[i][8] != 0 && motor_values[i][9] != 0){\n   motor_value = motor_values[i][9];\n  }\n\n  else if(motor_value < motor_values[i][8] && motor_values[i][8] != 0 && motor_values[i][9] != 0){\n   motor_value = motor_values[i][8];\n  }"
                    ).encode()
                )

                if num_servos > 0 and num_neopixel == 0:
                    outputfile.write(
                        (
                            "\n\n  // Set servo position\n  if(motor_values[i][0] == 1){\n   servos[i].writeMicroseconds(motor_value);\n   servo_index++;\n  }"
                        ).encode()
                    )
                elif num_servos > 0 and num_neopixel > 0:
                    outputfile.write(
                        (
                            "\n\n  // Set servo position\n  if(motor_values[i][0] == 1){\n   servos[i].write(motor_value);\n   servo_index++;\n  }"
                        ).encode()
                    )

                outputfile.write(
                    (
                        "\n\n  // Set PWM value\n  if(motor_values[i][0] == 2){\n   analogWrite(motor_values[i][1], map(motor_value, 0, 4000, 0, ANALOG_MAX));\n  }"
                    ).encode()
                )
                outputfile.write(
                    (
                        "\n\n  // Set ON/OFF value\n  if(motor_values[i][0] == 3){\n   if(motor_value > 0){\n    digitalWrite(motor_values[i][1], HIGH);\n   }\n\n   else{\n    digitalWrite(motor_values[i][1], LOW);\n   }\n  }"
                    ).encode()
                )
                if num_steppers > 0:
                    outputfile.write(
                        (
                            "\n\n  // Set stepper position\n  if(motor_values[i][0] == 5){\n   steppers[stepper_index].moveTo(motor_value);\n   stepper_index++;   \n  }"
                        ).encode()
                    )

                if num_lewansoul > 0:
                    outputfile.write(
                        (
                            "\n\n  // Set Lewansoul servos\n  if(motor_values[i][0] == 7){\n   BusServos.SetPos(motor_values[i][1], motor_value, busServoSpeed); // Might need to remap the speed...\n  }"
                        ).encode()
                    )

                if num_dynamixel > 0:
                    outputfile.write(
                        (
                            "\n\n  // Set Dynamixel\n  if(motor_values[i][0] == 8){\n   if(busServoSpeed != oldDynamixelSpeed){"
                        ).encode()
                    )
                    outputfile.write(
                        (
                            "\n    for(int i = 0; i < TOTAL_DYNAMIXELS+1; i++){\n     dxl.writeControlTableItem(PROFILE_VELOCITY, i, busServoSpeed);\n    }\n    oldDynamixelSpeed = busServoSpeed;\n   }"
                        ).encode()
                    )
                    outputfile.write(
                        (
                            "\n\n   dxl.setGoalPosition(motor_values[i][1], motor_value);\n  }"
                        ).encode()
                    )

                outputfile.write(
                    (
                        "\n\n  // Set Bi-directional PWM\n  if(motor_values[i][0] == 4){"
                    ).encode()
                )
                outputfile.write(("\n   if(motor_values[i][3] == 1){").encode())
                outputfile.write(("\n    if(motor_value > ANALOG_MAX/2){").encode())
                outputfile.write(
                    (
                        "\n     analogWrite(motor_values[i][1], map(motor_value, ANALOG_MAX/2, ANALOG_MAX, 0, ANALOG_MAX));"
                    ).encode()
                )
                outputfile.write(
                    ("\n     digitalWrite(motor_values[i][2], HIGH);\n    }").encode()
                )
                outputfile.write(("\n\n    else{").encode())
                outputfile.write(
                    (
                        "\n     analogWrite(motor_values[i][1], map(motor_value, ANALOG_MAX/2, 0, 0, ANALOG_MAX));"
                    ).encode()
                )
                outputfile.write(
                    (
                        "\n     digitalWrite(motor_values[i][2], LOW);\n    }\n   }"
                    ).encode()
                )
                outputfile.write(("\n\n   else{").encode())
                outputfile.write(("\n    if(motor_value > ANALOG_MAX/2){").encode())
                outputfile.write(
                    (
                        "\n     analogWrite(motor_values[i][2], map(motor_value, ANALOG_MAX/2, ANALOG_MAX, 0, ANALOG_MAX));"
                    ).encode()
                )
                outputfile.write(
                    ("\n     digitalWrite(motor_values[i][1], LOW);\n    }").encode()
                )
                outputfile.write(("\n\n    else{").encode())
                outputfile.write(
                    (
                        "\n     analogWrite(motor_values[i][1], map(motor_value, ANALOG_MAX/2, 0, 0, ANALOG_MAX));"
                    ).encode()
                )
                outputfile.write(
                    (
                        "\n     digitalWrite(motor_values[i][2], LOW);\n    }\n   }\n  }"
                    ).encode()
                )
                outputfile.write(("\n }").encode())

            # Neopixel and led
            if len(scn.my_list2) > 0:
                outputfile.write(
                    (
                        "\n long offset = expectedMotorBytes;\n\n // Speed bytes are sent in case we have bus servos...\n if(mode == 0){\n  offset = expectedMotorBytes+expectedSpeedBytes;\n }\n\n unsigned int neopixel_index = 0;\n"
                    ).encode()
                )
                outputfile.write(("\n for(int i = 0; i < TOTAL_LEDS; i++){\n").encode())
                outputfile.write(
                    (
                        "\n  // PWM LED\n  if(led_values[i][0] == 11){\n   analogWrite(led_values[i][1], map(int(frame_buffer[offset]), 0, 254, 0, ANALOG_MAX));\n   offset++;\n  }"
                    ).encode()
                )

                if num_neopixel + num_dotstars > 0:
                    outputfile.write(("\n\n  // Neopixel single color").encode())
                    outputfile.write(
                        (
                            "\n  else if(led_values[i][0] == 10 && frame_buffer[offset] == 1){"
                        ).encode()
                    )
                    outputfile.write(("\n   offset++;").encode())
                    outputfile.write(
                        ("\n   int red = int(frame_buffer[offset]);").encode()
                    )
                    outputfile.write(
                        ("\n   int green = int(frame_buffer[offset + 1]);").encode()
                    )
                    outputfile.write(
                        ("\n   int blue = int(frame_buffer[offset + 2]);").encode()
                    )
                    outputfile.write(("\n   int white = 0;\n").encode())
                    outputfile.write(("\n   if(led_values[i][4] == 1){").encode())
                    outputfile.write(
                        ("\n    white = int(frame_buffer[offset + 3]);").encode()
                    )
                    outputfile.write(("\n    offset += 4;\n   }\n").encode())
                    outputfile.write(("\n   else{\n    offset +=3;\n   }\n").encode())
                    outputfile.write(("\n   if(led_values[i][4] == 1){").encode())
                    outputfile.write(
                        (
                            "\n    fill_solid(neopixels[neopixel_index], "
                            + str(max_neopixels)
                            + ", CRGB(red, green, blue));\n   }\n"
                        ).encode()
                    )
                    outputfile.write(
                        (
                            "\n   else{\n    fill_solid(neopixels[neopixel_index], "
                            + str(max_neopixels)
                            + ", CRGB(red, green, blue));\n   }\n"
                        ).encode()
                    )
                    outputfile.write(("\n   FastLED.show();").encode())
                    outputfile.write(("\n   neopixel_index++;").encode())
                    outputfile.write(("\n  }").encode())

                    outputfile.write(
                        (
                            "\n\n  // Individually addressible\n  else if(led_values[i][0] == 10 && frame_buffer[offset] == 0){\n   offset++;\n"
                        ).encode()
                    )
                    outputfile.write(
                        (
                            "\n   for(unsigned int j = 0; j < led_values[i][3]; j++){"
                        ).encode()
                    )
                    outputfile.write(
                        ("\n    int red = int(frame_buffer[offset]);").encode()
                    )
                    outputfile.write(
                        ("\n    int green = int(frame_buffer[offset + 1]);").encode()
                    )
                    outputfile.write(
                        ("\n    int blue = int(frame_buffer[offset + 2]);\n").encode()
                    )
                    outputfile.write(("\n    if(led_values[i][4] == 1){").encode())
                    outputfile.write(
                        ("\n     int white = int(frame_buffer[offset + 3]);").encode()
                    )
                    outputfile.write(
                        (
                            "\n     neopixels[neopixel_index][j].setRGB(red, green, blue);"
                        ).encode()
                    )
                    outputfile.write(("\n     offset += 4;\n    }\n").encode())
                    outputfile.write(("\n    else{").encode())
                    outputfile.write(
                        (
                            "\n     neopixels[neopixel_index][j].setRGB(red, green, blue);"
                        ).encode()
                    )
                    outputfile.write(("\n     offset += 3;\n    }\n   }\n").encode())
                    outputfile.write(("\n   FastLED.show();").encode())
                    outputfile.write(("\n   neopixel_index++;").encode())
                    outputfile.write(("\n  }").encode())

                outputfile.write(("\n }").encode())

            outputfile.write(("\n}").encode())

            # SD file open and reading
            outputfile.write(("\n\nvoid playAnimationFile(){").encode())
            if scn.scn_prop.SDCardEnabled:
                # Update animation from file
                outputfile.write(("\n if(playingAnimation){").encode())
                outputfile.write(
                    ("\n  if(micros() - animationTimer > frameInterval){").encode()
                )
                outputfile.write(("\n    if(currentFrame == totalFrames){").encode())
                outputfile.write(("\n     playingAnimation = 0;").encode())
                outputfile.write(("\n     animFile.close();").encode())
                outputfile.write(('\n     Serial.println("Animation done!");').encode())
                outputfile.write(("\n     return;\n   }\n").encode())
                outputfile.write(("\n    char frame_buffer[frameByteLength];").encode())
                outputfile.write(
                    (
                        "\n    for(unsigned int i = 0; i < frameByteLength; i++){"
                    ).encode()
                )
                outputfile.write(
                    ("\n     frame_buffer[i] = animFile.read();\n    }\n").encode()
                )
                outputfile.write(
                    ("\n    updateMotorsAndLEDs(frame_buffer, 1);").encode()
                )
                outputfile.write(("\n    currentFrame++;").encode())
                outputfile.write(
                    ("\n    animationTimer = micros();\n  }\n }\n}").encode()
                )

                # Open SD card file
                outputfile.write(("\n\nvoid readAnimationFile(){").encode())
                outputfile.write(
                    (
                        "\n // If file is found, parse the header and begin playback"
                    ).encode()
                )
                outputfile.write(("\n if(SD.exists(filename)){").encode())
                outputfile.write(('\n  Serial.print("File \'");').encode())
                outputfile.write(("\n  Serial.print(filename);").encode())
                outputfile.write(('\n  Serial.print("\' found! Size: ");').encode())
                outputfile.write(("\n  animFile = SD.open(filename);").encode())
                outputfile.write(("\n  Serial.println(animFile.size());\n").encode())
                outputfile.write(("\n  animFile.read(); // LED mode").encode())
                outputfile.write(("\n  totalFrames = animFile.parseInt();").encode())
                outputfile.write(("\n  FPS = animFile.parseInt();").encode())
                outputfile.write(
                    ("\n  frameByteLength = animFile.parseInt();\n").encode()
                )
                outputfile.write(("\n  animFile.read(); // newline").encode())
                outputfile.write(("\n  animFile.read(); // carriage return\n").encode())
                outputfile.write(
                    ("\n  frameInterval = 1000*1000/FPS; // In microseconds\n").encode()
                )
                outputfile.write(('\n  Serial.print("Total frames: ");').encode())
                outputfile.write(("\n  Serial.print(totalFrames);").encode())
                outputfile.write(('\n  Serial.print(" | FPS: ");').encode())
                outputfile.write(("\n  Serial.print(FPS);").encode())
                outputfile.write(('\n  Serial.print(" | Bytes per frame: ");').encode())
                outputfile.write(("\n  Serial.println(frameByteLength);\n").encode())
                outputfile.write(("\n  playingAnimation = 1;").encode())
                outputfile.write(("\n  currentFrame = 0;").encode())
                outputfile.write(("\n  animationTimer = micros();\n }\n").encode())
                outputfile.write(("\n // File not found").encode())
                outputfile.write(("\n else{").encode())
                outputfile.write(
                    ('\n  Serial.println("File not found on SD card!");').encode()
                )
                outputfile.write(("\n  playingAnimation = 0;\n }\n}").encode())

                # Helper Function for SD playback
                outputfile.write(("\n\nvoid SDHelper(int mode){").encode())
                outputfile.write(("\n if(mode == 0){").encode())
                outputfile.write(("\n  playingAnimation = 0;").encode())
                outputfile.write(("\n  animFile.close();\n }\n").encode())
                outputfile.write(("\n else{").encode())
                outputfile.write(("\n  int i = 0;\n").encode())
                outputfile.write(("\n  // Reset filename").encode())
                outputfile.write(("\n  for(int j = 0; j < 20; j++){").encode())
                outputfile.write(("\n   filename[j] = '\\0';\n  }\n").encode())
                outputfile.write(("\n  while(Serial.available()){").encode())
                outputfile.write(("\n   char c = Serial.read();\n").encode())
                outputfile.write(("\n   if(c != '\\n'){").encode())
                outputfile.write(("\n    filename[i] += c;").encode())
                outputfile.write(("\n    i++;\n   }\n").encode())
                outputfile.write(("\n   else{").encode())
                outputfile.write(("\n    filename[i] = '\\0';").encode())
                outputfile.write(("\n    Serial.flush();\n   }\n").encode())
                outputfile.write(("\n   if(IS_AVR){").encode())
                outputfile.write(("\n    delay(SERIAL_DELAY);\n   }\n").encode())
                outputfile.write(("\n   else{").encode())
                outputfile.write(
                    ("\n    delayMicroseconds(SERIAL_DELAY);\n   }\n  }\n").encode()
                )
                outputfile.write(('\n  Serial.print("Filename: ");').encode())
                outputfile.write(("\n  Serial.println(filename);\n").encode())
                outputfile.write(("\n  readAnimationFile();\n }\n}").encode())

            else:
                outputfile.write(("}").encode())
                outputfile.write(("\n\nvoid SDHelper(int mode){}").encode())
                outputfile.write(("\n\nvoid readAnimationFile(){}").encode())

            outputfile.write(("\n\nvoid updateSteppers(){").encode())

            if num_steppers > 0:
                outputfile.write(
                    (
                        "\n for(int i = 0; i < TOTAL_STEPPERS; i++){\n  steppers[i].run();\n }\n}"
                    ).encode()
                )

            else:
                outputfile.write(("}").encode())

            outputfile.close()

        return {"FINISHED"}


# Turn off serial connection if cable is disconnected
class updateSerialConnection(bpy.types.Operator):
    bl_idname = "tl.update_serial"
    bl_label = "Update Serial Connection"

    def execute(self, context):
        scn = context.scene
        print("Running in background")
        if scn.scn_prop.SerialConnected and (
            scn.scn_prop.PortSelect == "" or scn.scn_prop.PortSelect == "None"
        ):
            print("Serial cable disconnected, stopping serial port")
            global serialport
            serialport.close()
            scn.scn_prop.SerialConnected = False

        return {"FINISHED"}


# Updates the angles in the 3D view based on what we're receiveing over serial from the Arduino
class setAngles(bpy.types.Operator):
    bl_idname = "tl.set_angles"
    bl_label = "Set the angles"

    global inputvalues2
    global inputservos

    def execute(self, context):
        scn = bpy.context.scene
        for value in range(0, len(inputvalues2)):
            # Figure out if the current actuator matches the actuator in the list
            lindex = 0
            axisrot = 0

            for i in range(0, len(scn.my_list)):
                if inputservos[value] == scn.my_list[i].JointNumber:
                    lindex = i

            if scn.my_list[lindex].AngleSelect == "X":
                axisrot = 0

            elif scn.my_list[lindex].AngleSelect == "Y":
                axisrot = 1

            elif scn.my_list[lindex].AngleSelect == "Z":
                axisrot = 2

            print(lindex + 1)
            print(
                degrees(
                    scn.objects[scn.my_list[value].ParentSelect]
                    .pose.bones[scn.my_list[value].ChildSelect]
                    .rotation_euler[axisrot]
                )
            )
            print(" > ")
            print(inputvalues2[lindex])

            scn.objects[scn.my_list[value].ParentSelect].pose.bones[
                scn.my_list[value].ChildSelect
            ].rotation_mode = "XYZ"
            scn.objects[scn.my_list[value].ParentSelect].pose.bones[
                scn.my_list[value].ChildSelect
            ].rotation_euler[axisrot] = radians(inputvalues2[lindex])

        return {"FINISHED"}


# Modal (continuously running) function that updates the 3D view of the model
# Based on the angles received from the Arduino. Hit ESC to stop
# TODO: Add support for adding keyframes (SPACEBAR) and scrubbing through timeline
# With the arrow keys
class setAnglesModal(bpy.types.Operator):
    bl_idname = "object.set_angles_modal"
    bl_label = "Update 3D view live, press ESC to exit"

    def __init__(self):
        print("Start")

    def __del__(self):
        print("End")

        # get the context arguments

    def execute(self, context):
        print("Modal running...")
        scn = bpy.context.scene
        for value in range(0, len(inputvalues)):
            # Figure out if the current actuator matches the actuator in the list
            lindex = 0
            axisrot = 0

            for i in range(0, len(scn.my_list)):
                if inputservos[value] == scn.my_list[i].JointNumber:
                    lindex = i

            if scn.my_list[lindex].AngleSelect == "X":
                axisrot = 0

            elif scn.my_list[lindex].AngleSelect == "Y":
                axisrot = 1

            elif scn.my_list[lindex].AngleSelect == "Z":
                axisrot = 2

            scn.objects[scn.my_list[value].ParentSelect].pose.bones[
                scn.my_list[value].ChildSelect
            ].rotation_euler[axisrot] = radians(inputvalues2[lindex])
            # bpy.context.area.tag_redraw()
            # bpy.context.scene.update()
            # bpy.data.scenes[0].update()

        return {"FINISHED"}

    def modal(self, context, event):
        scn = bpy.context.scene

        # Exit
        if event.type == "ESC":
            print("ESC key pressed, cancelling")
            return {"CANCELLED"}

        # Add keyframe
        elif event.type == "SPACE" and event.value == "PRESS":
            print("Adding keyframe")
            # Select all the bones in our armature and set the keyframe
            for item in range(0, len(scn.my_list)):
                bpy.data.objects[scn.my_list[item].ParentSelect].data.bones[
                    scn.my_list[item].ChildSelect
                ].select = True

            bpy.ops.anim.keyframe_insert_menu(type="Rotation")

        elif event.type == "LEFT_ARROW" and event.value == "PRESS" and not event.shift:
            framenum = bpy.data.scenes["Scene"].frame_current

            if framenum == bpy.data.scenes["Scene"].frame_start:
                framenum = bpy.data.scenes["Scene"].frame_end

            else:
                framenum = framenum - 1

            scn.frame_set(framenum)

        elif event.type == "RIGHT_ARROW" and event.value == "PRESS" and not event.shift:
            framenum = bpy.data.scenes["Scene"].frame_current

            if framenum == bpy.data.scenes["Scene"].frame_end:
                framenum = bpy.data.scenes["Scene"].frame_start

            else:
                framenum = framenum + 1

            scn.frame_set(framenum)

        elif event.type == "LEFT_ARROW" and event.shift and event.value == "PRESS":
            framenum = bpy.data.scenes["Scene"].frame_current
            framenum = framenum - 5

            if framenum == bpy.data.scenes["Scene"].frame_start:
                framenum = bpy.data.scenes["Scene"].frame_end

            scn.frame_set(framenum)

        elif event.type == "RIGHT_ARROW" and event.shift and event.value == "PRESS":
            framenum = bpy.data.scenes["Scene"].frame_current
            framenum = framenum + 5

            if framenum == bpy.data.scenes["Scene"].frame_end:
                framenum = bpy.data.scenes["Scene"].frame_start

            scn.frame_set(framenum)

        elif (
            (event.type == "DEL" or event.type == "BACK_SPACE")
            and event.value == "PRESS"
            and not event.shift
        ):
            try:
                bpy.ops.anim.keyframe_delete_v3d()
            except:
                # catch the exception here and deal with it, or in this case do nothing
                pass
            else:
                x = 0

        elif (
            event.shift
            and event.value == "PRESS"
            and (event.type == "DEL" or event.type == "BACK_SPACE")
        ):
            for item in range(0, len(scn.my_list)):
                bpy.data.objects[scn.my_list[item].ParentSelect].data.bones[
                    scn.my_list[item].ChildSelect
                ].select = True

            for ob in bpy.data.objects:
                ob.animation_data_clear()
            # bpy.ops.outliner.animdata_operation(type = "CLEAR_ANIMDATA")

        if bpy.context.scene.scn_prop.ReceivingValues:
            self.execute(context)
            # bpy.context.area.tag_redraw()
            # bpy.context.scene.update()
            # bpy.data.scenes[0].update()

        return {"RUNNING_MODAL"}

    def invoke(self, context, event):
        print("Invoke modal")
        self.timer = context.window_manager.event_timer_add(
            0.005, window=context.window
        )
        context.window_manager.modal_handler_add(self)
        return {"RUNNING_MODAL"}


# TODO: Fix the slow performance
# Record the angles received from the Arduino for the duration of the timeline
# Modal, so it will keep running until final frame is reached or user hits ESC
class recordAnglesModal(bpy.types.Operator):
    bl_idname = "object.record_angles_modal"
    bl_label = "Record servo movements for duration of animation"

    def __init__(self):
        print("Start")

    def __del__(self):
        print("End")

    def execute(self, context):
        global framenumber
        print("Modal running...")
        scn = bpy.context.scene
        for value in range(0, len(inputvalues)):
            # Figure out if the current actuator matches the actuator in the list
            lindex = 0
            axisrot = 0

            for i in range(0, len(scn.my_list)):
                if inputservos[value] == scn.my_list[i].JointNumber:
                    lindex = i

            if scn.my_list[lindex].AngleSelect == "X":
                axisrot = 0

            elif scn.my_list[lindex].AngleSelect == "Y":
                axisrot = 1

            elif scn.my_list[lindex].AngleSelect == "Z":
                axisrot = 2

            scn.objects[scn.my_list[lindex].ParentSelect].pose.bones[
                scn.my_list[lindex].ChildSelect
            ].rotation_euler[axisrot] = radians(inputvalues2[value])
            # bpy.context.area.tag_redraw()

            # Select all the bones in our armature
            for item in range(0, len(scn.my_list)):
                bpy.data.objects[scn.my_list[item].ParentSelect].data.bones[
                    scn.my_list[item].ChildSelect
                ].select = True

                bpy.ops.anim.keyframe_insert_menu(type="Rotation")

        return {"FINISHED"}

    def modal(self, context, event):
        global framenumber
        scn = bpy.context.scene
        # bpy.context.area.tag_redraw()
        if event.type == "ESC":  # Confirm
            print("ESC key pressed, cancelling")
            return {"CANCELLED"}

        if framenumber > bpy.data.scenes["Scene"].frame_end:
            print("Done capturing frames")
            return {"CANCELLED"}

        if bpy.context.scene.scn_prop.ReceivingValues:
            self.execute(context)
            framenumber = framenumber + 1
            # scn.frame_set(framenumber)
            bpy.context.area.tag_redraw()

        return {"RUNNING_MODAL"}

    def invoke(self, context, event):
        global framenumber
        scn = bpy.context.scene
        print("Invoke modal")
        self.timer = context.window_manager.event_timer_add(
            0.5 / scn.render.fps, context.window
        )
        self.timer = context.window_manager.event_timer_add(0.01, context.window)
        context.window_manager.modal_handler_add(self)
        framenumber = bpy.data.scenes["Scene"].frame_start
        scn.frame_set(framenumber)
        # bpy.context.area.tag_redraw()
        return {"RUNNING_MODAL"}


# Saves the current animation to a .txt file to the hard drive
class OutputToTextFileCache(bpy.types.Operator):
    bl_idname = "tl.output_text_file_cache"
    bl_label = "Cache file"
    bl_description = "Save animation to the specified file name and location"

    global last_sync, done_writing

    if done_writing:
        bpy.data.scenes[0].sync_mode = last_sync
        done_writing = False

    def execute(self, context):
        scn = context.scene
        global writingfile
        global framenumber
        global outputfile
        global cacheheaderlength
        global outputbytes
        global headerLen
        framenumber = scn.scn_prop.StartFrame
        writingfile = True

        # Set to play every frame
        last_sync = bpy.context.scene.sync_mode
        bpy.data.scenes[0].sync_mode = "NONE"

        # Update file name
        FileFormat = ".txt"
        FinalFile = scn.scn_prop.FileName

        # Update path to save in
        fullpath = os.path.join(bpy.path.abspath(scn.scn_prop.FileDirectory), FinalFile)
        outputfile = open(fullpath, "wb")

        # Go to first frame
        scn.frame_set(scn.scn_prop.StartFrame)

        if scn.scn_prop.WriteCacheHeader:
            header = ""
            if scn.scn_prop.LEDMultiSingle:
                header = header + "M"
            else:
                header = header + "S"
            header = (
                header
                + str(scn.scn_prop.EndFrame - scn.scn_prop.StartFrame + 1)
                + "F"
                + str(scn.render.fps)
                + "B"
                + str(len(outputbytes))
            )
            headerLen = len(header)
            header = header + "Q" + "\n"
            outputfile.write(header.encode())

        return {"FINISHED"}


class OutputToTextFileCancel(bpy.types.Operator):
    bl_idname = "tl.cache_cancel"
    bl_label = "Cancel"
    bl_description = "Cancel file caching"

    def execute(self, context):
        scn = context.scene
        global writingfile
        global framenumber
        global outputfile
        global cacheheaderlength
        global last_sync

        # Set to play every frame
        # bpy.context.scene.sync_mode = last_sync
        bpy.data.scenes[0].sync_mode = last_sync

        writingfile = False

        outputfile.close()

        return {"FINISHED"}


# Sets a keyframe for all bones' current rotations at the current place in the timeline
class SetKeyframe(bpy.types.Operator):
    bl_idname = "tl.set_keyframez"
    bl_label = "Set Keyframe"

    def execute(self, context):
        scn = context.scene

        # Update the values and the 3D view
        for value in range(0, len(inputvalues)):
            # Figure out if the current actuator matches the actuator in the list
            lindex = 0
            axisrot = 0

            for i in range(0, len(scn.my_list)):
                if inputservos[value] == scn.my_list[i].JointNumber:
                    lindex = i

            if scn.my_list[lindex].AngleSelect == "X":
                axisrot = 0

            elif scn.my_list[lindex].AngleSelect == "Y":
                axisrot = 1

            elif scn.my_list[lindex].AngleSelect == "Z":
                axisrot = 2

            scn.objects[scn.my_list[lindex].ParentSelect].pose.bones[
                scn.my_list[lindex].ChildSelect
            ].rotation_euler[axisrot] = radians(inputvalues2[value])

        bpy.context.area.tag_redraw()

        # Select all the bones in our armature and set the keyframe
        for item in range(0, len(scn.my_list)):
            bpy.data.objects[scn.my_list[item].ParentSelect].data.bones[
                scn.my_list[item].ChildSelect
            ].select = True

        bpy.ops.anim.keyframe_insert_menu(type="Rotation")
        return {"FINISHED"}


class refreshSerialPorts(bpy.types.Operator):
    bl_idname = "tl.refresh_ports"
    bl_label = "Refresh list"
    bl_description = "Refresh list of serial ports"

    def execute(self, context):
        update_serial_ports()

        return {"FINISHED"}


class updateFrameSync(bpy.types.Operator):
    bl_idname = "tl.update_sync_mode"
    bl_label = "Sync update"

    global sync_mode

    def execute(self, context):
        bpy.data.scenes[0].sync_mode = last_sync
        return {"FINISHED"}


class playOnMicrocontroller(bpy.types.Operator):
    bl_idname = "tl.play_on_microcontroller"
    bl_label = "Play on microcontroller"

    global serialport

    def execute(self, context):
        scn = context.scene
        file_to_play = scn.scn_prop.FileName
        serialport.write("P".encode() + file_to_play.encode())
        return {"FINISHED"}


class stopMicrocontrollerPlayback(bpy.types.Operator):
    bl_idname = "tl.stop_playback"
    bl_label = "Stop playback"

    global serialport

    def execute(self, context):
        scn = context.scene
        serialport.write("S".encode())
        return {"FINISHED"}


### PANELS & UI ###


# Serial connection setup
class MPANEL_PT_MarIOnette_SerialPanel(bpy.types.Panel):
    # bl_idname="OBJECT_PT_ArduinoAnimation_panel"
    bl_label = "Serial Setup"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "MarIOnette"  # creates a new tab in the tool menu on the left of the blender 3d viewer. Press 'T' if tool window is hidden
    # bl_context="posemode"
    bl_options = {"DEFAULT_CLOSED"}

    def draw(self, context):
        global outputvalue, outputvalues, outputbytes, cachefilepath

        layout = self.layout  # layout elements
        scn = context.scene  # scene elements

        # layout.operator("tl.print_bone_rotation")
        # col = layout.column(align = False)
        col = layout.column()

        # Check box to toggle serial connection
        col = layout.column()
        # layout.prop(scn.scn_prop, "ServoSpeedEnable")
        # col.prop(scn.scn_prop, "ServoSpeed") if scn.scn_prop.ServoSpeedEnable else 0
        layout.prop(scn.scn_prop, "SerialEnable")
        # layout.prop(scn.scn_prop, "UseCache")
        # layout.operator("tl.select_cache_file") if scn.scn_prop.UseCache else 0
        # layout.label(text = str(cachefilepath)) if scn.scn_prop.UseCache else 0
        # layout.prop(scn.scn_prop, "ReceivingValues")
        # layout.prop(scn.scn_prop, "SendLEDVals")
        layout.operator("tl.refresh_ports") if scn.scn_prop.SerialEnable else 0

        # Drop Down menu with serial ports and baud rate selection
        layout.prop(scn.scn_prop, "PortSelect") if scn.scn_prop.SerialEnable else 0
        layout.prop(scn.scn_prop, "BaudRate") if scn.scn_prop.SerialEnable else 0

        # Connect and disconnect from serial
        layout.operator("tl.connect_serial") if scn.scn_prop.SerialEnable else 0
        layout.operator("tl.disconnect_serial") if scn.scn_prop.SerialEnable else 0

        # Send command once, receive input, and then disconnect
        layout.operator("tl.send_single") if scn.scn_prop.SerialEnable else 0

        # New row?
        col = layout.column()

        return


# Actuator setup panel with UI List element
class MPANEL_PT_MarIOnette_ActuatorPanel(bpy.types.Panel):
    bl_label = "Actuator Setup"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "MarIOnette"
    bl_options = {"DEFAULT_CLOSED"}

    def draw(self, context):
        global outputvalue
        global outputvalues, rawvalues, currentArmature, currentBone, currentAxis, currentListValue
        layout = self.layout  # layout elements
        scn = context.scene  # scene elements
        mprops = scn.scn_prop

        col = layout.column()

        rows = 2
        row = layout.row()
        row.template_list("LIST_UL_Items", "", scn, "my_list", scn, "list_index")

        col = row.column(align=True)

        col.operator("my_list.list_action", icon="ADD", text="").action = "ADD"
        col.operator("my_list.list_action", icon="REMOVE", text="").action = "REMOVE"
        col.separator()
        col.operator("my_list.list_action", icon="TRIA_UP", text="").action = "UP"
        col.operator("my_list.list_action", icon="TRIA_DOWN", text="").action = "DOWN"

        col = layout.column()

        # Only display data for active joint
        for value in range(0, len(scn.my_list)):
            # Only display selected joint data
            if scn.list_index >= 0 and len(scn.my_list) > 0 and value == scn.list_index:
                item = scn.my_list[scn.list_index]
                item2 = scn.my_list[value]

                col: layout.column()
                col.prop(item, "Name")
                # col.prop(item, "JointNumber")
                col.prop(item, "ParentSelect")

                if scn.my_list[value].ParentSelect != "None":
                    currentArmature = scn.my_list[value].ParentSelect
                    col.operator("tl.toggle_bone_axes")

                col.prop(item, "ChildSelect")
                col.prop(item, "AngleSelect")

                if (
                    scn.my_list[value].ParentSelect != "None"
                    and scn.my_list[value].ChildSelect != "None"
                    and not scn.my_list[value].BoneAxisLock
                ):
                    currentBone = scn.my_list[value].ChildSelect
                    currentAxis = scn.my_list[value].AngleSelect
                    currentListValue = value
                    col.operator("tl.lock_bone_axes")
                elif (
                    scn.my_list[value].ParentSelect != "None"
                    and scn.my_list[value].ChildSelect != "None"
                    and scn.my_list[value].BoneAxisLock
                ):
                    currentAxis = scn.my_list[value].AngleSelect
                    currentBone = scn.my_list[value].ChildSelect
                    currentListValue = value
                    col.operator("tl.unlock_bone_axes")

                if (
                    scn.my_list[value].ParentSelect != "None"
                    and scn.my_list[value].ChildSelect != "None"
                ):
                    col.prop(item, "MotorType")

                    if scn.my_list[value].MotorType == "Servo":
                        col.prop(item, "ServoPin")
                    elif scn.my_list[value].MotorType == "PWM":
                        col.prop(item, "PWMPin1")
                    elif scn.my_list[value].MotorType == "ON/OFF":
                        col.prop(item, "ONOFFPIN")
                    elif scn.my_list[value].MotorType == "Bi-directional PWM":
                        col.prop(item, "PWMandDir")
                        col.prop(item, "PWMPin1")
                        if scn.my_list[value].PWMandDir:
                            col.prop(item, "DirPin")
                        else:
                            col.prop(item, "PWMPin2")
                    elif (
                        scn.my_list[value].MotorType == "Dynamixel"
                        or scn.my_list[value].MotorType == "LewanSoul Bus Servo"
                    ):
                        col.prop(item, "IDNumber")
                        if scn.my_list[value].MotorType == "LewanSoul Bus Servo":
                            col.prop(scn.scn_prop, "BusServoSerialPort")
                            col.prop(scn.scn_prop, "BusServoSpeed")
                        elif scn.my_list[value].MotorType == "Dynamixel":
                            col.prop(scn.scn_prop, "DynamixelSerialPort")
                            col.prop(scn.scn_prop, "DynamixelBaud")
                            col.prop(scn.scn_prop, "BusServoSpeed")
                            col.prop(scn.scn_prop, "DynamixelProtocolVer")
                            col.prop(scn.scn_prop, "DynamixelDirPin")
                    elif scn.my_list[value].MotorType == "Stepper":
                        col.prop(item, "DirPin")
                        col.prop(item, "StepPin")
                        # col.prop(item, "EnPin")
                        col.prop(item, "StepperMicrosteps")
                        col.prop(item, "Speed")
                        col.prop(item, "Acceleration")

                    # col.prop(item, "RawValues")
                    col.prop(item, "MinAngle")
                    col.prop(item, "MaxAngle")
                    col.prop(item, "MinMotorValue")
                    col.prop(item, "MaxMotorValue")
                    col.prop(item, "EnableLimits")
                    if (
                        scn.my_list[value].EnableLimits
                        and scn.my_list[value].MotorType != "ON/OFF"
                    ):
                        col.prop(item, "LimitMin")
                        col.prop(item, "LimitMax")
                    col.prop(item, "ShowValueDebug")
                    row = layout.row()
                    # col.prop(item, "FlipDirection")

                    # Show value of angle for debugging
                    if (
                        scn.my_list[value].ParentSelect != "None"
                        and scn.my_list[value].ChildSelect != "None"
                        and scn.my_list[value].ShowValueDebug
                    ):
                        col = layout.column()
                        layout.label(
                            text="Raw angle value: " + str(round(rawvalues[value], 2))
                        )
                        # layout.label("Raw angle value x: " + str(round(degrees(rot.to_euler().x), 2)) + " y: " + str(round(degrees(rot.to_euler().y), 2)) + " z: " + str(round(degrees(rot.to_euler().z), 2)))
                        layout.label(text="Output value: " + str(outputvalues[value]))

                else:
                    layout.label(
                        text="Please select an Armature and Bone for the actuator!"
                    )

        return


# LED setup panel with UI List element
class MPANEL_PT_MarIOnette_LedPanel(bpy.types.Panel):
    bl_label = "RGB LED Setup"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "MarIOnette"
    bl_options = {"DEFAULT_CLOSED"}

    def draw(self, context):
        global outputvalue
        layout = self.layout  # layout elements
        scn = context.scene  # scene elements
        mprops = scn.scn_prop

        # UI customizable list
        rows = 2
        row = layout.row()
        row.template_list("LIST_UL_Items2", "", scn, "my_list2", scn, "list_index2")

        col = row.column(align=True)
        col.operator("my_list2.list_action2", icon="ADD", text="").action = "ADD"
        col.operator("my_list2.list_action2", icon="REMOVE", text="").action = "REMOVE"
        col.separator()
        col.operator("my_list2.list_action2", icon="TRIA_UP", text="").action = "UP"
        col.operator("my_list2.list_action2", icon="TRIA_DOWN", text="").action = "DOWN"

        col = layout.column()

        # Only display data for active LED, and then send everything through serial
        for value in range(0, len(scn.my_list2)):
            # Only display selected joint data
            if (
                scn.list_index2 >= 0
                and len(scn.my_list2) > 0
                and value == scn.list_index2
            ):
                item = scn.my_list2[scn.list_index2]
                item2 = scn.my_list2[value]

                col: layout.column()
                col.prop(item, "Name")
                col.prop(item, "LEDType")

                isRGBW = False

                if (
                    scn.my_list2[value].NeopixelOrder == "NEO_RGBW + NEO_KHZ800"
                    or scn.my_list2[value].NeopixelOrder == "NEO_GRBW + NEO_KHZ800"
                    or scn.my_list2[value].NeopixelOrder == "NEO_RGBW + NEO_KHZ400"
                    or scn.my_list2[value].NeopixelOrder == "NEO_GRBW + NEO_KHZ400"
                ):
                    isRGBW = True

                if (
                    scn.my_list2[value].LEDType == "Neopixel"
                    or scn.my_list2[value].LEDType == "Dotstar"
                ):
                    # col.prop(item, "RGBW")
                    col.prop(item, "DataPin")
                    if scn.my_list2[value].LEDType == "Dotstar":
                        col.prop(item, "ClockPin")
                    col.prop(item, "NumLeds")
                    col.prop(item, "NeopixelOrder")
                    col.prop(item, "LEDSingle")

                    if scn.my_list2[value].LEDSingle:
                        col.prop(item, "ArmatureSelect")
                        col.prop(item, "RedSelect1")
                        col.prop(item, "RedAngle")
                        col.prop(item, "GreenSelect1")
                        col.prop(item, "GreenAngle")
                        col.prop(item, "BlueSelect1")
                        col.prop(item, "BlueAngle")

                        if scn.my_list2[value].RGBW:
                            col.prop(item, "WhiteSelect1")
                            col.prop(item, "WhiteAngle")

                    else:
                        col.prop(item, "GeoSelect")
                        col.prop(item, "RedSelect2")
                        col.prop(item, "GreenSelect2")
                        col.prop(item, "BlueSelect2")

                        if scn.my_list2[value].RGBW:
                            col.prop(item, "WhiteSelect2")

                    col.prop(item, "ShowValueDebug")

                elif scn.my_list2[value].LEDType == "Single Color":
                    col.prop(item, "ArmatureSelect")
                    col.prop(item, "LEDSelect")
                    col.prop(item, "LEDAngle")
                    col.prop(item, "DataPin")
                    col.prop(item, "ShowValueDebug")

                # Show LED values
                # Neopixels
                if (
                    scn.my_list2[value].LEDType == "Neopixel"
                    or scn.my_list2[value].LEDType == "Dotstar"
                    and scn.my_list2[value].ShowValueDebug == True
                ):
                    # Single color for whole strip
                    if (
                        scn.my_list2[value].LEDSingle
                        and scn.my_list2[value].ArmatureSelect != "None"
                        and scn.my_list2[value].RedSelect1 != "None"
                        and scn.my_list2[value].GreenSelect1 != "None"
                        and scn.my_list2[value].BlueSelect1 != "None"
                    ):
                        redVal = math.floor(
                            interp(
                                get_bones_rotation(
                                    scn.my_list2[value].ArmatureSelect,
                                    scn.my_list2[value].RedSelect1,
                                    scn.my_list2[value].RedAngle,
                                ),
                                [0, 90],
                                [0, 255],
                            )
                        )
                        greenVal = math.floor(
                            interp(
                                get_bones_rotation(
                                    scn.my_list2[value].ArmatureSelect,
                                    scn.my_list2[value].GreenSelect1,
                                    scn.my_list2[value].GreenAngle,
                                ),
                                [0, 90],
                                [0, 255],
                            )
                        )
                        blueVal = math.floor(
                            interp(
                                get_bones_rotation(
                                    scn.my_list2[value].ArmatureSelect,
                                    scn.my_list2[value].BlueSelect1,
                                    scn.my_list2[value].BlueAngle,
                                ),
                                [0, 90],
                                [0, 255],
                            )
                        )

                        if isRGBW:
                            if scn.my_list2[value].WhiteSelect1 != "None":
                                whiteVal = math.floor(
                                    interp(
                                        get_bones_rotation(
                                            scn.my_list2[value].ArmatureSelect,
                                            scn.my_list2[value].WhiteSelect1,
                                            scn.my_list2[value].WhiteAngle,
                                        ),
                                        [0, 90],
                                        [0, 255],
                                    )
                                )
                                (
                                    layout.label(
                                        text="R: "
                                        + str(redVal)
                                        + " | G: "
                                        + str(greenVal)
                                        + " | B: "
                                        + str(blueVal)
                                        + " | W: "
                                        + str(whiteVal)
                                    )
                                    if scn.my_list2[value].ShowValueDebug
                                    else 0
                                )
                        else:
                            (
                                layout.label(
                                    text="R: "
                                    + str(redVal)
                                    + " | G: "
                                    + str(greenVal)
                                    + " | B: "
                                    + str(blueVal)
                                )
                                if scn.my_list2[value].ShowValueDebug
                                else 0
                            )

                    # Individually addressable
                    elif (
                        not scn.my_list2[value].LEDSingle
                        and scn.my_list2[value].GeoSelect != "None"
                        and scn.my_list2[value].RedSelect2 != "None"
                        and scn.my_list2[value].GreenSelect2 != "None"
                        and scn.my_list2[value].BlueSelect2 != "None"
                    ):
                        if value > 0:
                            from_index = len(scn.my_list) * 2

                            for val in range(0, len(scn.my_list2)):
                                if scn.my_list2[val].LEDType == "Single Color":
                                    from_index += 1
                                else:
                                    if isNeopixelRGBW(val):
                                        from_index = (
                                            from_index
                                            + 1
                                            + scn.my_list2[val].NumLeds * 4
                                        )
                                    else:
                                        from_index = (
                                            from_index
                                            + 1
                                            + scn.my_list2[val].NumLeds * 3
                                        )

                        else:
                            from_index = len(scn.my_list) * 2 + 1

                        debug_string = ""

                        if scn.my_list2[value].RGBW:
                            to_index = from_index + scn.my_list2[value].NumLeds * 4 - 1
                        else:
                            to_index = from_index + scn.my_list2[value].NumLeds * 3 - 1

                        for i in range(from_index, to_index):
                            debug_string = debug_string + str(outputbytes[i]) + " "

                        (
                            layout.label(text="LED values: " + debug_string)
                            if scn.my_list2[value].ShowValueDebug
                            else 0
                        )

                # PWM channels
                elif (
                    scn.my_list2[value].ShowValueDebug == True
                    and scn.my_list2[value].LEDType == "Single Color"
                ):
                    if (
                        scn.my_list2[value].ArmatureSelect != "None"
                        and scn.my_list2[value].LEDSelect != "None"
                        and scn.my_list2[value].LEDAngle != "None"
                    ):
                        layout.label(
                            text="PWM LED value: "
                            + str(
                                math.floor(
                                    interp(
                                        get_bones_rotation(
                                            scn.my_list2[value].ArmatureSelect,
                                            scn.my_list2[value].LEDSelect,
                                            scn.my_list2[value].LEDAngle,
                                        ),
                                        [0, 90],
                                        [0, 255],
                                    )
                                )
                            )
                        )
        return


# Sync up MarIOnette config with microcontroller (store to EEPROM, SD card, or .h file)?
class MPANEL_PT_MarIOnette_SyncConfig(bpy.types.Panel):
    bl_label = "Sync"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "MarIOnette"
    bl_options = {"DEFAULT_CLOSED"}

    def draw(self, context):
        global configSize

        layout = self.layout  # layout elements
        scn = context.scene  # scene elements

        col = layout.column()
        layout.label(
            text="Create configuration file and import Arduino template from Github"
        )
        # layout.prop(scn.scn_prop,"ConfigType")

        if scn.scn_prop.ConfigType == ".h File":
            # layout.prop(scn.scn_prop, "FileName")
            col = layout.column()
            layout.prop(scn.scn_prop, "OverwriteArduinoFile")
            layout.label(
                text='Uncheck "relative path box" in the file browser to display more readable path'
            )
            layout.prop(scn.scn_prop, "FolderName", text="")
            layout.prop(scn.scn_prop, "FileDirectory", text="")

        if (
            scn.scn_prop.ConfigType == "EEPROM"
            and configSize < scn.scn_prop.ConfigEEPROMSize
        ) or scn.scn_prop.ConfigType != "EEPROM":
            layout.operator("tl.sync_conf")

        return


# Debug panel to view the final output values
class MPANEL_PT_MarIOnette_OutputValuesPanel(bpy.types.Panel):
    bl_label = "Output Values"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "MarIOnette"
    bl_options = {"DEFAULT_CLOSED"}

    def draw(self, context):
        global outputvalue
        layout = self.layout  # layout elements
        scn = context.scene  # scene elements
        mprops = scn.scn_prop

        col = layout.column()

        for value in range(0, len(scn.my_list)):
            if (
                scn.my_list[value].ParentSelect != "None"
                and scn.my_list[value].ChildSelect != "None"
            ):
                col = layout.column()
                col.label(
                    text="Actuator "
                    + str(scn.my_list[value].JointNumber)
                    + " out: "
                    + str(outputvalues[value])
                )

        layout.label(
            text="Total motor values: "
            + str(len(scn.my_list))
            + " ("
            + str(len(scn.my_list) * 2)
            + " bytes)"
        )

        if scn.scn_prop.SendLEDVals:
            ledvaluenumber = 0

            for value in range(0, len(scn.my_list2)):
                if scn.my_list2[value].LEDType == "Single Color":
                    ledvaluenumber = ledvaluenumber + 1

                elif (
                    scn.my_list2[value].LEDType == "Neopixel"
                    or scn.my_list2[value].LEDType == "Dotstar"
                ):
                    ledvaluenumber = ledvaluenumber + 1  # Single or multi byte
                    if scn.my_list2[value].LEDSingle:
                        if scn.my_list2[value].RGBW:
                            ledvaluenumber = ledvaluenumber + 4
                        else:
                            ledvaluenumber = ledvaluenumber + 3
                    else:
                        if scn.my_list2[value].RGBW:
                            ledvaluenumber = (
                                ledvaluenumber + 4 * scn.my_list2[value].NumLeds
                            )
                        else:
                            ledvaluenumber = (
                                ledvaluenumber + 3 * scn.my_list2[value].NumLeds
                            )

            layout.label(
                text="Total LED Values: "
                + str(len(scn.my_list2))
                + " ("
                + str(ledvaluenumber)
                + " bytes)"
            )

        layout.label(
            text="Sending " + str(len(outputbytes) + 2) + " bytes" + str(outputbytes)
        )

        return


# Saves animation to a file in a certain frame range (will overwrite all information if frame data already exists)
class MPANEL_PT_MarIOnette_CachePanel(bpy.types.Panel):
    bl_label = "Cache"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "MarIOnette"
    bl_options = {"DEFAULT_CLOSED"}

    def draw(self, context):
        global outputvalue, outputvalues, outputfile, writingfile, framenumber, outputbytes, last_sync, done_writing

        layout = self.layout  # layout elements
        scn = context.scene  # scene elements
        mprops = scn.scn_prop

        if not scn.scn_prop.ReceivingValues:
            if writingfile:
                col = layout.column()
                layout.label(text="Writing to file...")
                layout.label(text="Please keep this panel open to prevent errors!")
                layout.operator("tl.cache_cancel")
                percentage_done = round((framenumber / scn.scn_prop.EndFrame * 100), 2)
                if percentage_done > 100:
                    percentage_done = 100.00
                layout.label(text="Percentage done: " + str(percentage_done) + "%")
            else:
                layout.label(
                    text="Do not create files with names longer than 7 characters if exporting for Arduino"
                )
                if bpy.context.scene.sync_mode != "NONE":
                    layout.label(
                        text="Please ensure playback is set to every frame in Timeline > Playback > Sync menu"
                    )
                layout.prop(scn.scn_prop, "FileName")
                col = layout.column()
                layout.label(
                    text='Uncheck "relative path box" in the file browser to display more readable path'
                )
                layout.prop(scn.scn_prop, "FileDirectory", text="")

                # layout.prop(scn.scn_prop, "UseSceneFrameRange")

                layout.prop(scn.scn_prop, "StartFrame")
                layout.prop(scn.scn_prop, "EndFrame")

                col = layout.column()
                layout.prop(scn.scn_prop, "WriteCacheHeader")
                (
                    layout.prop(scn.scn_prop, "LEDMultiSingle")
                    if scn.scn_prop.WriteCacheHeader
                    else layout.label(text="No Cache File Selected")
                )
                layout.operator("tl.output_text_file_cache")
                layout.label(
                    text="Please make sure the right animation is active before output"
                )
                layout.prop(scn.scn_prop, "UseCache")
                layout.operator("tl.select_cache_file") if scn.scn_prop.UseCache else 0
                layout.label(text=str(cachefilepath)) if scn.scn_prop.UseCache else 0

                layout.prop(scn.scn_prop, "SDCardEnabled")
                (
                    layout.prop(scn.scn_prop, "SDCardInternal")
                    if scn.scn_prop.SDCardEnabled
                    else 0
                )
                (
                    layout.prop(scn.scn_prop, "CSPin")
                    if not scn.scn_prop.SDCardInternal and scn.scn_prop.SDCardEnabled
                    else 0
                )

                if scn.scn_prop.SerialConnected and scn.scn_prop.SDCardEnabled:
                    layout.operator("tl.play_on_microcontroller")
                    layout.operator("tl.stop_playback")

        else:
            layout.label(
                text='Please uncheck "Receiving Joint Angles" in Serial Setup Tab found above in the MarIOnette panel'
            )

        # Can't do a for loop because that blocks Blender's updating, so we just refresh for every frame
        if writingfile and framenumber < (scn.scn_prop.EndFrame + 1):
            get_angles_and_leds()

            # Find needed length of packet, otherwise Blender will drop values...
            packetLength = 0
            packetLength = len(scn.my_list) * 2

            for i in range(0, len(scn.my_list2)):
                if scn.my_list2[i].LEDSingle:
                    packetLength = packetLength + 4
                else:
                    packetLength = packetLength + scn.my_list2[i].NumLeds * 4

            outputfile.write(bytes(outputbytes))
            # outputfile.write('\n'.encode())
            # Go to next frame
            framenumber = framenumber + 1
            scn.frame_set(framenumber)
            bpy.context.area.tag_redraw()
            # get_angles_and_leds()

        # Done writing, close file
        elif writingfile and framenumber == (scn.scn_prop.EndFrame + 1):
            writingfile = False
            layout.label(text="Closing file, please wait")
            outputfile.close()
            # layout.operator("tl.update_sync_mode")
            # updateFrameSync.execute(self, bpy)
            done_writing = True
            # bpy.context.scene.sync_mode  = last_sync
            # bpy.data.scenes[0].sync_mode = last_sync

        return


# Hidden panel that updates all the values so we don't have to keep other panels open
class MPANEL_PT_MarIOnette_updatePanel(bpy.types.Panel):
    bl_label = "Update Panel"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "MarIOnette"  # creates a new tab in the tool menu on the left of the blender 3d viewer. Press 'T' if tool window is hidden
    # bl_context="posemode"
    bl_options = {"HIDE_HEADER"}

    def draw(self, context):
        global outputvalue, outputvalues, outputbytes, bytesToSend, lastframenumber, inputservos, inputvalues, inputvalues2, cachefilepath, writingfile, cacheFileOject, readingCacheFile
        global cacheheaderlength, uploadingnimation, updateActive

        # Start handler
        if not updateActive:
            # print("Update per frame not active")
            bpy.app.handlers.frame_change_post.append(handler1)

        # global serialport
        layout = self.layout  # layout elements
        scn = context.scene  # scene elements
        mprops = scn.scn_prop

        """
        # Update out output values length
        while len(outputvalues) > len(scn.my_list):
            #outputvalues.remove(len(outputvalues-1))
            outputvalues.pop()
        """

        # Read from cached file
        current_frame = bpy.data.scenes[0].frame_current
        global serialport

        if (
            mprops.UseCache
            and lastframenumber != current_frame
            and not writingfile
            and current_frame <= scn.scn_prop.EndFrame
            and cachefilepath != 0
            and not uploadingnimation
        ):
            if not readingCacheFile and cachefilepath != 0:
                readingCacheFile = True
                print(cachefilepath)
                cacheFileOject = open(cachefilepath, "rb")
                cacheFileOject.seek(0, os.SEEK_END)
                print("Opened file of size:", cacheFileOject.tell(), "bytes")

            lastframenumber = current_frame
            outputbytes.clear()
            tempOutputBytes = []
            tempOutputBytes = bytearray(tempOutputBytes)

            # Find needed length of packet, otherwise Blender will drop values...
            packetLength = 0
            packetLength = len(scn.my_list) * 2

            isRGBW = False

            for i in range(0, len(scn.my_list2)):
                if scn.my_list2[i].LEDType == "Single Color":
                    packetLength = packetLength + 1

                else:
                    if (
                        scn.my_list2[i].NeopixelOrder == "NEO_RGBW + NEO_KHZ800"
                        or scn.my_list2[i].NeopixelOrder == "NEO_GRBW + NEO_KHZ800"
                        or scn.my_list2[i].NeopixelOrder == "NEO_RGBW + NEO_KHZ400"
                        or scn.my_list2[i].NeopixelOrder == "NEO_GRBW + NEO_KHZ400"
                    ):
                        isRGBW = True

                    if scn.my_list2[i].LEDSingle:
                        if not isRGBW:
                            packetLength = packetLength + 3
                        else:
                            packetLength = packetLength + 4
                    else:
                        if not isRGBW:
                            packetLength = packetLength + scn.my_list2[i].NumLeds * 3
                        else:
                            packetLength = packetLength + scn.my_list2[i].NumLeds * 4

            global headerLen
            packetLength = packetLength + 1  # account for newline character
            if mprops.WriteCacheHeader:
                position = ((current_frame - 1) * packetLength) + headerLen
            else:
                position = (current_frame - 1) * packetLength

            # print(position)

            cacheFileOject.seek(position)

            for i in range(position, (position + packetLength)):
                tempOutputBytes = tempOutputBytes + (cacheFileOject.read(1))

            for i in range(0, len(tempOutputBytes)):
                outputbytes.append((tempOutputBytes[i]))

            # Now Send everything through serial!
            # global serialport
            if (
                scn.scn_prop.SerialConnected
                and (not uploadingnimation)
                and (not scn.scn_prop.ReceivingValues)
                and serialport.isOpen()
            ):
                # Convert to byte arrays
                message = bytearray(outputbytes)
                speed = bytearray(
                    math.floor(scn.scn_prop.BusServoSpeed).to_bytes(2, "big")
                )
                length = bytearray(len(outputbytes).to_bytes(2, "big"))

                # Write the data
                serialport.write("B".encode() + length + message + ".".encode())

        # Always update for live update
        elif not mprops.UseCache and not writingfile and not uploadingnimation:
            if readingCacheFile:
                readingCacheFile = False
                cacheFileOject.close()
                print("Cache file closed")

            lastframenumber = current_frame
            get_angles_and_leds()
            # global serialport
            # Now Send everything through serial!
            if (
                scn.scn_prop.SerialConnected
                and (not uploadingnimation)
                and (not scn.scn_prop.ReceivingValues)
                and serialport.isOpen()
            ):
                # Convert to byte array before sending
                # global serialport
                message = bytearray(outputbytes)
                speed = bytearray(
                    math.floor(scn.scn_prop.BusServoSpeed).to_bytes(2, "big")
                )
                length = bytearray(len(outputbytes).to_bytes(2, "big"))

                # Write the data
                serialport.write("A".encode() + length + speed + message + ".".encode())

        return


def handler1(scene):
    global updateActive
    updateActive = True

    for area in bpy.context.screen.areas:
        if area.type == "VIEW_3D":
            area.tag_redraw()


classes = (
    ListItem,
    ListItem2,
    SceneProperties,
    LIST_UL_Items,
    LIST_UL_Items2,
    Uilist_actions,
    Uilist_actions2,
    ConnectSerial,
    DisconnectSerial,
    updateSerialConnection,
    SendSingle,
    setAngles,
    setAnglesModal,
    recordAnglesModal,
    OutputToTextFileCache,
    OutputToTextFileCancel,
    SelectCacheFile,
    Sync,
    SetKeyframe,
    refreshSerialPorts,
    updateConfigSize,
    ToggleArmatureAxisVisibility,
    UnlockBoneAxes,
    LockOtherBoneAxes,
    updateFrameSync,
    playOnMicrocontroller,
    stopMicrocontrollerPlayback,
    MPANEL_PT_MarIOnette_SerialPanel,
    MPANEL_PT_MarIOnette_ActuatorPanel,
    MPANEL_PT_MarIOnette_LedPanel,
    MPANEL_PT_MarIOnette_OutputValuesPanel,
    MPANEL_PT_MarIOnette_CachePanel,
    MPANEL_PT_MarIOnette_SyncConfig,
    MPANEL_PT_MarIOnette_updatePanel,
)


def register():
    from bpy.utils import register_class

    for cls in classes:
        register_class(cls)

    bpy.types.Scene.my_list = bpy.props.CollectionProperty(type=ListItem)
    bpy.types.Scene.my_list2 = bpy.props.CollectionProperty(type=ListItem2)
    bpy.types.Scene.scn_prop = bpy.props.PointerProperty(type=SceneProperties)
    bpy.types.Scene.list_index = bpy.props.IntProperty(
        name="Index for my_list", default=0
    )
    bpy.types.Scene.list_index2 = bpy.props.IntProperty(
        name="Index for my_list2", default=0
    )
    bpy.app.handlers.frame_change_post.append(handler1)


def unregister():
    from bpy.utils import unregister_class

    for cls in reversed(classes):
        unregister_class(cls)

    del bpy.types.Scene.my_list
    del bpy.types.Scene.my_list2
    del bpy.types.Scene.list_index
    del bpy.types.Scene.list_index2
    del bpy.types.Scene.scn_prop
    bpy.app.handlers.frame_change_post.remove(handler1)

    global updateActive
    updateActive = False

    """
    from bpy.utils import unregister_class
    for cls in reversed(classes):
        unregister_class(cls)
    """


if __name__ == "__main__":
    register()
