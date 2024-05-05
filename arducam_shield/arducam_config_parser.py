import ctypes
import sys
from ament_index_python import get_package_share_directory

try:
    complied_so = get_package_share_directory("arducam_shield") + "/libfiles/" + "libarducam_config_parser.so"
    _lib = ctypes.cdll.LoadLibrary(complied_so)
except Exception as e:
    print("Load libarducam_config_parser fail.")
    print("Make sure you have arducam_config_parser installed.")
    print("For more information, please visit: https://github.com/ArduCAM/arducam_config_parser")
    print(e)
    sys.exit(0)


MAX_CONFIGS = 8192
SECTION_TYPE_CAMERA = 0x01 << 24
SECTION_TYPE_BOARD = 0x02 << 24
SECTION_TYPE_REG = 0x03 << 24

SECTION_TYPE_BOARD_2 = SECTION_TYPE_BOARD | (0x02 << 16)
SECTION_TYPE_BOARD_3_2 = SECTION_TYPE_BOARD | (0x04 << 16)
SECTION_TYPE_BOARD_3_3 = SECTION_TYPE_BOARD | (0x03 << 16)
SECTION_TYPE_REG_3_2 = SECTION_TYPE_REG | (0x04 << 16)
SECTION_TYPE_REG_3_3 = SECTION_TYPE_REG | (0x03 << 16)

CONFIG_TYPE_REG = 0x0001
CONFIG_TYPE_VRCMD = 0x0002
CONFIG_TYPE_DELAY = 0x0003

# +───────────────+───────────+──────────────+
# |     8bit      |    8bit   |    16bit     |
# +───────────────+───────────+──────────────+
# | section type  | usb type  | config type  |
# +───────────────+───────────+──────────────+


class Config(ctypes.Structure):
    _fields_ = [
        ("type", ctypes.c_uint32),
        ("params", ctypes.c_uint32 * 16),
        ("params_length", ctypes.c_uint8),
    ]


class Control(ctypes.Structure):
    _fields_ = [
        ("min", ctypes.c_int64),
        ("max", ctypes.c_int64),
        ("step", ctypes.c_int32),
        ("def", ctypes.c_int64),
        ("flags", ctypes.c_uint32),
        ("name", ctypes.c_char * 128),
        ("func", ctypes.c_char * 128),
        ("code", ctypes.c_char_p),
    ]


class CameraParam(ctypes.Structure):
    _fields_ = [
        ("cfg_mode", ctypes.c_uint32),
        ("type", ctypes.c_char * 50),
        ("width", ctypes.c_uint32),
        ("height", ctypes.c_uint32),
        ("bit_width", ctypes.c_uint8),
        ("format", ctypes.c_uint16),
        ("i2c_mode", ctypes.c_uint8),
        ("i2c_addr", ctypes.c_uint16),
        ("trans_lvl", ctypes.c_uint32),
    ]

    def getdict(struct):
        return dict((field.upper(), getattr(struct, field) if field != "format" else (struct.format >> 8, struct.format & 0xFF)) for field, _ in struct._fields_)


class CameraConfigs(ctypes.Structure):
    _fields_ = [
        ("camera_param", CameraParam),
        ("configs", ctypes.POINTER(Config)),
        ("configs_length", ctypes.c_uint32),
        ("controls", ctypes.POINTER(Control)),
        ("controls_length", ctypes.c_uint32),
    ]


parse = _lib.arducam_parse_config
parse.argtypes = [ctypes.c_char_p, ctypes.POINTER(CameraConfigs)]


def LoadConfigFile(name):
    cfgs = CameraConfigs()
    if sys.version_info[0] == 3:
        filename = name.encode("utf-8")
    else:
        filename = name
    if parse(filename, ctypes.byref(cfgs)) != 0:
        raise RuntimeError("Loading configuration file {} failed.".format(name))
    return cfgs
