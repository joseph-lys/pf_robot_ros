""" command_struct.py

copyright Joseph Lee Yuan Sheng 2019

"""
from vstruct import BoolItem, AlignPad, LEItem, LEArray, Type, VStruct


class CommandStruct(VStruct):
    """ CommandStruct

    Command Struct Data
    """
    enabled = BoolItem()  # motor is enabled
    size = LEItem(Type.uint8_t, bit_size=6)  # number of motors
    pad1 = AlignPad(1)  # pad so next element starts on a new byte

    ids = LEArray(Type.uint8_t, bit_size=5, array_size=32)  # motor ids
    pad2 = AlignPad(1)  # pad so next element starts on a new byte
    position = LEArray(Type.uint16_t, bit_size=10, array_size=32)  # goal position
    speed = LEArray(Type.uint16_t, bit_size=12, array_size=32)  # moving speed
    pad3 = AlignPad(1)  # pad so next element starts on a new byte

