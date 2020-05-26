""" feedback_struct.py

copyright Joseph Lee Yuan Sheng 2019

"""
from vstruct import BoolItem, AlignPad, LEItem, LEArray, Type, VStruct


class FeedbackStruct(VStruct):
    """ FeedbackStruct

    Feedback Struct Data
    """
    system_time = LEItem(Type.uint32_t, bit_size=32)  # system time on hardware board	
    voltage = LEItem(Type.uint8_t, bit_size=8)  # motor voltage retrieved from one of the motors
    enabled = BoolItem()  # motor is enabled
    size = LEItem(Type.uint8_t, bit_size=6)  # number of motors
    pad1 = AlignPad(1)  # pad so next element starts on a new byte

    # The following arrays shall be following the motor ids of the CommandStruct
    status = LEArray(Type.uint8_t, bit_size=8, array_size=32)  # feedback status from motor
    position = LEArray(Type.uint16_t, bit_size=10, array_size=32)  # present position
    speed = LEArray(Type.uint16_t, bit_size=12, array_size=32)  # present speed
    load = LEArray(Type.uint16_t, bit_size=12, array_size=32)  # present position	
    pad2 = AlignPad(1)  # pad so next element starts on a new byte
    
    ain = LEArray(Type.uint16_t, bit_size=12, array_size=8)  # value of analog inputs


