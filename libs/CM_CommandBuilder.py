"""CameraManager command builder python version.\n
Note this file is kinda just gonna be a massive blob.\n
If someone requests, I'll decompress it later.\n
This includes *NO* ROS2 code, only returning a c_uint32 that you must deal with.\n
See other documentation examples for this on the google drive."""

from ctypes import c_uint32 # integer class for forming the command accurately
from std_msgs.msg import UInt32
from enum import Enum

class MessageType(Enum):
    """Message type enumeration"""
    LOCAL_START = 0
    STREAM_START = 1
    LOCAL_STOP = 2
    STREAM_STOP = 3
    FORCE_RESTART = 4
    MODIFY_ATTR = 5

    # Provided for testing purposes only
    DUMMY = 7

class Quality(Enum):
    """Quality enumeration.\n
    Pulled from settings.hpp"""
    lowest = 0 # ///< 160x90, 5fps
    low = 1 # ///< 320x180, 5fps
    lowish = 2 # ///< 320x180, 10fps
    okay = 3 # ///< 640x360, 10fps
    okayish = 4 # ///< 640x360, 15fps
    medium = 5 # ///< 1280x720, 10fps
    mediumish = 6 # ///< 1280x720, 15fps
    high = 7 # ///< 1920x1080, 10fps
    higher = 8 # ///< 1920x1080, 15fps
    highest = 9 # ///< 1920x1080, 20fps

class Command:
    def __init__(self):
        self.__message = UInt32()
        self.__data:int = 0
        self.__message_type = None

    def set_message_type(self, type:MessageType):
        """Sets message type. Not dependent on other parameters."""
        self.__data &= 0x1FFF_FFFF # preserve all bits except top 3
        self.__data |= (type.value << 29) # set the top 3 as applicable
        self.__message_type = type

    def set_cam_id(self, id:int):
        """Sets camera id. Dependent on message type.\n
        Out of caution, reset this whenever you change the message type.\n
        :raises: ValueError when Message Type was not set before attempting."""
        if self.__message_type is None:
            raise ValueError("Message type has not been set, so camera id cannot be set.")
        
        if self.__message_type == MessageType.LOCAL_START or self.__message_type == MessageType.STREAM_START:
            # Long command -> 23:19
            self.__data &= 0xFF07_FFFF
            self.__data |= (id << 19)
        else:
            # Short command -> 28:24
            self.__data &= 0xE0FF_FFFF # clear applicable bits only
            self.__data |= (id << 24)

    def set_quality(self, qual:Quality):
        """Sets quality. Dependent on message type.\n
        Out of caution, reset this whenever you change the message type.\n
        :raises: ValueError when Message Type was not set before attempting."""
        if self.__message_type is None:
            raise ValueError("Message type has not been set, so camera id cannot be set.")
        
        if self.__message_type == MessageType.LOCAL_START or self.__message_type == MessageType.STREAM_START:
            # Long command -> 28:24
            self.__data &= 0xE0FF_FFFF # clear applicable bits only
            self.__data |= (qual.value << 24)
        else:
            # Short command -> not applicable
            raise ValueError("Quality setting not applicable to short commands")
        
    def set_attribute(self, attr: int, attr_value: int):
        if self.__message_type != MessageType.MODIFY_ATTR:
            raise ValueError("Message not appropriate for attribute setting")
        
        # Make attr_bits from value (int16)
        attr_bits = 0
        if attr_value > 32767 or attr_value < -32768:
            raise ValueError("Attribute value outside of range of int16")
        elif attr_value < 0:
            attr_bits |= (1 << 15)
            attr_bits |= (-(attr_value))
        
        # Attribute -> 23:20, Value -> 16:0
        self.__data &= 0xFF0F_0000
        self.__data |= (attr << 20)
        self.__data |= (attr_bits)

    def get_message(self):
        """Returns the message."""
        self.__message.data = self.__data
        return self.__message

def demo():
    cmd = Command()
    cmd.set_message_type(MessageType.STREAM_START)
    cmd.set_cam_id(3)
    cmd.set_quality(Quality.okayish)

    msg:UInt32 = cmd.get_message()
    print("The below number excludes the highest order zeros")
    print(bin(msg.data))

if __name__ == "__main__":
    demo()