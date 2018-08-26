from ctypes import *
from enum import Enum

class CtypePrimtiveType(Enum):
    uint8  = 1
    uint16 = 2
    uint32 = 4


class CTypePrimitiveUint8(Structure):
	_fields_ = (
		('data', c_uint8),
	)

class CTypePrimitiveUint16(Structure):
	_fields_ = (
		('data', c_uint16),
	)


class CTypePrimitiveUint32(Structure):
	_fields_ = (
		('data', c_uint32),
	)


class CtypePrimitiveReader:
    def __init__(self, binary, ctype):
        self.data = None
        if ctype == CtypePrimtiveType.uint8:
            self.p_point = cast(binary, POINTER(CTypePrimitiveUint8))
            self.data = self.p_point.contents.data
        elif ctype == CtypePrimtiveType.uint16:
            self.p_point = cast(binary, POINTER(CTypePrimitiveUint16))
            self.data = self.p_point.contents.data
        elif ctype == CtypePrimtiveType.uint32:
            self.p_point = cast(binary, POINTER(CTypePrimitiveUint32))
            self.data = self.p_point.contents.data


    def read(self):
        return str(self.data)
