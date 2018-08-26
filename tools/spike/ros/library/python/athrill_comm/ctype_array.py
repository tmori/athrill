from ctypes import *
from ctype_primitive import *

class CtypePrimitiveArrayReader:
    def __init__(self, binary, ctype, array_size):
        if ctype == CtypePrimtiveType.uint8:
            size = 1
        elif ctype == CtypePrimtiveType.uint16:
            size = 2
        elif ctype == CtypePrimtiveType.uint32:
            size = 4
        else:
            return None

        self.array = []
        for index in  range(array_size):
            off = index * size
            eoff = off + size
            parts_array = binary[off:eoff]
            if ctype == CtypePrimtiveType.uint8:
                p_point = cast(parts_array, POINTER(CTypePrimitiveUint8))
                self.array.append(p_point.contents.data)
            elif ctype == CtypePrimtiveType.uint16:
                p_point = cast(parts_array, POINTER(CTypePrimitiveUint16))
                self.array.append(p_point.contents.data)
            elif ctype == CtypePrimtiveType.uint32:
                p_point = cast(parts_array, POINTER(CTypePrimitiveUint32))
                self.array.append(p_point.contents.data)


