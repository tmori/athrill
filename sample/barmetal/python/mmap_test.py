import mmap
import fcntl
import io
from ctypes import *

class CanDataStructure(Structure):
	_fields_ = (
		('isArrival', c_int),
		('isSended', c_int),
		('c0', c_uint8),
		('c1', c_uint8),
		('c2', c_uint8),
		('c3', c_uint8),
		('c4', c_uint8),
		('c5', c_uint8),
		('c6', c_uint8),
		('c7', c_uint8),
	)

class CanDataStructureReader:
    def __init__(self, binary):
        self.p_point = cast(binary, POINTER(CanDataStructure))
        self.can_data = [ 
            self.p_point.contents.c0, 
            self.p_point.contents.c1, 
            self.p_point.contents.c2, 
            self.p_point.contents.c3, 
            self.p_point.contents.c4, 
            self.p_point.contents.c5, 
            self.p_point.contents.c6, 
            self.p_point.contents.c7 ]
    
    def isArrival(self):
        if (str(self.p_point.contents.isArrival) == "0"):
            return False
        else:
            return True

    def isSended(self):
        if (str(self.p_point.contents.isSended) == "0"):
            return False
        else:
            return True
    
    def read(self, off):
        return str(self.can_data[off])

class CanDataStructureWriter:
    def __init__(self):
        self.contents = CanDataStructure()
    
    def setIsArrival(self, flag):
        if (flag == True):
            self.contents.isArrival = 1
        else:
            self.contents.isArrival = 0

    def setIsSended(self, flag):
        if (flag == True):
            self.contents.isSended = 1
        else:
            self.contents.isSended = 0
    
    def write(self, off, data):
        if off == 0:
            self.contents.c0 = data
        elif off == 1:
            self.contents.c1 = data
        elif off == 2:
            self.contents.c2 = data
        elif off == 3:
            self.contents.c3 = data
        elif off == 4:
            self.contents.c4 = data
        elif off == 5:
            self.contents.c5 = data
        elif off == 6:
            self.contents.c6 = data
        else:
            self.contents.c7 = data

    def getBinaryData(self):
        buffer = io.BytesIO()
        buffer.write(self.contents)
        return buffer.getvalue()


class MmapCanBus:
    def __init__(self, filepath):
        self.mmap_filepath = filepath
        self.mm = None

    def open(self, can_message_size):
        self.f = open(self.mmap_filepath, "r+b")
        self.mm = mmap.mmap(fileno=self.f.fileno(), length=0, flags=mmap.MAP_SHARED, prot=mmap.PROT_READ | mmap.PROT_WRITE)
        self.can_message_size = can_message_size

    def _ex_lock(self):
        try:
            fcntl.flock(self.f.fileno(), fcntl.LOCK_EX)
            return True
        except IOError:
            return False

    def _ex_unlock(self):
        fcntl.flock(self.f.fileno(), fcntl.LOCK_UN)

    def receive(self, id):
        off = id * self.can_message_size;
        if (self._ex_lock() == True):
            bin = self.mm[off:off+self.can_message_size]
            self._ex_unlock()
            return bin
        else:
            return None

    def send(self, id, can_message):
        off = id * self.can_message_size

        if (self._ex_lock() == True):
            self.mm.seek(off)
            self.mm.write(can_message)
            self._ex_unlock()
            return True
        else:
            return False

    def close(self):
        if self.mm is None:
	        self.mm.close()
	        self.mm = None


def test():
    canbus = MmapCanBus("mmap_file.bin")
    canbus.open(sizeof(CanDataStructure))

    #READ sample
    binary = canbus.receive(3)
    reader = CanDataStructureReader(binary)
    print(str(reader.isArrival()))
    print(str(reader.isSended()))
    print(reader.read(0))
    print(reader.read(1))
    print(reader.read(2))
    print(reader.read(3))
    print(reader.read(4))
    print(reader.read(5))
    print(reader.read(6))
    print(reader.read(7))

    #WRITE sample
    can_message = CanDataStructureWriter()
    can_message.setIsArrival(True)
    can_message.setIsSended(True)
    can_message.write(0, 5)
    can_message.write(1, 4)
    can_message.write(2, 3)
    can_message.write(3, 9)
    can_message.write(4, 255)
    can_message.write(5, 128)
    can_message.write(6, 127)
    can_message.write(7, 99)
    canbus.send(3, can_message.getBinaryData())
    canbus.close()

if __name__ == '__main__':
    test()