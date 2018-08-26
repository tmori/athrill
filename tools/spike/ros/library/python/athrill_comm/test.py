import sys
from mmap_io import MmapIo
from athrill_bus import *
from ctype_primitive import *
from ctype_array import *

def test(args):
    print(args[1])
    rawbus = MmapIo(args[1])
    rawbus.open()

    bus = AthrillBus(rawbus)
    bus.load()

    #reader = bus.meta_reader
    #print("meta_version=" + str(reader.meta_version))
    #print("meta_busid=" + str(reader.meta_busid))

    bus.read(2)

    rawbus.close()

if __name__ == '__main__':
    args = sys.argv
    test(args)