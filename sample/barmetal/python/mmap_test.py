import mmap
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

t = CanDataStructure()

f = open("mmap_file.bin", "r+b")
mm = mmap.mmap(fileno=f.fileno(), length=0, flags=mmap.MAP_SHARED, prot=mmap.PROT_READ | mmap.PROT_WRITE)


#READ sample
id = 3
off = id * sizeof(CanDataStructure);
p_point = cast(mm[off:off+sizeof(CanDataStructure)], POINTER(CanDataStructure))

#print(sizeof(CanDataStructure))

print(str(p_point.contents.isArrival))
print(str(p_point.contents.isSended))
print(str(p_point.contents.c0))
print(str(p_point.contents.c1))
print(str(p_point.contents.c2))
print(str(p_point.contents.c3))
print(str(p_point.contents.c4))
print(str(p_point.contents.c5))
print(str(p_point.contents.c6))
print(str(p_point.contents.c7))

#WRITE sample
id = 3
off = id * sizeof(CanDataStructure);


buffer = io.BytesIO()
buffer.write(CanDataStructure(1, 0, 0, 1, 2, 3, 4, 5, 6, 7))
mm.seek(off)
mm.write(buffer.getvalue())

mm.close()


