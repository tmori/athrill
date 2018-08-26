
from ctypes import *
from ctype_primitive import *
from ctype_array import *

class AthrillBusMetaDataType(Structure):
	_fields_ = (
		('meta_version', c_uint32),
		('meta_magicno', c_uint32),
		('meta_busid', c_uint32),
		('meta_entrynum', c_uint32),
		('meta_buffer_offset_soff', c_uint32),
		('meta_buffer_offset_size', c_uint32),
		('meta_buffer_size_soff', c_uint32),
		('meta_buffer_size_size', c_uint32),
		('meta_buffer_elmsize_soff', c_uint32),
		('meta_buffer_elmsize_size', c_uint32),
		('meta_buffer_type_soff', c_uint32),
		('meta_buffer_type_size', c_uint32),
		('data_data_soff', c_uint32),
		('data_data_size', c_uint32),
	)


class AthrillBusMetaDataReader:
    def __init__(self, binary):
        self.p_point = cast(binary, POINTER(AthrillBusMetaDataType))
        self.meta_version = self.p_point.contents.meta_version
        self.meta_busid = self.p_point.contents.meta_busid
        self.meta_entrynum = self.p_point.contents.meta_entrynum
        self.meta_buffer_offset_soff = self.p_point.contents.meta_buffer_offset_soff
        self.meta_buffer_offset_size = self.p_point.contents.meta_buffer_offset_size
        self.meta_buffer_size_soff = self.p_point.contents.meta_buffer_size_soff
        self.meta_buffer_size_size = self.p_point.contents.meta_buffer_size_size
        self.meta_buffer_elmsize_soff = self.p_point.contents.meta_buffer_elmsize_soff
        self.meta_buffer_elmsize_size = self.p_point.contents.meta_buffer_elmsize_size
        self.meta_buffer_type_soff = self.p_point.contents.meta_buffer_type_soff
        self.meta_buffer_type_size = self.p_point.contents.meta_buffer_type_size
        self.data_data_soff = self.p_point.contents.data_data_soff
        self.data_data_size = self.p_point.contents.data_data_size

class AthrillBus:
    def __init__(self, rawbus):
        self.version = "1.0.0"
        self.meta_size = 56
        self.rawbus = rawbus

    def load(self):
        meta = self.rawbus.read(0, self.meta_size)
        self.meta_reader = AthrillBusMetaDataReader(meta)

        raw_data = self.rawbus.read(self.meta_reader.meta_buffer_offset_soff, self.meta_reader.meta_buffer_offset_size)
        self.elm_soff_array = CtypePrimitiveArrayReader(raw_data, CtypePrimtiveType.uint32, self.meta_reader.meta_entrynum)

        raw_data = self.rawbus.read(self.meta_reader.meta_buffer_size_soff, self.meta_reader.meta_buffer_size_size)
        self.elm_size_array = CtypePrimitiveArrayReader(raw_data, CtypePrimtiveType.uint32, self.meta_reader.meta_entrynum)

        raw_data = self.rawbus.read(self.meta_reader.meta_buffer_elmsize_soff, self.meta_reader.meta_buffer_elmsize_size)
        self.elm_elmsize_array = CtypePrimitiveArrayReader(raw_data, CtypePrimtiveType.uint32, self.meta_reader.meta_entrynum)

        raw_data = self.rawbus.read(self.meta_reader.meta_buffer_type_soff, self.meta_reader.meta_buffer_type_size)
        self.elm_type_array = CtypePrimitiveArrayReader(raw_data, CtypePrimtiveType.uint32, self.meta_reader.meta_entrynum)

        #print("soff[0]=" + str(self.elm_soff_array.array[0]))
        #print("size[0]=" + str(self.elm_size_array.array[0]))
        #print("elmsize[0]=" + str(self.elm_elmsize_array.array[0]))
        #print("type[0]=" + str(self.elm_type_array.array[0]))


    def read(self, index):
        if (index >= self.meta_reader.meta_entrynum):
            print("ERROR:AthrillBus.read():Invalid index("+str(index)+")")
            return

        etype = self.elm_type_array.array[index]
        if etype != 0:
            print("ERROR:AthrillBus.read():Invalid type("+str(etype)+")")
            return

        soff = self.elm_soff_array.array[index]
        size = self.elm_size_array.array[index]
        eoff = soff + self.elm_size_array.array[index]
        esize = self.elm_elmsize_array.array[index]

        raw_data = self.rawbus.read(soff, eoff)
        if (size <= 4):
            print("Primtive")
        else:
            print("Array")
 