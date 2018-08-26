import mmap
import fcntl
import io

class MmapIo:
    def __init__(self, filepath):
        self.mmap_filepath = filepath
        self.mm = None

    def open(self):
        self.f = open(self.mmap_filepath, "r+b")
        self.mm = mmap.mmap(fileno=self.f.fileno(), length=0, flags=mmap.MAP_SHARED, prot=mmap.PROT_READ | mmap.PROT_WRITE)

    def _ex_lock(self):
        try:
            fcntl.flock(self.f.fileno(), fcntl.LOCK_EX)
            return True
        except IOError:
            return False

    def _ex_unlock(self):
        fcntl.flock(self.f.fileno(), fcntl.LOCK_UN)

    def read(self, off, size):
        bin = self.mm[off:off+size]
        return bin

    def write(self, off, data):
        self.mm.seek(off)
        self.mm.write(data)

    def close(self):
        if self.mm is not None:
	        self.mm.close()
	        self.mm = None
