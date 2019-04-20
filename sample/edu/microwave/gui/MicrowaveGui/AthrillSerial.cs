using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MicrowaveGui
{
    enum SerialStateType
    {
        INIT,
        KEY,
        VALUE,
    }

    class AthrillSerial
    {
        private string serial_path = null;
        private long serial_fsize;
        private long serial_off;
        private Hashtable hash = new Hashtable();
        private SerialStateType state = SerialStateType.INIT;

        private string key = null;
        private string value = null;
        public AthrillSerial()
        {

        }

        public void Open(string path)
        {
            serial_path = path;
            using (FileStream fs = new FileStream(path, FileMode.Open, FileAccess.ReadWrite, FileShare.ReadWrite))
            {
                serial_fsize = fs.Length;
                serial_off = serial_fsize;
            }
        }

        public void Parse()
        {
            if (serial_path == null)
            {
                return;
            }
            byte[] data = new byte[1];

            while (true)
            {
                using (FileStream fs = new FileStream(serial_path, FileMode.Open, FileAccess.ReadWrite, FileShare.ReadWrite))
                {
                    fs.Seek(serial_off, SeekOrigin.Begin);
                    var readSize = fs.Read(data, (int)0, 1);
                    if (readSize <= 0)
                    {
                        return;
                    }
                    serial_off++;
                }

                switch (state)
                {
                    case SerialStateType.INIT:
                        key = System.Text.Encoding.ASCII.GetString(data);
                        state = SerialStateType.KEY;
                        break;
                    case SerialStateType.KEY:
                        if (data[0] == '\n')
                        {
                            value = null;
                            state = SerialStateType.VALUE;
                        }
                        else
                        {
                            key += System.Text.Encoding.ASCII.GetString(data);
                        }
                        break;
                    case SerialStateType.VALUE:
                        if (data[0] == '\n')
                        {
                            hash[key] = value;
                            state = SerialStateType.INIT;
                        }
                        else
                        {
                            value += System.Text.Encoding.ASCII.GetString(data);
                        }
                        break;
                    default:
                        break;

                }
            }

        }

        public string GetValue(string key)
        {
            return (string)hash[key];
        }
    }
}
