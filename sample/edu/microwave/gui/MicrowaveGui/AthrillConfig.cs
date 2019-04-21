using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MicrowaveGui
{
    class AthrillConfig
    {
        private string folderPath = null;
        private string defaultPath = @"C:\project\esm\athrill\sample\edu\microwave";

        public AthrillConfig()
        {
        }

        public string GetDefaultPath()
        {
            return defaultPath;
        }
        public void SetPath(string path)
        {
            folderPath = path;
        }

        public string GetDigitalPath()
        {
            if (folderPath == null)
            {
                return null;
            }
            return folderPath + "\\bin\\digital\\digital.bin";
        }
        public string GetSerialPath()
        {
            if (folderPath == null)
            {
                return null;
            }
            return folderPath + "\\bin\\serial\\serial_in.txt";
        }

        public string GetFolderPath()
        {
            return folderPath;
        }

    }
}
