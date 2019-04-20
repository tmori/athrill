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

        public AthrillConfig()
        {
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
