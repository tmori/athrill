using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MicrowaveGui
{
    public class AthrillDigital
    {
        public void SetBit(string path, int bitoff)
        {
            byte[] buf = new byte[1]; // データ格納用配列
            int pattern = 1 << bitoff;
            int data;


            using (FileStream fs = new FileStream(path, FileMode.Open, FileAccess.ReadWrite, FileShare.ReadWrite))
            {
                fs.Read(buf, 0, 1);
                data = buf[0];
                data = data | pattern;
                buf[0] = (byte)data;
                fs.Seek(0, SeekOrigin.Begin);
                fs.WriteByte(buf[0]);
                Console.WriteLine(buf[0]);
            }
        }
        public void ClearBit(string path, int bitoff)
        {
            byte[] buf = new byte[1]; // データ格納用配列
            int pattern = 1 << bitoff;
            int data;


            using (FileStream fs = new FileStream(path, FileMode.Open, FileAccess.ReadWrite, FileShare.ReadWrite))
            {
                fs.Read(buf, 0, 1);
                data = buf[0];
                data = data & ~pattern;
                buf[0] = (byte)data;
                fs.Seek(0, SeekOrigin.Begin);
                fs.WriteByte(buf[0]);
                Console.WriteLine(buf[0]);
            }
        }
    }
}
