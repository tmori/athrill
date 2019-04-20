using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.IO;
using System.IO.MemoryMappedFiles;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace MicrowaveGui
{
    enum TimerType
    {
        TimerStartButton,
        TimerStopButton,
        TimerUp1Button,
        TimerDown1Button,
        TimerUp2Button,
        TimerDown2Button,
        TimerBackGround,
        TimerNum,
    }
    public partial class Form1 : Form
    {
        private AthrillConfig config = new AthrillConfig();
        private AthrillDigital digital = new AthrillDigital();
        private AthrillSerial serial = new AthrillSerial();
        private Timer[] timer;
        private Button[] buttons;

        public Form1()
        {
            InitializeComponent();
            this.timer = new Timer[(int)TimerType.TimerNum];
            for (int i = 0; i < (int)TimerType.TimerNum; i++)
            {
                this.timer[i] = new System.Windows.Forms.Timer();
                this.timer[i].Enabled = true;
                this.timer[i].Interval = 500;
                this.timer[i].Tag = i;
                this.timer[i].Tick += new System.EventHandler(this.timer_Tick);
                this.timer[i].Stop();
            }
            this.buttons = new Button[(int)TimerType.TimerNum];

            this.buttons[(int)TimerType.TimerStartButton] = startButton;
            this.buttons[(int)TimerType.TimerStopButton] = stopButton;
            this.buttons[(int)TimerType.TimerUp1Button] = up1Button;
            this.buttons[(int)TimerType.TimerDown1Button] = down1Button;
            this.buttons[(int)TimerType.TimerUp2Button] = up2Button;
            this.buttons[(int)TimerType.TimerDown2Button] = down2Button;

            this.timer[(int)TimerType.TimerBackGround].Start();
        }

        private void UpdateStatus()
        {
            serial.Parse();
            //TODO AthrillStatus name
            if (serial.GetValue("athrill_up") != null)
            {
                athrillStatus.Text = "ON";
            }
            else
            {
                athrillStatus.Text = "OFF";
            }
            if (serial.GetValue("heat_time") != null)
            {
                heatTime.Text = serial.GetValue("heat_time");
            }
            if (serial.GetValue("heat_method") != null)
            {
                heatMethod.Text = serial.GetValue("heat_method");
            }
            if (serial.GetValue("last_time") != null)
            {
                lastTime.Text = serial.GetValue("last_time");
            }
        }

        private void timer_Tick(object sender, EventArgs e)
        {
            if (config.GetDigitalPath() == null)
            {
                return;
            }

            int index = (int)((Timer)sender).Tag;
            if ( index == (int)TimerType.TimerBackGround)
            {
                UpdateStatus();
                return;
            }
            else
            {
                digital.ClearBit(config.GetDigitalPath(), index);
                this.buttons[index].Enabled = true;
                timer[index].Stop();
                this.buttons[index].Focus();
            }
        }

        private void Button_Click(int index, object sender, EventArgs e)
        {
            if (config.GetDigitalPath() == null)
            {
                MessageBox.Show("microwave フォルダを選択してください", "ERROR", MessageBoxButtons.OK, MessageBoxIcon.Error);
                return;
            }
            digital.SetBit(config.GetDigitalPath(), index);
            this.buttons[index].Enabled = false;
            timer[index].Start();
            this.buttons[index].Focus();
        }
        private void StartButton_Click(object sender, EventArgs e)
        {
            int index = (int)TimerType.TimerStartButton;
            Button_Click(index, sender, e);
        }
        private void StopButton_Click(object sender, EventArgs e)
        {
            int index = (int)TimerType.TimerStopButton;
            Button_Click(index, sender, e);
        }
        private void UP1_Click(object sender, EventArgs e)
        {
            int index = (int)TimerType.TimerUp1Button;
            Button_Click(index, sender, e);
        }

        private void Down1Button_Click(object sender, EventArgs e)
        {
            int index = (int)TimerType.TimerDown1Button;
            Button_Click(index, sender, e);
        }

        private void Up2Button_Click(object sender, EventArgs e)
        {
            int index = (int)TimerType.TimerUp2Button;
            Button_Click(index, sender, e);
        }

        private void Down2Button_Click(object sender, EventArgs e)
        {
            int index = (int)TimerType.TimerDown2Button;
            Button_Click(index, sender, e);
        }


        private void FileSelect_Click(object sender, EventArgs e)
        {
            //FolderBrowserDialogクラスのインスタンスを作成
            FolderBrowserDialog fbd = new FolderBrowserDialog();

            //上部に表示する説明テキストを指定する
            fbd.Description = "microwave フォルダを指定してください。";
            //ルートフォルダを指定する
            //デフォルトでDesktop
            fbd.RootFolder = Environment.SpecialFolder.Desktop;
            //最初に選択するフォルダを指定する
            //RootFolder以下にあるフォルダである必要がある
            if (config.GetFolderPath() != null)
            {
                fbd.SelectedPath = @config.GetFolderPath();
            }
            else
            {
                fbd.SelectedPath = @"C:\Windows";
            }

            //ユーザーが新しいフォルダを作成できるようにする
            //デフォルトでTrue
            fbd.ShowNewFolderButton = true;

            //ダイアログを表示する
            if (fbd.ShowDialog(this) == DialogResult.OK)
            {
                //選択されたフォルダを表示する
                Console.WriteLine(fbd.SelectedPath);
                folderPathTextBox.Text = fbd.SelectedPath;
                config.SetPath(fbd.SelectedPath);
                serial.Open(config.GetSerialPath());

            }
        }
        private void GroupBox3_Enter(object sender, EventArgs e)
        {
        }

        private void FolderPathTextBox_TextChanged(object sender, EventArgs e)
        {
            Console.WriteLine(folderPathTextBox.Text);
            config.SetPath(folderPathTextBox.Text);
            serial.Open(config.GetSerialPath());
        }


    }
}
