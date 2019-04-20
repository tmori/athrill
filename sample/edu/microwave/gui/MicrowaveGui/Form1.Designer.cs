namespace MicrowaveGui
{
    partial class Form1
    {
        /// <summary>
        /// 必要なデザイナー変数です。
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        /// 使用中のリソースをすべてクリーンアップします。
        /// </summary>
        /// <param name="disposing">マネージド リソースを破棄する場合は true を指定し、その他の場合は false を指定します。</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Windows フォーム デザイナーで生成されたコード

        /// <summary>
        /// デザイナー サポートに必要なメソッドです。このメソッドの内容を
        /// コード エディターで変更しないでください。
        /// </summary>
        private void InitializeComponent()
        {
            this.startButton = new System.Windows.Forms.Button();
            this.FileSelect = new System.Windows.Forms.Button();
            this.folderPathTextBox = new System.Windows.Forms.TextBox();
            this.label1 = new System.Windows.Forms.Label();
            this.athrillStatus = new System.Windows.Forms.TextBox();
            this.label2 = new System.Windows.Forms.Label();
            this.label3 = new System.Windows.Forms.Label();
            this.label4 = new System.Windows.Forms.Label();
            this.heatTime = new System.Windows.Forms.TextBox();
            this.heatMethod = new System.Windows.Forms.TextBox();
            this.lastTime = new System.Windows.Forms.TextBox();
            this.groupBox1 = new System.Windows.Forms.GroupBox();
            this.groupBox2 = new System.Windows.Forms.GroupBox();
            this.groupBox3 = new System.Windows.Forms.GroupBox();
            this.groupBox4 = new System.Windows.Forms.GroupBox();
            this.stopButton = new System.Windows.Forms.Button();
            this.up1Button = new System.Windows.Forms.Button();
            this.down1Button = new System.Windows.Forms.Button();
            this.up2Button = new System.Windows.Forms.Button();
            this.down2Button = new System.Windows.Forms.Button();
            this.groupBox1.SuspendLayout();
            this.groupBox3.SuspendLayout();
            this.SuspendLayout();
            // 
            // startButton
            // 
            this.startButton.Location = new System.Drawing.Point(25, 368);
            this.startButton.Name = "startButton";
            this.startButton.Size = new System.Drawing.Size(309, 41);
            this.startButton.TabIndex = 0;
            this.startButton.Text = "START";
            this.startButton.UseVisualStyleBackColor = true;
            this.startButton.Click += new System.EventHandler(this.StartButton_Click);
            // 
            // FileSelect
            // 
            this.FileSelect.Location = new System.Drawing.Point(13, 22);
            this.FileSelect.Name = "FileSelect";
            this.FileSelect.Size = new System.Drawing.Size(81, 23);
            this.FileSelect.TabIndex = 1;
            this.FileSelect.Text = "フォルダ選択";
            this.FileSelect.UseVisualStyleBackColor = true;
            this.FileSelect.Click += new System.EventHandler(this.FileSelect_Click);
            // 
            // folderPathTextBox
            // 
            this.folderPathTextBox.Location = new System.Drawing.Point(108, 24);
            this.folderPathTextBox.Name = "folderPathTextBox";
            this.folderPathTextBox.Size = new System.Drawing.Size(226, 19);
            this.folderPathTextBox.TabIndex = 2;
            this.folderPathTextBox.TextChanged += new System.EventHandler(this.FolderPathTextBox_TextChanged);
            // 
            // label1
            // 
            this.label1.AutoSize = true;
            this.label1.Location = new System.Drawing.Point(19, 58);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(75, 12);
            this.label1.TabIndex = 3;
            this.label1.Text = "Athrill Status:";
            // 
            // athrillStatus
            // 
            this.athrillStatus.Enabled = false;
            this.athrillStatus.Font = new System.Drawing.Font("Impact", 9F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(128)));
            this.athrillStatus.Location = new System.Drawing.Point(108, 55);
            this.athrillStatus.Name = "athrillStatus";
            this.athrillStatus.Size = new System.Drawing.Size(36, 22);
            this.athrillStatus.TabIndex = 4;
            this.athrillStatus.Text = "OFF";
            this.athrillStatus.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            // 
            // label2
            // 
            this.label2.AutoSize = true;
            this.label2.Location = new System.Drawing.Point(23, 98);
            this.label2.Name = "label2";
            this.label2.Size = new System.Drawing.Size(71, 12);
            this.label2.TabIndex = 5;
            this.label2.Text = "温め時間(秒)";
            // 
            // label3
            // 
            this.label3.AutoSize = true;
            this.label3.Location = new System.Drawing.Point(23, 128);
            this.label3.Name = "label3";
            this.label3.Size = new System.Drawing.Size(39, 12);
            this.label3.TabIndex = 6;
            this.label3.Text = "温め方";
            // 
            // label4
            // 
            this.label4.AutoSize = true;
            this.label4.Location = new System.Drawing.Point(23, 162);
            this.label4.Name = "label4";
            this.label4.Size = new System.Drawing.Size(69, 12);
            this.label4.TabIndex = 7;
            this.label4.Text = "残り時間(秒)";
            // 
            // heatTime
            // 
            this.heatTime.Enabled = false;
            this.heatTime.Location = new System.Drawing.Point(108, 95);
            this.heatTime.Name = "heatTime";
            this.heatTime.Size = new System.Drawing.Size(100, 19);
            this.heatTime.TabIndex = 8;
            this.heatTime.Text = "0";
            // 
            // heatMethod
            // 
            this.heatMethod.Enabled = false;
            this.heatMethod.Location = new System.Drawing.Point(108, 125);
            this.heatMethod.Name = "heatMethod";
            this.heatMethod.Size = new System.Drawing.Size(100, 19);
            this.heatMethod.TabIndex = 9;
            this.heatMethod.Text = "slow";
            // 
            // lastTime
            // 
            this.lastTime.Enabled = false;
            this.lastTime.Location = new System.Drawing.Point(108, 159);
            this.lastTime.Name = "lastTime";
            this.lastTime.Size = new System.Drawing.Size(100, 19);
            this.lastTime.TabIndex = 10;
            this.lastTime.Text = "0";
            // 
            // groupBox1
            // 
            this.groupBox1.Controls.Add(this.down1Button);
            this.groupBox1.Controls.Add(this.up1Button);
            this.groupBox1.Controls.Add(this.groupBox2);
            this.groupBox1.Location = new System.Drawing.Point(24, 208);
            this.groupBox1.Name = "groupBox1";
            this.groupBox1.Size = new System.Drawing.Size(144, 107);
            this.groupBox1.TabIndex = 11;
            this.groupBox1.TabStop = false;
            this.groupBox1.Text = "温め時間設定";
            // 
            // groupBox2
            // 
            this.groupBox2.Location = new System.Drawing.Point(166, 3);
            this.groupBox2.Name = "groupBox2";
            this.groupBox2.Size = new System.Drawing.Size(144, 75);
            this.groupBox2.TabIndex = 12;
            this.groupBox2.TabStop = false;
            this.groupBox2.Text = "温め時間設定";
            // 
            // groupBox3
            // 
            this.groupBox3.Controls.Add(this.down2Button);
            this.groupBox3.Controls.Add(this.up2Button);
            this.groupBox3.Controls.Add(this.groupBox4);
            this.groupBox3.Location = new System.Drawing.Point(190, 211);
            this.groupBox3.Name = "groupBox3";
            this.groupBox3.Size = new System.Drawing.Size(144, 104);
            this.groupBox3.TabIndex = 13;
            this.groupBox3.TabStop = false;
            this.groupBox3.Text = "温め方設定";
            this.groupBox3.Enter += new System.EventHandler(this.GroupBox3_Enter);
            // 
            // groupBox4
            // 
            this.groupBox4.Location = new System.Drawing.Point(166, 3);
            this.groupBox4.Name = "groupBox4";
            this.groupBox4.Size = new System.Drawing.Size(144, 75);
            this.groupBox4.TabIndex = 12;
            this.groupBox4.TabStop = false;
            this.groupBox4.Text = "温め時間設定";
            // 
            // stopButton
            // 
            this.stopButton.Location = new System.Drawing.Point(25, 321);
            this.stopButton.Name = "stopButton";
            this.stopButton.Size = new System.Drawing.Size(309, 41);
            this.stopButton.TabIndex = 0;
            this.stopButton.Text = "STOP";
            this.stopButton.UseVisualStyleBackColor = true;
            this.stopButton.Click += new System.EventHandler(this.StopButton_Click);
            // 
            // up1Button
            // 
            this.up1Button.Location = new System.Drawing.Point(6, 18);
            this.up1Button.Name = "up1Button";
            this.up1Button.Size = new System.Drawing.Size(132, 40);
            this.up1Button.TabIndex = 15;
            this.up1Button.Text = "UP";
            this.up1Button.UseVisualStyleBackColor = true;
            this.up1Button.Click += new System.EventHandler(this.UP1_Click);
            // 
            // down1Button
            // 
            this.down1Button.Location = new System.Drawing.Point(6, 64);
            this.down1Button.Name = "down1Button";
            this.down1Button.Size = new System.Drawing.Size(132, 37);
            this.down1Button.TabIndex = 16;
            this.down1Button.Text = "DOWN";
            this.down1Button.UseVisualStyleBackColor = true;
            this.down1Button.Click += new System.EventHandler(this.Down1Button_Click);
            // 
            // up2Button
            // 
            this.up2Button.Location = new System.Drawing.Point(6, 18);
            this.up2Button.Name = "up2Button";
            this.up2Button.Size = new System.Drawing.Size(132, 40);
            this.up2Button.TabIndex = 17;
            this.up2Button.Text = "UP";
            this.up2Button.UseVisualStyleBackColor = true;
            this.up2Button.Click += new System.EventHandler(this.Up2Button_Click);
            // 
            // down2Button
            // 
            this.down2Button.Location = new System.Drawing.Point(6, 61);
            this.down2Button.Name = "down2Button";
            this.down2Button.Size = new System.Drawing.Size(132, 37);
            this.down2Button.TabIndex = 17;
            this.down2Button.Text = "DOWN";
            this.down2Button.UseVisualStyleBackColor = true;
            this.down2Button.Click += new System.EventHandler(this.Down2Button_Click);
            // 
            // Form1
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 12F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(363, 421);
            this.Controls.Add(this.stopButton);
            this.Controls.Add(this.groupBox3);
            this.Controls.Add(this.groupBox1);
            this.Controls.Add(this.lastTime);
            this.Controls.Add(this.heatMethod);
            this.Controls.Add(this.heatTime);
            this.Controls.Add(this.label4);
            this.Controls.Add(this.label3);
            this.Controls.Add(this.label2);
            this.Controls.Add(this.athrillStatus);
            this.Controls.Add(this.label1);
            this.Controls.Add(this.folderPathTextBox);
            this.Controls.Add(this.FileSelect);
            this.Controls.Add(this.startButton);
            this.Name = "Form1";
            this.Text = "Form1";
            this.groupBox1.ResumeLayout(false);
            this.groupBox3.ResumeLayout(false);
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.Button startButton;
        private System.Windows.Forms.Button FileSelect;
        private System.Windows.Forms.TextBox folderPathTextBox;
        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.TextBox athrillStatus;
        private System.Windows.Forms.Label label2;
        private System.Windows.Forms.Label label3;
        private System.Windows.Forms.Label label4;
        private System.Windows.Forms.TextBox heatTime;
        private System.Windows.Forms.TextBox heatMethod;
        private System.Windows.Forms.TextBox lastTime;
        private System.Windows.Forms.GroupBox groupBox1;
        private System.Windows.Forms.GroupBox groupBox2;
        private System.Windows.Forms.GroupBox groupBox3;
        private System.Windows.Forms.GroupBox groupBox4;
        private System.Windows.Forms.Button stopButton;
        private System.Windows.Forms.Button up1Button;
        private System.Windows.Forms.Button down1Button;
        private System.Windows.Forms.Button down2Button;
        private System.Windows.Forms.Button up2Button;
    }
}

