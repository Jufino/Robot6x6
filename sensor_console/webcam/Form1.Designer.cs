namespace webcam
{
    partial class Form1
    {
        /// <summary>
        /// Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        /// Clean up any resources being used.
        /// </summary>
        /// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Windows Form Designer generated code

        /// <summary>
        /// Required method for Designer support - do not modify
        /// the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            this.button1 = new System.Windows.Forms.Button();
            this.buttonUp = new System.Windows.Forms.RadioButton();
            this.buttonMiddle = new System.Windows.Forms.RadioButton();
            this.buttonDown = new System.Windows.Forms.RadioButton();
            this.groupBox1 = new System.Windows.Forms.GroupBox();
            this.groupBox2 = new System.Windows.Forms.GroupBox();
            this.zPos = new System.Windows.Forms.TextBox();
            this.yPos = new System.Windows.Forms.TextBox();
            this.xPos = new System.Windows.Forms.TextBox();
            this.groupBox3 = new System.Windows.Forms.GroupBox();
            this.yawPos = new System.Windows.Forms.TextBox();
            this.pitchPos = new System.Windows.Forms.TextBox();
            this.rollPos = new System.Windows.Forms.TextBox();
            this.label1 = new System.Windows.Forms.Label();
            this.label2 = new System.Windows.Forms.Label();
            this.label3 = new System.Windows.Forms.Label();
            this.label4 = new System.Windows.Forms.Label();
            this.label5 = new System.Windows.Forms.Label();
            this.label6 = new System.Windows.Forms.Label();
            this.rollPosDegree = new System.Windows.Forms.TextBox();
            this.pitchPosDegree = new System.Windows.Forms.TextBox();
            this.yawPosDegree = new System.Windows.Forms.TextBox();
            this.groupBox4 = new System.Windows.Forms.GroupBox();
            this.volts = new System.Windows.Forms.TextBox();
            this.capacityPercent = new System.Windows.Forms.TextBox();
            this.groupBox5 = new System.Windows.Forms.GroupBox();
            this.redUp = new System.Windows.Forms.CheckBox();
            this.greenUp = new System.Windows.Forms.CheckBox();
            this.redMiddle = new System.Windows.Forms.CheckBox();
            this.greenMiddle = new System.Windows.Forms.CheckBox();
            this.redDown = new System.Windows.Forms.CheckBox();
            this.greenDown = new System.Windows.Forms.CheckBox();
            this.groupBox6 = new System.Windows.Forms.GroupBox();
            this.redKinect = new System.Windows.Forms.RadioButton();
            this.greenKinect = new System.Windows.Forms.RadioButton();
            this.orangeKinect = new System.Windows.Forms.RadioButton();
            this.blinkRedOrangeKinect = new System.Windows.Forms.RadioButton();
            this.blinkGreenKinect = new System.Windows.Forms.RadioButton();
            this.blinkOrangeKinect = new System.Windows.Forms.RadioButton();
            this.groupBox1.SuspendLayout();
            this.groupBox2.SuspendLayout();
            this.groupBox3.SuspendLayout();
            this.groupBox4.SuspendLayout();
            this.groupBox5.SuspendLayout();
            this.groupBox6.SuspendLayout();
            this.SuspendLayout();
            // 
            // button1
            // 
            this.button1.Location = new System.Drawing.Point(13, 13);
            this.button1.Name = "button1";
            this.button1.Size = new System.Drawing.Size(470, 23);
            this.button1.TabIndex = 0;
            this.button1.Text = "Open connection";
            this.button1.UseVisualStyleBackColor = true;
            this.button1.Click += new System.EventHandler(this.button1_Click);
            // 
            // buttonUp
            // 
            this.buttonUp.AutoCheck = false;
            this.buttonUp.AutoSize = true;
            this.buttonUp.Location = new System.Drawing.Point(22, 19);
            this.buttonUp.Name = "buttonUp";
            this.buttonUp.Size = new System.Drawing.Size(14, 13);
            this.buttonUp.TabIndex = 2;
            this.buttonUp.TabStop = true;
            this.buttonUp.UseVisualStyleBackColor = true;
            // 
            // buttonMiddle
            // 
            this.buttonMiddle.AutoCheck = false;
            this.buttonMiddle.AutoSize = true;
            this.buttonMiddle.Location = new System.Drawing.Point(22, 38);
            this.buttonMiddle.Name = "buttonMiddle";
            this.buttonMiddle.Size = new System.Drawing.Size(14, 13);
            this.buttonMiddle.TabIndex = 2;
            this.buttonMiddle.TabStop = true;
            this.buttonMiddle.UseVisualStyleBackColor = true;
            // 
            // buttonDown
            // 
            this.buttonDown.AutoCheck = false;
            this.buttonDown.AutoSize = true;
            this.buttonDown.Location = new System.Drawing.Point(22, 57);
            this.buttonDown.Name = "buttonDown";
            this.buttonDown.Size = new System.Drawing.Size(14, 13);
            this.buttonDown.TabIndex = 2;
            this.buttonDown.TabStop = true;
            this.buttonDown.UseVisualStyleBackColor = true;
            // 
            // groupBox1
            // 
            this.groupBox1.Controls.Add(this.buttonUp);
            this.groupBox1.Controls.Add(this.buttonDown);
            this.groupBox1.Controls.Add(this.buttonMiddle);
            this.groupBox1.Location = new System.Drawing.Point(13, 148);
            this.groupBox1.Name = "groupBox1";
            this.groupBox1.Size = new System.Drawing.Size(63, 86);
            this.groupBox1.TabIndex = 3;
            this.groupBox1.TabStop = false;
            this.groupBox1.Text = "Buttons";
            // 
            // groupBox2
            // 
            this.groupBox2.Controls.Add(this.zPos);
            this.groupBox2.Controls.Add(this.yPos);
            this.groupBox2.Controls.Add(this.label6);
            this.groupBox2.Controls.Add(this.label5);
            this.groupBox2.Controls.Add(this.label4);
            this.groupBox2.Controls.Add(this.xPos);
            this.groupBox2.Location = new System.Drawing.Point(13, 42);
            this.groupBox2.Name = "groupBox2";
            this.groupBox2.Size = new System.Drawing.Size(139, 100);
            this.groupBox2.TabIndex = 4;
            this.groupBox2.TabStop = false;
            this.groupBox2.Text = "Axis position";
            // 
            // zPos
            // 
            this.zPos.Location = new System.Drawing.Point(28, 71);
            this.zPos.Name = "zPos";
            this.zPos.Size = new System.Drawing.Size(100, 20);
            this.zPos.TabIndex = 0;
            // 
            // yPos
            // 
            this.yPos.Location = new System.Drawing.Point(28, 45);
            this.yPos.Name = "yPos";
            this.yPos.Size = new System.Drawing.Size(100, 20);
            this.yPos.TabIndex = 0;
            // 
            // xPos
            // 
            this.xPos.Location = new System.Drawing.Point(29, 19);
            this.xPos.Name = "xPos";
            this.xPos.Size = new System.Drawing.Size(100, 20);
            this.xPos.TabIndex = 0;
            // 
            // groupBox3
            // 
            this.groupBox3.Controls.Add(this.label3);
            this.groupBox3.Controls.Add(this.label2);
            this.groupBox3.Controls.Add(this.label1);
            this.groupBox3.Controls.Add(this.yawPosDegree);
            this.groupBox3.Controls.Add(this.yawPos);
            this.groupBox3.Controls.Add(this.pitchPosDegree);
            this.groupBox3.Controls.Add(this.pitchPos);
            this.groupBox3.Controls.Add(this.rollPosDegree);
            this.groupBox3.Controls.Add(this.rollPos);
            this.groupBox3.Location = new System.Drawing.Point(158, 42);
            this.groupBox3.Name = "groupBox3";
            this.groupBox3.Size = new System.Drawing.Size(197, 100);
            this.groupBox3.TabIndex = 4;
            this.groupBox3.TabStop = false;
            this.groupBox3.Text = "Angle position";
            // 
            // yawPos
            // 
            this.yawPos.Location = new System.Drawing.Point(41, 71);
            this.yawPos.Name = "yawPos";
            this.yawPos.Size = new System.Drawing.Size(70, 20);
            this.yawPos.TabIndex = 3;
            // 
            // pitchPos
            // 
            this.pitchPos.Location = new System.Drawing.Point(41, 45);
            this.pitchPos.Name = "pitchPos";
            this.pitchPos.Size = new System.Drawing.Size(70, 20);
            this.pitchPos.TabIndex = 2;
            // 
            // rollPos
            // 
            this.rollPos.Location = new System.Drawing.Point(41, 19);
            this.rollPos.Name = "rollPos";
            this.rollPos.Size = new System.Drawing.Size(70, 20);
            this.rollPos.TabIndex = 1;
            // 
            // label1
            // 
            this.label1.AutoSize = true;
            this.label1.Location = new System.Drawing.Point(7, 26);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(28, 13);
            this.label1.TabIndex = 4;
            this.label1.Text = "Roll:";
            // 
            // label2
            // 
            this.label2.AutoSize = true;
            this.label2.Location = new System.Drawing.Point(7, 49);
            this.label2.Name = "label2";
            this.label2.Size = new System.Drawing.Size(34, 13);
            this.label2.TabIndex = 4;
            this.label2.Text = "Pitch:";
            // 
            // label3
            // 
            this.label3.AutoSize = true;
            this.label3.Location = new System.Drawing.Point(7, 75);
            this.label3.Name = "label3";
            this.label3.Size = new System.Drawing.Size(31, 13);
            this.label3.TabIndex = 4;
            this.label3.Text = "Yaw:";
            // 
            // label4
            // 
            this.label4.AutoSize = true;
            this.label4.Location = new System.Drawing.Point(6, 22);
            this.label4.Name = "label4";
            this.label4.Size = new System.Drawing.Size(17, 13);
            this.label4.TabIndex = 4;
            this.label4.Text = "X:";
            // 
            // label5
            // 
            this.label5.AutoSize = true;
            this.label5.Location = new System.Drawing.Point(6, 48);
            this.label5.Name = "label5";
            this.label5.Size = new System.Drawing.Size(17, 13);
            this.label5.TabIndex = 4;
            this.label5.Text = "Y:";
            // 
            // label6
            // 
            this.label6.AutoSize = true;
            this.label6.Location = new System.Drawing.Point(6, 74);
            this.label6.Name = "label6";
            this.label6.Size = new System.Drawing.Size(17, 13);
            this.label6.TabIndex = 4;
            this.label6.Text = "Z:";
            // 
            // rollPosDegree
            // 
            this.rollPosDegree.Location = new System.Drawing.Point(117, 19);
            this.rollPosDegree.Name = "rollPosDegree";
            this.rollPosDegree.Size = new System.Drawing.Size(70, 20);
            this.rollPosDegree.TabIndex = 1;
            // 
            // pitchPosDegree
            // 
            this.pitchPosDegree.Location = new System.Drawing.Point(117, 46);
            this.pitchPosDegree.Name = "pitchPosDegree";
            this.pitchPosDegree.Size = new System.Drawing.Size(70, 20);
            this.pitchPosDegree.TabIndex = 2;
            // 
            // yawPosDegree
            // 
            this.yawPosDegree.Location = new System.Drawing.Point(117, 71);
            this.yawPosDegree.Name = "yawPosDegree";
            this.yawPosDegree.Size = new System.Drawing.Size(70, 20);
            this.yawPosDegree.TabIndex = 3;
            // 
            // groupBox4
            // 
            this.groupBox4.Controls.Add(this.capacityPercent);
            this.groupBox4.Controls.Add(this.volts);
            this.groupBox4.Location = new System.Drawing.Point(361, 42);
            this.groupBox4.Name = "groupBox4";
            this.groupBox4.Size = new System.Drawing.Size(122, 70);
            this.groupBox4.TabIndex = 5;
            this.groupBox4.TabStop = false;
            this.groupBox4.Text = "Voltage";
            // 
            // volts
            // 
            this.volts.Location = new System.Drawing.Point(7, 20);
            this.volts.Name = "volts";
            this.volts.Size = new System.Drawing.Size(100, 20);
            this.volts.TabIndex = 0;
            // 
            // capacityPercent
            // 
            this.capacityPercent.Location = new System.Drawing.Point(6, 45);
            this.capacityPercent.Name = "capacityPercent";
            this.capacityPercent.Size = new System.Drawing.Size(100, 20);
            this.capacityPercent.TabIndex = 0;
            // 
            // groupBox5
            // 
            this.groupBox5.Controls.Add(this.greenDown);
            this.groupBox5.Controls.Add(this.greenMiddle);
            this.groupBox5.Controls.Add(this.greenUp);
            this.groupBox5.Controls.Add(this.redDown);
            this.groupBox5.Controls.Add(this.redMiddle);
            this.groupBox5.Controls.Add(this.redUp);
            this.groupBox5.Location = new System.Drawing.Point(82, 148);
            this.groupBox5.Name = "groupBox5";
            this.groupBox5.Size = new System.Drawing.Size(122, 100);
            this.groupBox5.TabIndex = 6;
            this.groupBox5.TabStop = false;
            this.groupBox5.Text = "Leds";
            // 
            // redUp
            // 
            this.redUp.AutoSize = true;
            this.redUp.Location = new System.Drawing.Point(7, 21);
            this.redUp.Name = "redUp";
            this.redUp.Size = new System.Drawing.Size(46, 17);
            this.redUp.TabIndex = 0;
            this.redUp.Text = "Red";
            this.redUp.UseVisualStyleBackColor = true;
            // 
            // greenUp
            // 
            this.greenUp.AutoSize = true;
            this.greenUp.Location = new System.Drawing.Point(59, 21);
            this.greenUp.Name = "greenUp";
            this.greenUp.Size = new System.Drawing.Size(55, 17);
            this.greenUp.TabIndex = 0;
            this.greenUp.Text = "Green";
            this.greenUp.UseVisualStyleBackColor = true;
            // 
            // redMiddle
            // 
            this.redMiddle.AutoSize = true;
            this.redMiddle.Location = new System.Drawing.Point(6, 45);
            this.redMiddle.Name = "redMiddle";
            this.redMiddle.Size = new System.Drawing.Size(46, 17);
            this.redMiddle.TabIndex = 0;
            this.redMiddle.Text = "Red";
            this.redMiddle.UseVisualStyleBackColor = true;
            // 
            // greenMiddle
            // 
            this.greenMiddle.AutoSize = true;
            this.greenMiddle.Location = new System.Drawing.Point(58, 45);
            this.greenMiddle.Name = "greenMiddle";
            this.greenMiddle.Size = new System.Drawing.Size(55, 17);
            this.greenMiddle.TabIndex = 0;
            this.greenMiddle.Text = "Green";
            this.greenMiddle.UseVisualStyleBackColor = true;
            // 
            // redDown
            // 
            this.redDown.AutoSize = true;
            this.redDown.Location = new System.Drawing.Point(6, 68);
            this.redDown.Name = "redDown";
            this.redDown.Size = new System.Drawing.Size(46, 17);
            this.redDown.TabIndex = 0;
            this.redDown.Text = "Red";
            this.redDown.UseVisualStyleBackColor = true;
            // 
            // greenDown
            // 
            this.greenDown.AutoSize = true;
            this.greenDown.Location = new System.Drawing.Point(58, 68);
            this.greenDown.Name = "greenDown";
            this.greenDown.Size = new System.Drawing.Size(55, 17);
            this.greenDown.TabIndex = 0;
            this.greenDown.Text = "Green";
            this.greenDown.UseVisualStyleBackColor = true;
            // 
            // groupBox6
            // 
            this.groupBox6.Controls.Add(this.blinkOrangeKinect);
            this.groupBox6.Controls.Add(this.blinkGreenKinect);
            this.groupBox6.Controls.Add(this.blinkRedOrangeKinect);
            this.groupBox6.Controls.Add(this.orangeKinect);
            this.groupBox6.Controls.Add(this.greenKinect);
            this.groupBox6.Controls.Add(this.redKinect);
            this.groupBox6.Location = new System.Drawing.Point(210, 148);
            this.groupBox6.Name = "groupBox6";
            this.groupBox6.Size = new System.Drawing.Size(115, 166);
            this.groupBox6.TabIndex = 7;
            this.groupBox6.TabStop = false;
            this.groupBox6.Text = "Led Kinect";
            // 
            // redKinect
            // 
            this.redKinect.AutoSize = true;
            this.redKinect.Location = new System.Drawing.Point(6, 20);
            this.redKinect.Name = "redKinect";
            this.redKinect.Size = new System.Drawing.Size(45, 17);
            this.redKinect.TabIndex = 0;
            this.redKinect.TabStop = true;
            this.redKinect.Text = "Red";
            this.redKinect.UseVisualStyleBackColor = true;
            // 
            // greenKinect
            // 
            this.greenKinect.AutoSize = true;
            this.greenKinect.Location = new System.Drawing.Point(6, 43);
            this.greenKinect.Name = "greenKinect";
            this.greenKinect.Size = new System.Drawing.Size(54, 17);
            this.greenKinect.TabIndex = 0;
            this.greenKinect.TabStop = true;
            this.greenKinect.Text = "Green";
            this.greenKinect.UseVisualStyleBackColor = true;
            // 
            // orangeKinect
            // 
            this.orangeKinect.AutoSize = true;
            this.orangeKinect.Location = new System.Drawing.Point(6, 66);
            this.orangeKinect.Name = "orangeKinect";
            this.orangeKinect.Size = new System.Drawing.Size(60, 17);
            this.orangeKinect.TabIndex = 0;
            this.orangeKinect.TabStop = true;
            this.orangeKinect.Text = "Orange";
            this.orangeKinect.UseVisualStyleBackColor = true;
            // 
            // blinkRedOrangeKinect
            // 
            this.blinkRedOrangeKinect.AutoSize = true;
            this.blinkRedOrangeKinect.Location = new System.Drawing.Point(6, 89);
            this.blinkRedOrangeKinect.Name = "blinkRedOrangeKinect";
            this.blinkRedOrangeKinect.Size = new System.Drawing.Size(105, 17);
            this.blinkRedOrangeKinect.TabIndex = 0;
            this.blinkRedOrangeKinect.TabStop = true;
            this.blinkRedOrangeKinect.Text = "Blink red, orange";
            this.blinkRedOrangeKinect.UseVisualStyleBackColor = true;
            // 
            // blinkGreenKinect
            // 
            this.blinkGreenKinect.AutoSize = true;
            this.blinkGreenKinect.Location = new System.Drawing.Point(6, 112);
            this.blinkGreenKinect.Name = "blinkGreenKinect";
            this.blinkGreenKinect.Size = new System.Drawing.Size(78, 17);
            this.blinkGreenKinect.TabIndex = 0;
            this.blinkGreenKinect.TabStop = true;
            this.blinkGreenKinect.Text = "Blink green";
            this.blinkGreenKinect.UseVisualStyleBackColor = true;
            // 
            // blinkOrangeKinect
            // 
            this.blinkOrangeKinect.AutoSize = true;
            this.blinkOrangeKinect.Location = new System.Drawing.Point(6, 135);
            this.blinkOrangeKinect.Name = "blinkOrangeKinect";
            this.blinkOrangeKinect.Size = new System.Drawing.Size(84, 17);
            this.blinkOrangeKinect.TabIndex = 0;
            this.blinkOrangeKinect.TabStop = true;
            this.blinkOrangeKinect.Text = "Blink orange";
            this.blinkOrangeKinect.UseVisualStyleBackColor = true;
            // 
            // Form1
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(492, 323);
            this.Controls.Add(this.groupBox6);
            this.Controls.Add(this.groupBox5);
            this.Controls.Add(this.groupBox4);
            this.Controls.Add(this.groupBox3);
            this.Controls.Add(this.groupBox2);
            this.Controls.Add(this.groupBox1);
            this.Controls.Add(this.button1);
            this.Name = "Form1";
            this.Text = "Sensor connection";
            this.FormClosing += new System.Windows.Forms.FormClosingEventHandler(this.Form1_FormClosing);
            this.groupBox1.ResumeLayout(false);
            this.groupBox1.PerformLayout();
            this.groupBox2.ResumeLayout(false);
            this.groupBox2.PerformLayout();
            this.groupBox3.ResumeLayout(false);
            this.groupBox3.PerformLayout();
            this.groupBox4.ResumeLayout(false);
            this.groupBox4.PerformLayout();
            this.groupBox5.ResumeLayout(false);
            this.groupBox5.PerformLayout();
            this.groupBox6.ResumeLayout(false);
            this.groupBox6.PerformLayout();
            this.ResumeLayout(false);

        }

        #endregion

        private System.Windows.Forms.Button button1;
        private System.Windows.Forms.RadioButton buttonUp;
        private System.Windows.Forms.RadioButton buttonMiddle;
        private System.Windows.Forms.RadioButton buttonDown;
        private System.Windows.Forms.GroupBox groupBox1;
        private System.Windows.Forms.GroupBox groupBox2;
        private System.Windows.Forms.TextBox zPos;
        private System.Windows.Forms.TextBox yPos;
        private System.Windows.Forms.TextBox xPos;
        private System.Windows.Forms.GroupBox groupBox3;
        private System.Windows.Forms.TextBox yawPos;
        private System.Windows.Forms.TextBox pitchPos;
        private System.Windows.Forms.TextBox rollPos;
        private System.Windows.Forms.Label label6;
        private System.Windows.Forms.Label label5;
        private System.Windows.Forms.Label label4;
        private System.Windows.Forms.Label label3;
        private System.Windows.Forms.Label label2;
        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.TextBox yawPosDegree;
        private System.Windows.Forms.TextBox pitchPosDegree;
        private System.Windows.Forms.TextBox rollPosDegree;
        private System.Windows.Forms.GroupBox groupBox4;
        private System.Windows.Forms.TextBox capacityPercent;
        private System.Windows.Forms.TextBox volts;
        private System.Windows.Forms.GroupBox groupBox5;
        private System.Windows.Forms.CheckBox greenDown;
        private System.Windows.Forms.CheckBox greenMiddle;
        private System.Windows.Forms.CheckBox greenUp;
        private System.Windows.Forms.CheckBox redDown;
        private System.Windows.Forms.CheckBox redMiddle;
        private System.Windows.Forms.CheckBox redUp;
        private System.Windows.Forms.GroupBox groupBox6;
        private System.Windows.Forms.RadioButton blinkOrangeKinect;
        private System.Windows.Forms.RadioButton blinkGreenKinect;
        private System.Windows.Forms.RadioButton blinkRedOrangeKinect;
        private System.Windows.Forms.RadioButton orangeKinect;
        private System.Windows.Forms.RadioButton greenKinect;
        private System.Windows.Forms.RadioButton redKinect;
    }
}

