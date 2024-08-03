namespace HID_terminal
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
            this.textBoxSend = new System.Windows.Forms.TextBox();
            this.button_send = new System.Windows.Forms.Button();
            this.textBoxRecieved = new System.Windows.Forms.TextBox();
            this.label1 = new System.Windows.Forms.Label();
            this.label2 = new System.Windows.Forms.Label();
            this.SuspendLayout();
            // 
            // textBoxSend
            // 
            this.textBoxSend.Location = new System.Drawing.Point(33, 126);
            this.textBoxSend.Name = "textBoxSend";
            this.textBoxSend.Size = new System.Drawing.Size(122, 20);
            this.textBoxSend.TabIndex = 0;
            // 
            // button_send
            // 
            this.button_send.Location = new System.Drawing.Point(33, 161);
            this.button_send.Name = "button_send";
            this.button_send.Size = new System.Drawing.Size(75, 23);
            this.button_send.TabIndex = 2;
            this.button_send.Text = "Send";
            this.button_send.UseVisualStyleBackColor = true;
            this.button_send.Click += new System.EventHandler(this.button_send_Click);
            // 
            // textBoxRecieved
            // 
            this.textBoxRecieved.Location = new System.Drawing.Point(209, 80);
            this.textBoxRecieved.Multiline = true;
            this.textBoxRecieved.Name = "textBoxRecieved";
            this.textBoxRecieved.Size = new System.Drawing.Size(221, 238);
            this.textBoxRecieved.TabIndex = 3;
            // 
            // label1
            // 
            this.label1.AutoSize = true;
            this.label1.Location = new System.Drawing.Point(30, 98);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(56, 13);
            this.label1.TabIndex = 4;
            this.label1.Text = "Data send";
            // 
            // label2
            // 
            this.label2.AutoSize = true;
            this.label2.Location = new System.Drawing.Point(206, 60);
            this.label2.Name = "label2";
            this.label2.Size = new System.Drawing.Size(77, 13);
            this.label2.TabIndex = 5;
            this.label2.Text = "Received data";
            // 
            // Form1
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(464, 404);
            this.Controls.Add(this.label2);
            this.Controls.Add(this.label1);
            this.Controls.Add(this.textBoxRecieved);
            this.Controls.Add(this.button_send);
            this.Controls.Add(this.textBoxSend);
            this.Name = "Form1";
            this.Text = "HID terminal";
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.TextBox textBoxSend;
        private System.Windows.Forms.Button button_send;
        private System.Windows.Forms.TextBox textBoxRecieved;
        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.Label label2;
    }
}

