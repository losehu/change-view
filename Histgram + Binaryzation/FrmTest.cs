using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Text;
using System.Windows.Forms;
using System.Drawing.Imaging;
using System.Runtime.InteropServices;
using System.Diagnostics;
namespace BilateralFilter
{
    public unsafe partial class FrmTest : Form
    {
        static class Program
        {
            /// <summary>
            /// 应用程序的主入口点。
            /// </summary>
            [STAThread]
            static void Main()
            {
                Application.EnableVisualStyles();
                Application.SetCompatibleTextRenderingDefault(false);
                Application.Run(new FrmTest());
            }
        }
        private Bitmap SrcBmp;
        private Bitmap DestBmp;
        private Bitmap HistBmp;
        private Bitmap SmoothHistBmp;
        private int[] HistGram = new int[256];
        private int[] HistGramS = new int[256];
        private int Thr;
        bool Init = false;
        public FrmTest()
        {
            InitializeComponent();
        }


        private void CmdOpen_Click(object sender, EventArgs e)
        {
            OpenFileDialog openFileDialog = new OpenFileDialog();
            openFileDialog.RestoreDirectory = true;
            if (openFileDialog.ShowDialog() == DialogResult.OK)
            {
                Bitmap Temp = (Bitmap)Bitmap.FromFile(openFileDialog.FileName);
                if (IsGrayBitmap(Temp) == true)
                    SrcBmp = Temp;
                else
                {
                    SrcBmp = ConvertToGrayBitmap(Temp);
                    Temp.Dispose();
                }
                DestBmp = CreateGrayBitmap(SrcBmp.Width, SrcBmp.Height);
                GetHistGram(SrcBmp, HistGram);
                SrcPic.Image = SrcBmp;
                DestPic.Image = DestBmp;
                Update();
              
            }
            openFileDialog.Dispose();
        }


        private void FrmTest_Load(object sender, EventArgs e)
        {

            CmbMethod.Items.Add("灰度平均值");
            CmbMethod.Items.Add("黄式模糊阈值");
            CmbMethod.Items.Add("谷底最小值");
            CmbMethod.Items.Add("双峰平均值");
            CmbMethod.Items.Add("百分比阈值");
            CmbMethod.Items.Add("迭代阈值法");
            CmbMethod.Items.Add("大津法");
            CmbMethod.Items.Add("一维最大熵");
            CmbMethod.Items.Add("动能保持");
            CmbMethod.Items.Add("Kittler最小错误");
            CmbMethod.Items.Add("ISODATA法");
            CmbMethod.Items.Add("Shanbhag法");
            CmbMethod.Items.Add("Yen法"); 
            CmbMethod.SelectedIndex = 2;
            SrcBmp = global::Binaryzation.Properties.Resources.Lena;
            DestBmp = CreateGrayBitmap(SrcBmp.Width, SrcBmp.Height);
            GetHistGram(SrcBmp, HistGram);
            SrcPic.Image = SrcBmp;
            DestPic.Image = DestBmp;
            HistBmp = CreateGrayBitmap(256, 100);
            SmoothHistBmp = CreateGrayBitmap(256, 100);
            PicHist.Image = HistBmp;
            PicSmoothHist.Image = SmoothHistBmp;
            Update();
            Init = true;
        }

        private Bitmap CreateGrayBitmap(int Width, int Height)
        {
            Bitmap Bmp = new Bitmap(Width, Height, PixelFormat.Format8bppIndexed);
            ColorPalette Pal = Bmp.Palette;
            for (int Y = 0; Y < Pal.Entries.Length; Y++) Pal.Entries[Y] = Color.FromArgb(255, Y, Y, Y);
            Bmp.Palette = Pal;
            return Bmp;
        }

        private bool IsGrayBitmap(Bitmap Bmp)
        {
            if (Bmp.PixelFormat != PixelFormat.Format8bppIndexed) return false;
            if (Bmp.Palette.Entries.Length != 256) return false;
            for (int Y = 0; Y < Bmp.Palette.Entries.Length; Y++)
                if (Bmp.Palette.Entries[Y] != Color.FromArgb(255, Y, Y, Y)) return false;
            return true;
        }

        private Bitmap ConvertToGrayBitmap(Bitmap Src)
        {
            Bitmap Dest = CreateGrayBitmap(Src.Width, Src.Height);
            BitmapData SrcData = Src.LockBits(new Rectangle(0, 0, Src.Width, Src.Height), ImageLockMode.ReadWrite, PixelFormat.Format24bppRgb);
            BitmapData DestData = Dest.LockBits(new Rectangle(0, 0, Dest.Width, Dest.Height), ImageLockMode.ReadWrite, Dest.PixelFormat);
            int Width = SrcData.Width, Height = SrcData.Height;
            int SrcStride = SrcData.Stride, DestStride = DestData.Stride;
            byte* SrcP, DestP;
            for (int Y = 0; Y < Height; Y++)
            {
                SrcP = (byte*)SrcData.Scan0 + Y * SrcStride;         // 必须在某个地方开启unsafe功能，其实C#中的unsafe很safe，搞的好吓人。            
                DestP = (byte*)DestData.Scan0 + Y * DestStride;
                for (int X = 0; X < Width; X++)
                {
                    *DestP = (byte)((*SrcP + (*(SrcP + 1) << 1) + *(SrcP + 2)) >> 2);
                    SrcP += 3;
                    DestP++;
                }
            }
            Src.UnlockBits(SrcData);
            Dest.UnlockBits(DestData);
            return Dest;
        }

        private void GetHistGram(Bitmap Src, int[] HistGram)
        {
            BitmapData SrcData = Src.LockBits(new Rectangle(0, 0, Src.Width, Src.Height), ImageLockMode.ReadWrite, Src.PixelFormat);
            int Width = SrcData.Width, Height = SrcData.Height, SrcStride = SrcData.Stride;
            byte* SrcP;
            for (int Y = 0; Y < 256; Y++) HistGram[Y] = 0;
            for (int Y = 0; Y < Height; Y++)
            {
                SrcP = (byte*)SrcData.Scan0 + Y * SrcStride;
                for (int X = 0; X < Width; X++, SrcP++) HistGram[*SrcP]++;
            }
            Src.UnlockBits(SrcData);
        }

        private void DoBinaryzation(Bitmap Src, Bitmap Dest, int Threshold)
        {
            if (Threshold == -1)
            {
                MessageBox.Show("选择了非法的阈值变量.");
                return;
            }
            BitmapData SrcData = Src.LockBits(new Rectangle(0, 0, Src.Width, Src.Height), ImageLockMode.ReadWrite, Src.PixelFormat);
            BitmapData DestData = Dest.LockBits(new Rectangle(0, 0, Dest.Width, Dest.Height), ImageLockMode.ReadWrite, Dest.PixelFormat);
            int Width = SrcData.Width, Height = SrcData.Height;
            int SrcStride = SrcData.Stride, DestStride = DestData.Stride;
            byte* SrcP, DestP;
            for (int Y = 0; Y < Height; Y++)
            {
                SrcP = (byte*)SrcData.Scan0 + Y * SrcStride;         // 必须在某个地方开启unsafe功能，其实C#中的unsafe很safe，搞的好吓人。            
                DestP = (byte*)DestData.Scan0 + Y * DestStride;
                for (int X = 0; X < Width; X++, SrcP++, DestP++)
                    *DestP = *SrcP > Threshold ? byte.MaxValue : byte.MinValue;     // 写成255和0，C#编译器不认。
            }
            Src.UnlockBits(SrcData);
            Dest.UnlockBits(DestData);
            DestPic.Invalidate();
            LblThreshold.Text = Threshold.ToString();
        }

        private int GetThreshold()
        {
            switch (CmbMethod.SelectedItem.ToString())
            {
                case "灰度平均值":
                    return Threshold.GetMeanThreshold(HistGram);
                case "黄式模糊阈值":
                    return Threshold.GetHuangFuzzyThreshold(HistGram);
                case "谷底最小值":
                    return Threshold.GetMinimumThreshold(HistGram,  HistGramS);
                case "双峰平均值":
                    return Threshold.GetIntermodesThreshold(HistGram,  HistGramS);
                case "百分比阈值":
                    return Threshold.GetPTileThreshold(HistGram);
                case "迭代阈值法":
                    return Threshold.GetIterativeBestThreshold(HistGram);
                case "大津法":
                    return Threshold.GetOSTUThreshold(HistGram);
                case "一维最大熵":
                    return Threshold.Get1DMaxEntropyThreshold(HistGram);
                case "动能保持":
                    return Threshold.GetMomentPreservingThreshold(HistGram);
                case "Kittler最小错误":
                    return Threshold.GetKittlerMinError(HistGram);
                case "ISODATA法":
                    return Threshold.GetIsoDataThreshold(HistGram);
                case "Shanbhag法":
                    return Threshold.GetShanbhagThreshold(HistGram);
                case "Yen法":
                    return Threshold.GetYenThreshold(HistGram);
                default:
                    break;
            }
            return -1;
        }

        public void DrawHistGram(Bitmap SrcBmp,int []Histgram)
        {
            BitmapData HistData = SrcBmp.LockBits(new Rectangle(0, 0, SrcBmp.Width, SrcBmp.Height), ImageLockMode.ReadWrite, SrcBmp.PixelFormat);
            int X, Y, Max = 0;
            byte* P;
            for (Y = 0; Y < 256; Y++) if (Max < Histgram[Y]) Max = Histgram[Y];
            for (X = 0; X < 256; X++)
            {
                P = (byte*)HistData.Scan0 + X;
                for (Y = 0; Y < 100; Y++)
                {
                    if ((100 - Y) > Histgram[X] * 100 / Max)
                        *P = 220;
                    else
                        *P = 0;
                    P += HistData.Stride;
                }
            }

            P = (byte*)HistData.Scan0 + Thr;
            for (Y = 0; Y < 100; Y++)
            {
                *P = 255;
                P += HistData.Stride;
            }
            SrcBmp.UnlockBits(HistData);
        }

        private void CmbMethod_SelectedIndexChanged(object sender, EventArgs e)
        {
            if (Init == true) Update();
        }

        private void Update()
        {
            Thr = GetThreshold();
            DoBinaryzation(SrcBmp, DestBmp, Thr);
            DrawHistGram(HistBmp, HistGram);
            PicHist.Invalidate();
            if (CmbMethod.SelectedItem.ToString() == "谷底最小值" || CmbMethod.SelectedItem.ToString() == "双峰平均值")
            {
                DrawHistGram(SmoothHistBmp,HistGramS);
                PicSmoothHist.Invalidate();
            }
        }

        private void CmdSave_Click(object sender, EventArgs e)
        {
            SaveFileDialog saveFileDialog = new SaveFileDialog();
            saveFileDialog.Filter = "Bitmap files (*.Bitmap)|*.Bmp|Jpeg files (*.jpg)|*.jpg|Png files (*.png)|*.png";
            saveFileDialog.FilterIndex = 4;
            saveFileDialog.RestoreDirectory = true;
            if (saveFileDialog.ShowDialog() == DialogResult.OK)
            {
                DestPic.Image.Save(saveFileDialog.FileName);

            }
        }


    }
}
