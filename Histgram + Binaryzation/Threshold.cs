using System;
using System.Collections.Generic;
using System.Windows.Forms;

public static class Threshold
{
    /// <summary>
    /// 基于灰度平均值的阈值
    /// </summary>
    /// <param name="HistGram">灰度图像的直方图</param>
    /// <returns></returns>
    public static int GetMeanThreshold(int[] HistGram)
    {
        int Sum = 0, Amount = 0;
        for (int Y = 0; Y < 256; Y++)
        {
            Amount += HistGram[Y];
            Sum += Y * HistGram[Y];
        }
        return Sum / Amount;
    }
    /// <summary>
    /// 基于模糊集的黄式阈值算法
    /// http://www.ktl.elf.stuba.sk/study/vacso/Zadania-Cvicenia/Cvicenie_3/TimA2/Huang_E016529624.pdf
    /// </summary>
    /// <param name="HistGram">灰度图像的直方图</param>
    /// <returns></returns>

    public static int GetHuangFuzzyThreshold(int[] HistGram)
    {
        int X, Y;
        int First, Last;
        int Threshold = -1;
        double BestEntropy = Double.MaxValue, Entropy;
        //   找到第一个和最后一个非0的色阶值
        for (First = 0; First < HistGram.Length && HistGram[First] == 0; First++) ;
        for (Last = HistGram.Length - 1; Last > First && HistGram[Last] == 0; Last--) ;
        if (First == Last) return First;                // 图像中只有一个颜色
        if (First + 1 == Last) return First;            // 图像中只有二个颜色

        // 计算累计直方图以及对应的带权重的累计直方图
        int[] S = new int[Last + 1];
        int[] W = new int[Last + 1];            // 对于特大图，此数组的保存数据可能会超出int的表示范围，可以考虑用long类型来代替
        S[0] = HistGram[0];
        for (Y = First > 1 ? First : 1; Y <= Last; Y++)
        {
            S[Y] = S[Y - 1] + HistGram[Y];
            W[Y] = W[Y - 1] + Y * HistGram[Y];
        }

        // 建立公式（4）及（6）所用的查找表
        double[] Smu = new double[Last + 1 - First];
        for (Y = 1; Y < Smu.Length; Y++)
        {
            double mu = 1 / (1 + (double)Y / (Last - First));               // 公式（4）
            Smu[Y] = -mu * Math.Log(mu) - (1 - mu) * Math.Log(1 - mu);      // 公式（6）
        }

        // 迭代计算最佳阈值
        for (Y = First; Y <= Last; Y++)
        {
            Entropy = 0;
            int mu = (int)Math.Round((double)W[Y] / S[Y]);             // 公式17
            for (X = First; X <= Y; X++)
                Entropy += Smu[Math.Abs(X - mu)] * HistGram[X];
            mu = (int)Math.Round((double)(W[Last] - W[Y]) / (S[Last] - S[Y]));  // 公式18       
            for (X = Y + 1; X <= Last; X++)
                Entropy += Smu[Math.Abs(X - mu)] * HistGram[X];       // 公式8
            if (BestEntropy > Entropy)
            {
                BestEntropy = Entropy;      // 取最小熵处为最佳阈值
                Threshold = Y;
            }
        }
        return Threshold;
    }


    /// <summary>
    /// 基于谷底最小值的阈值
    /// 此方法实用于具有明显双峰直方图的图像，其寻找双峰的谷底作为阈值
    /// References: 
    /// J. M. S. Prewitt and M. L. Mendelsohn, "The analysis of cell images," in
    /// nnals of the New York Academy of Sciences, vol. 128, pp. 1035-1053, 1966.
    /// C. A. Glasbey, "An analysis of histogram-based thresholding algorithms,"
    /// CVGIP: Graphical Models and Image Processing, vol. 55, pp. 532-537, 1993.
    /// </summary>
    /// <param name="HistGram">灰度图像的直方图</param>
    /// <param name="HistGramS">返回平滑后的直方图</param>
    /// <returns></returns>
    public static int GetMinimumThreshold(int[] HistGram, int[] HistGramS)
    {
        int Y, Iter = 0;
        double[] HistGramC = new double[256];           // 基于精度问题，一定要用浮点数来处理，否则得不到正确的结果
        double[] HistGramCC = new double[256];          // 求均值的过程会破坏前面的数据，因此需要两份数据
        for (Y = 0; Y < 256; Y++)
        {
            HistGramC[Y] = HistGram[Y];
            HistGramCC[Y] = HistGram[Y];
        }

        // 通过三点求均值来平滑直方图
        while (IsDimodal(HistGramCC) == false)                                        // 判断是否已经是双峰的图像了      
        {
            HistGramCC[0] = (HistGramC[0] + HistGramC[0] + HistGramC[1]) / 3;                 // 第一点
            for (Y = 1; Y < 255; Y++)
                HistGramCC[Y] = (HistGramC[Y - 1] + HistGramC[Y] + HistGramC[Y + 1]) / 3;     // 中间的点
            HistGramCC[255] = (HistGramC[254] + HistGramC[255] + HistGramC[255]) / 3;         // 最后一点
            System.Buffer.BlockCopy(HistGramCC, 0, HistGramC, 0, 256 * sizeof(double));
            Iter++;
            if (Iter >= 1000) return -1;                                                   // 直方图无法平滑为双峰的，返回错误代码
        }
        for (Y = 0; Y < 256; Y++) HistGramS[Y] = (int)HistGramCC[Y];
        // 阈值极为两峰之间的最小值 
        bool Peakfound = false;
        for (Y = 1; Y < 255; Y++)
        {
            if (HistGramCC[Y - 1] < HistGramCC[Y] && HistGramCC[Y + 1] < HistGramCC[Y]) Peakfound = true;
            if (Peakfound == true && HistGramCC[Y - 1] >= HistGramCC[Y] && HistGramCC[Y + 1] >= HistGramCC[Y])
                return Y - 1;
        }
        return -1;
    }

    /// <summary>
    /// 基于双峰平均值的阈值
    /// 此方法实用于具有明显双峰直方图的图像，其寻找双峰的谷底作为阈值
    /// References: 
    /// J. M. S. Prewitt and M. L. Mendelsohn, "The analysis of cell images," in
    /// nnals of the New York Academy of Sciences, vol. 128, pp. 1035-1053, 1966.
    /// C. A. Glasbey, "An analysis of histogram-based thresholding algorithms,"
    /// CVGIP: Graphical Models and Image Processing, vol. 55, pp. 532-537, 1993.
    /// </summary>
    /// <param name="HistGram">灰度图像的直方图</param>
    /// <param name="HistGramS">返回平滑后的直方图</param>
    /// <returns></returns>

    public static int GetIntermodesThreshold(int[] HistGram, int[] HistGramS)
    {
        int Y, Iter = 0, Index;
        double[] HistGramC = new double[256];           // 基于精度问题，一定要用浮点数来处理，否则得不到正确的结果
        double[] HistGramCC = new double[256];          // 求均值的过程会破坏前面的数据，因此需要两份数据
        for (Y = 0; Y < 256; Y++)
        {
            HistGramC[Y] = HistGram[Y];
            HistGramCC[Y] = HistGram[Y];
        }
        // 通过三点求均值来平滑直方图
        while (IsDimodal(HistGramCC) == false)                                                  // 判断是否已经是双峰的图像了      
        {
            HistGramCC[0] = (HistGramC[0] + HistGramC[0] + HistGramC[1]) / 3;                   // 第一点
            for (Y = 1; Y < 255; Y++)
                HistGramCC[Y] = (HistGramC[Y - 1] + HistGramC[Y] + HistGramC[Y + 1]) / 3;       // 中间的点
            HistGramCC[255] = (HistGramC[254] + HistGramC[255] + HistGramC[255]) / 3;           // 最后一点
            System.Buffer.BlockCopy(HistGramCC, 0, HistGramC, 0, 256 * sizeof(double));         // 备份数据，为下一次迭代做准备
            Iter++;
            if (Iter >= 10000) return -1;                                                       // 似乎直方图无法平滑为双峰的，返回错误代码
        }
        for (Y = 0; Y < 256; Y++) HistGramS[Y] = (int)HistGramCC[Y];
        // 阈值为两峰值的平均值
        int[] Peak = new int[2];
        for (Y = 1, Index = 0; Y < 255; Y++)
            if (HistGramCC[Y - 1] < HistGramCC[Y] && HistGramCC[Y + 1] < HistGramCC[Y]) Peak[Index++] = Y - 1;
        return ((Peak[0] + Peak[1]) / 2);
    }
    /// <summary>
    /// 百分比阈值
    /// </summary>
    /// <param name="HistGram">灰度图像的直方图</param>
    /// <param name="Tile">背景在图像中所占的面积百分比</param>
    /// <returns></returns>
    public static int GetPTileThreshold(int[] HistGram, int Tile = 50)
    {
        int Y, Amount = 0, Sum = 0;
        for (Y = 0; Y < 256; Y++) Amount += HistGram[Y];        //  像素总数
        for (Y = 0; Y < 256; Y++)
        {
            Sum = Sum + HistGram[Y];
            if (Sum >= Amount * Tile / 100) return Y;
        }
        return -1;
    }

    /// <summary>
    /// 迭代法获得阈值
    /// </summary>
    /// <param name="HistGram">灰度图像的直方图</param>
    /// <returns></returns>
    public static int GetIterativeBestThreshold(int[] HistGram)
    {
        int X, Iter = 0;
        int MeanValueOne, MeanValueTwo, SumOne, SumTwo, SumIntegralOne, SumIntegralTwo;
        int MinValue, MaxValue;
        int Threshold, NewThreshold;

        for (MinValue = 0; MinValue < 256 && HistGram[MinValue] == 0; MinValue++) ;
        for (MaxValue = 255; MaxValue > MinValue && HistGram[MinValue] == 0; MaxValue--) ;

        if (MaxValue == MinValue) return MaxValue;          // 图像中只有一个颜色             
        if (MinValue + 1 == MaxValue) return MinValue;      // 图像中只有二个颜色

        Threshold = MinValue;
        NewThreshold = (MaxValue + MinValue) >> 1;
        while (Threshold != NewThreshold)    // 当前后两次迭代的获得阈值相同时，结束迭代    
        {
            SumOne = 0; SumIntegralOne = 0;
            SumTwo = 0; SumIntegralTwo = 0;
            Threshold = NewThreshold;
            for (X = MinValue; X <= Threshold; X++)         //根据阈值将图像分割成目标和背景两部分，求出两部分的平均灰度值      
            {
                SumIntegralOne += HistGram[X] * X;
                SumOne += HistGram[X];
            }
            MeanValueOne = SumIntegralOne / SumOne;
            for (X = Threshold + 1; X <= MaxValue; X++)
            {
                SumIntegralTwo += HistGram[X] * X;
                SumTwo += HistGram[X];
            }
            MeanValueTwo = SumIntegralTwo / SumTwo;
            NewThreshold = (MeanValueOne + MeanValueTwo) >> 1;       //求出新的阈值
            Iter++;
            if (Iter >= 1000) return -1;
        }
        return Threshold;
    }


    public static int GetOSTUThreshold(int[] HistGram)
    {
        int X, Y, Amount = 0;
        int PixelBack = 0, PixelFore = 0, PixelIntegralBack = 0, PixelIntegralFore = 0, PixelIntegral = 0;
        double OmegaBack, OmegaFore, MicroBack, MicroFore, SigmaB, Sigma;              // 类间方差;
        int MinValue, MaxValue;
        int Threshold = 0;

        for (MinValue = 0; MinValue < 256 && HistGram[MinValue] == 0; MinValue++) ;
        for (MaxValue = 255; MaxValue > MinValue && HistGram[MinValue] == 0; MaxValue--) ;
        if (MaxValue == MinValue) return MaxValue;          // 图像中只有一个颜色             
        if (MinValue + 1 == MaxValue) return MinValue;      // 图像中只有二个颜色

        for (Y = MinValue; Y <= MaxValue; Y++) Amount += HistGram[Y];        //  像素总数

        PixelIntegral = 0;
        for (Y = MinValue; Y <= MaxValue; Y++) PixelIntegral += HistGram[Y] * Y;
        SigmaB = -1;
        for (Y = MinValue; Y < MaxValue; Y++)
        {
            PixelBack = PixelBack + HistGram[Y];
            PixelFore = Amount - PixelBack;
            OmegaBack = (double)PixelBack / Amount;
            OmegaFore = (double)PixelFore / Amount;
            PixelIntegralBack += HistGram[Y] * Y;
            PixelIntegralFore = PixelIntegral - PixelIntegralBack;
            MicroBack = (double)PixelIntegralBack / PixelBack;
            MicroFore = (double)PixelIntegralFore / PixelFore;
            Sigma = OmegaBack * OmegaFore * (MicroBack - MicroFore) * (MicroBack - MicroFore);
            if (Sigma > SigmaB)
            {
                SigmaB = Sigma;
                Threshold = Y;
            }
        }
        return Threshold;
    }

    public static int Get1DMaxEntropyThreshold(int[] HistGram)
    {
        int X, Y, Amount = 0;
        double[] HistGramD = new double[256];
        double SumIntegral, EntropyBack, EntropyFore, MaxEntropy;
        int MinValue = 255, MaxValue = 0;
        int Threshold = 0;

        for (MinValue = 0; MinValue < 256 && HistGram[MinValue] == 0; MinValue++) ;
        for (MaxValue = 255; MaxValue > MinValue && HistGram[MinValue] == 0; MaxValue--) ;
        if (MaxValue == MinValue) return MaxValue;          // 图像中只有一个颜色             
        if (MinValue + 1 == MaxValue) return MinValue;      // 图像中只有二个颜色

        for (Y = MinValue; Y <= MaxValue; Y++) Amount += HistGram[Y];        //  像素总数

        for (Y = MinValue; Y <= MaxValue; Y++) HistGramD[Y] = (double)HistGram[Y] / Amount + 1e-17;

        MaxEntropy = double.MinValue; ;
        for (Y = MinValue + 1; Y < MaxValue; Y++)
        {
            SumIntegral = 0;
            for (X = MinValue; X <= Y; X++) SumIntegral += HistGramD[X];
            EntropyBack = 0;
            for (X = MinValue; X <= Y; X++) EntropyBack += (-HistGramD[X] / SumIntegral * Math.Log(HistGramD[X] / SumIntegral));
            EntropyFore = 0;
            for (X = Y + 1; X <= MaxValue; X++) EntropyFore += (-HistGramD[X] / (1 - SumIntegral) * Math.Log(HistGramD[X] / (1 - SumIntegral)));
            if (MaxEntropy < EntropyBack + EntropyFore)
            {
                Threshold = Y;
                MaxEntropy = EntropyBack + EntropyFore;
            }
        }
        return Threshold;
    }

    // http://fiji.sc/wiki/index.php/Auto_Threshold#Huang
    //   W. Tsai, "Moment-preserving thresholding: a new approach," Computer
    //   Vision, Graphics, and Image Processing, vol. 29, pp. 377-393, 1985.
    //
    //  C. A. Glasbey, "An analysis of histogram-based thresholding algorithms,"
    //  CVGIP: Graphical Models and Image Processing, vol. 55, pp. 532-537, 1993.

    public static byte GetMomentPreservingThreshold(int[] HistGram)
    {
        int X, Y, Index = 0, Amount = 0;
        double[] Avec = new double[256];
        double X2, X1, X0, Min;

        for (Y = 0; Y <= 255; Y++) Amount += HistGram[Y];        //  像素总数
        for (Y = 0; Y < 256; Y++) Avec[Y] = (double)A(HistGram, Y) / Amount;       // The threshold is chosen such that A(y,t)/A(y,n) is closest to x0.

        // The following finds x0.

        X2 = (double)(B(HistGram, 255) * C(HistGram, 255) - A(HistGram, 255) * D(HistGram, 255)) / (double)(A(HistGram, 255) * C(HistGram, 255) - B(HistGram, 255) * B(HistGram, 255));
        X1 = (double)(B(HistGram, 255) * D(HistGram, 255) - C(HistGram, 255) * C(HistGram, 255)) / (double)(A(HistGram, 255) * C(HistGram, 255) - B(HistGram, 255) * B(HistGram, 255));
        X0 = 0.5 - (B(HistGram, 255) / A(HistGram, 255) + X2 / 2) / Math.Sqrt(X2 * X2 - 4 * X1);

        for (Y = 0, Min = double.MaxValue; Y < 256; Y++)
        {
            if (Math.Abs(Avec[Y] - X0) < Min)
            {
                Min = Math.Abs(Avec[Y] - X0);
                Index = Y;
            }
        }
        return (byte)Index;
    }

    public static int GetKittlerMinError(int[] HistGram)
    {
        int X, Y;
        int MinValue, MaxValue;
        int Threshold ;
        int PixelBack, PixelFore;
        double OmegaBack, OmegaFore, MinSigma, Sigma, SigmaBack, SigmaFore;
        for (MinValue = 0; MinValue < 256 && HistGram[MinValue] == 0; MinValue++) ;
        for (MaxValue = 255; MaxValue > MinValue && HistGram[MinValue] == 0; MaxValue--) ;
        if (MaxValue == MinValue) return MaxValue;          // 图像中只有一个颜色             
        if (MinValue + 1 == MaxValue) return MinValue;      // 图像中只有二个颜色
        Threshold = -1;
        MinSigma = 1E+20;
        for (Y = MinValue; Y < MaxValue; Y++)
        {
            PixelBack = 0; PixelFore = 0;
            OmegaBack = 0; OmegaFore = 0;
            for (X = MinValue; X <= Y; X++)
            {
                PixelBack += HistGram[X];
                OmegaBack = OmegaBack + X * HistGram[X];
            }
            for (X = Y + 1; X <= MaxValue; X++)
            {
                PixelFore += HistGram[X];
                OmegaFore = OmegaFore + X * HistGram[X];
            }
            OmegaBack = OmegaBack / PixelBack;
            OmegaFore = OmegaFore / PixelFore;
            SigmaBack = 0; SigmaFore = 0;
            for (X = MinValue; X <= Y; X++) SigmaBack = SigmaBack + (X - OmegaBack) * (X - OmegaBack) * HistGram[X];
            for (X = Y + 1; X <= MaxValue; X++) SigmaFore = SigmaFore + (X - OmegaFore) * (X - OmegaFore) * HistGram[X];
            if (SigmaBack == 0 || SigmaFore == 0)
            {
                if (Threshold == -1)
                    Threshold = Y;
            }
            else
            {
                SigmaBack = Math.Sqrt(SigmaBack / PixelBack);
                SigmaFore = Math.Sqrt(SigmaFore / PixelFore);
                Sigma = 1 + 2 * (PixelBack * Math.Log(SigmaBack / PixelBack) + PixelFore * Math.Log(SigmaFore / PixelFore));
                if (Sigma < MinSigma)
                {
                    MinSigma = Sigma;
                    Threshold = Y;
                }
            }
        }
        return Threshold;
    }

    // Also called intermeans
    // Iterative procedure based on the isodata algorithm [T.W. Ridler, S. Calvard, Picture 
    // thresholding using an iterative selection method, IEEE Trans. System, Man and 
    // Cybernetics, SMC-8 (1978) 630-632.] 
    // The procedure divides the image into objects and background by taking an initial threshold,
    // then the averages of the pixels at or below the threshold and pixels above are computed. 
    // The averages of those two values are computed, the threshold is incremented and the 
    // process is repeated until the threshold is larger than the composite average. That is,
    //  threshold = (average background + average objects)/2
    // The code in ImageJ that implements this function is the getAutoThreshold() method in the ImageProcessor class. 
    //
    // From: Tim Morris (dtm@ap.co.umist.ac.uk)
    // Subject: Re: Thresholding method?
    // posted to sci.image.processing on 1996/06/24
    // The algorithm implemented in NIH Image sets the threshold as that grey
    // value, G, for which the average of the averages of the grey values
    // below and above G is equal to G. It does this by initialising G to the
    // lowest sensible value and iterating:

    // L = the average grey value of pixels with intensities < G
    // H = the average grey value of pixels with intensities > G
    // is G = (L + H)/2?
    // yes => exit
    // no => increment G and repeat
    //
    // There is a discrepancy with IJ because they are slightly different methods

    public static int GetIsoDataThreshold(int[] HistGram)
    {
        int i, l, toth, totl, h, g = 0;
        for (i = 1; i < HistGram.Length; i++)
        {
            if (HistGram[i] > 0)
            {
                g = i + 1;
                break;
            }
        }
        while (true)
        {
            l = 0;
            totl = 0;
            for (i = 0; i < g; i++)
            {
                totl = totl + HistGram[i];
                l = l + (HistGram[i] * i);
            }
            h = 0;
            toth = 0;
            for (i = g + 1; i < HistGram.Length; i++)
            {
                toth += HistGram[i];
                h += (HistGram[i] * i);
            }
            if (totl > 0 && toth > 0)
            {
                l /= totl;
                h /= toth;
                if (g == (int)Math.Round((l + h) / 2.0))
                    break;
            }
            g++;
            if (g > HistGram.Length - 2)
            {
                return 0;
            }
        }
        return g;
    }



    // Shanhbag A.G. (1994) "Utilization of Information Measure as a Means of
    //  Image Thresholding" Graphical Models and Image Processing, 56(5): 414-419
    // Ported to ImageJ plugin by G.Landini from E Celebi's fourier_0.8 routines
    public static int GetShanbhagThreshold(int[] HistGram)
    {
        int threshold;
        int ih, it;
        int first_bin;
        int last_bin;
        double term;
        double tot_ent;  /* total entropy */
        double min_ent;  /* max entropy */
        double ent_back; /* entropy of the background pixels at a given threshold */
        double ent_obj;  /* entropy of the object pixels at a given threshold */
        double[] norm_histo = new double[HistGram.Length]; /* normalized histogram */
        double[] P1 = new double[HistGram.Length]; /* cumulative normalized histogram */
        double[] P2 = new double[HistGram.Length];

        int total = 0;
        for (ih = 0; ih < HistGram.Length; ih++)
            total += HistGram[ih];

        for (ih = 0; ih < HistGram.Length; ih++)
            norm_histo[ih] = (double)HistGram[ih] / total;

        P1[0] = norm_histo[0];
        P2[0] = 1.0 - P1[0];
        for (ih = 1; ih < HistGram.Length; ih++)
        {
            P1[ih] = P1[ih - 1] + norm_histo[ih];
            P2[ih] = 1.0 - P1[ih];
        }

        /* Determine the first non-zero bin */
        first_bin = 0;
        for (ih = 0; ih < HistGram.Length; ih++)
        {
            if (!(Math.Abs(P1[ih]) < 2.220446049250313E-16))
            {
                first_bin = ih;
                break;
            }
        }

        /* Determine the last non-zero bin */
        last_bin = HistGram.Length - 1;
        for (ih = HistGram.Length - 1; ih >= first_bin; ih--)
        {
            if (!(Math.Abs(P2[ih]) < 2.220446049250313E-16))
            {
                last_bin = ih;
                break;
            }
        }

        // Calculate the total entropy each gray-level
        // and find the threshold that maximizes it 
        threshold = -1;
        min_ent = Double.MaxValue;

        for (it = first_bin; it <= last_bin; it++)
        {
            /* Entropy of the background pixels */
            ent_back = 0.0;
            term = 0.5 / P1[it];
            for (ih = 1; ih <= it; ih++)
            { //0+1?
                ent_back -= norm_histo[ih] * Math.Log(1.0 - term * P1[ih - 1]);
            }
            ent_back *= term;

            /* Entropy of the object pixels */
            ent_obj = 0.0;
            term = 0.5 / P2[it];
            for (ih = it + 1; ih < HistGram.Length; ih++)
            {
                ent_obj -= norm_histo[ih] * Math.Log(1.0 - term * P2[ih]);
            }
            ent_obj *= term;

            /* Total entropy */
            tot_ent = Math.Abs(ent_back - ent_obj);

            if (tot_ent < min_ent)
            {
                min_ent = tot_ent;
                threshold = it;
            }
        }
        return threshold;
    }

    // M. Emre Celebi
    // 06.15.2007
    // Ported to ImageJ plugin by G.Landini from E Celebi's fourier_0.8 routines
    public static int GetYenThreshold(int[] HistGram)
    {
        int threshold;
        int ih, it;
        double crit;
        double max_crit;
        double[] norm_histo = new double[HistGram.Length]; /* normalized histogram */
        double[] P1 = new double[HistGram.Length]; /* cumulative normalized histogram */
        double[] P1_sq = new double[HistGram.Length];
        double[] P2_sq = new double[HistGram.Length];

        int total = 0;
        for (ih = 0; ih < HistGram.Length; ih++)
            total += HistGram[ih];

        for (ih = 0; ih < HistGram.Length; ih++)
            norm_histo[ih] = (double)HistGram[ih] / total;

        P1[0] = norm_histo[0];
        for (ih = 1; ih < HistGram.Length; ih++)
            P1[ih] = P1[ih - 1] + norm_histo[ih];

        P1_sq[0] = norm_histo[0] * norm_histo[0];
        for (ih = 1; ih < HistGram.Length; ih++)
            P1_sq[ih] = P1_sq[ih - 1] + norm_histo[ih] * norm_histo[ih];

        P2_sq[HistGram.Length - 1] = 0.0;
        for (ih = HistGram.Length - 2; ih >= 0; ih--)
            P2_sq[ih] = P2_sq[ih + 1] + norm_histo[ih + 1] * norm_histo[ih + 1];

        /* Find the threshold that maximizes the criterion */
        threshold = -1;
        max_crit = Double.MinValue;
        for (it = 0; it < HistGram.Length; it++)
        {
            crit = -1.0 * ((P1_sq[it] * P2_sq[it]) > 0.0 ? Math.Log(P1_sq[it] * P2_sq[it]) : 0.0) + 2 * ((P1[it] * (1.0 - P1[it])) > 0.0 ? Math.Log(P1[it] * (1.0 - P1[it])) : 0.0);
            if (crit > max_crit)
            {
                max_crit = crit;
                threshold = it;
            }
        }
        return threshold;
    }

    private static double A(int[] HistGram, int Index)
    {
        double Sum = 0;
        for (int Y = 0; Y <= Index; Y++)
            Sum += HistGram[Y];
        return Sum;
    }

    private static double B(int[] HistGram, int Index)
    {
        double Sum = 0;
        for (int Y = 0; Y <= Index; Y++)
            Sum += (double)Y * HistGram[Y];
        return Sum;
    }

    private static double C(int[] HistGram, int Index)
    {
        double Sum = 0;
        for (int Y = 0; Y <= Index; Y++)
            Sum += (double)Y * Y * HistGram[Y];
        return Sum;
    }

    private static double D(int[] HistGram, int Index)
    {
        double Sum = 0;
        for (int Y = 0; Y <= Index; Y++)
            Sum += (double)Y * Y * Y * HistGram[Y];
        return Sum;
    }


    private static bool IsDimodal(double[] HistGram)       // 检测直方图是否为双峰的
    {
        // 对直方图的峰进行计数，只有峰数位2才为双峰 
        int Count = 0;
        for (int Y = 1; Y < 255; Y++)
        {
            if (HistGram[Y - 1] < HistGram[Y] && HistGram[Y + 1] < HistGram[Y])
            {
                Count++;
                if (Count > 2) return false;
            }
        }
        if (Count == 2)
            return true;
        else
            return false;
    }


}