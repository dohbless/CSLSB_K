using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Media;
using System.Windows.Media.Imaging;

namespace Microsoft.Samples.Kinect.BodyIndexBasics
{
    class Hu
    {
        public double[] hu;

        public Hu()
        {
            hu = new double[7];
        }

        public double h1
        {
            set
            {
                this.hu[0] = value;
            }
            get
            {
                return this.hu[0];
            }
        }

        public double h2
        {
            set
            {
                this.hu[1] = value;
            }
            get
            {
                return this.hu[1];
            }
        }

        public double h3
        {
            set
            {
                this.hu[2] = value;
            }
            get
            {
                return this.hu[2];
            }
        }

        public double h4
        {
            set
            {
                this.hu[3] = value;
            }
            get
            {
                return this.hu[3];
            }
        }

        public double h5
        {
            set
            {
                this.hu[4] = value;
            }
            get
            {
                return this.hu[4];
            }
        }
        public double h6
        {
            set
            {
                this.hu[5] = value;
            }
            get
            {
                return this.hu[5];
            }
        }

        public double h7
        {
            set
            {
                this.hu[6] = value;
            }
            get
            {
                return this.hu[6];
            }
        }

        public Hu(WriteableBitmap bitmap, Int32Rect rect) :this()
        {
            ushort[,] pixel = new ushort[rect.Height, rect.Width];
            unsafe
            {
                var p = (ushort*)bitmap.BackBuffer.ToPointer();
                for (int i = 0; i < rect.Height; ++i)
                    for (int j = 0; j < rect.Width; ++j)
                    {
                        pixel[i, j] = p[(rect.Y + j) * bitmap.PixelWidth + rect.X + i];
                    }
            }

            long m00 = getM(pixel, 0, 0);
            long m01 = getM(pixel, 0, 1);
            long m10 = getM(pixel, 1, 0);
            long m02 = getM(pixel, 0, 2);
            long m20 = getM(pixel, 2, 0);
            long m11 = getM(pixel, 1, 1);
            long m03 = getM(pixel, 0, 3);
            long m30 = getM(pixel, 3, 0);
            long m21 = getM(pixel, 2, 1);
            long m12 = getM(pixel, 1, 2);

            double u10 = m10 / m00;
            double u01 = m01 / m00;

            double y00 = m00;
            double y11 = m11 - u01 * m10;
            double y20 = m20 - u10 * m10;
            double y02 = m02 - u01 * m01;
            double y30 = m30 - 3 * u10 * m20 + 2 * Math.Pow(u10,2) * m10;
            double y12 = m12 - 2 * u01 * m11 - u10 * m02 + 2 * Math.Pow(u01, 2) * m10;
            double y21 = m21 - 2 * u10 * m11 - u01 * m20 + 2 * Math.Pow(u10, 2) * m01;
            double y03 = m03 - 3 * u01 * m02 + 2 * Math.Pow(u01, 2) * m01;

            double n20 = y20 / Math.Pow(m00 , 2);
            double n02 = y02 / Math.Pow(m00 , 2);
            double n11 = y11 / Math.Pow(m00 , 2);
            double n30 = y30 / Math.Pow(m00 , 2.5);
            double n03 = y03 / Math.Pow(m00 , 2.5);
            double n12 = y12 / Math.Pow(m00 , 2.5);
            double n21 = y21 / Math.Pow(m00 , 2.5);

            this.hu[0] =  n20 + n02;
            this.hu[1] = Math.Pow((n20 - n02) , 2) + 4 * Math.Pow(n11 , 2);
            this.hu[2] = Math.Pow((n30 - 3 * n12) , 2) + Math.Pow((3 * n21 - n03), 2);
            this.hu[3] = Math.Pow((n30 + n12) , 2 )+ Math.Pow((n21 + n03) , 2);
            this.hu[4] = (n30 - 3 * n12) * (n30 + n12) * (Math.Pow((n30 + n12) , 2)- 3 * Math.Pow((n21 + n03), 2)) + (3 * n21 - n03) * (n21 + n03) * (3 * Math.Pow((n30 + n12) ,2) - Math.Pow((n21 + n03), 2));
            this.hu[5] = (n20 - n02) * (Math.Pow((n30 + n12),2 )- Math.Pow((n21 + n03),2)) + 4 * n11 * (n30 + n12) * (n21 + n03);
            this.hu[6] = (3 * n21 - n03) * (n30 + n12) * (Math.Pow((n30 + n12),2) - 3 * Math.Pow((n21 + n03) ,2)) + (3 * n12 - n30) * (n21 + n03) * (3 * Math.Pow((n30 + n12) ,2)- Math.Pow((n21 + n03) ,2));

        }


        public  Hu(ushort[,] pixel) : this()
        {
            long m00 = getM(pixel, 0, 0);
            long m01 = getM(pixel, 0, 1);
            long m10 = getM(pixel, 1, 0);
            long m02 = getM(pixel, 0, 2);
            long m20 = getM(pixel, 2, 0);
            long m11 = getM(pixel, 1, 1);
            long m03 = getM(pixel, 0, 3);
            long m30 = getM(pixel, 3, 0);
            long m21 = getM(pixel, 2, 1);
            long m12 = getM(pixel, 1, 2);

            double u10 = m10 / m00;
            double u01 = m01 / m00;

            double y00 = m00;
            double y11 = m11 - u01 * m10;
            double y20 = m20 - u10 * m10;
            double y02 = m02 - u01 * m01;
            double y30 = m30 - 3 * u10 * m20 + 2 * Math.Pow(u10, 2) * m10;
            double y12 = m12 - 2 * u01 * m11 - u10 * m02 + 2 * Math.Pow(u01, 2) * m10;
            double y21 = m21 - 2 * u10 * m11 - u01 * m20 + 2 * Math.Pow(u10, 2) * m01;
            double y03 = m03 - 3 * u01 * m02 + 2 * Math.Pow(u01, 2) * m01;

            double n20 = y20 / Math.Pow(m00, 2);
            double n02 = y02 / Math.Pow(m00, 2);
            double n11 = y11 / Math.Pow(m00, 2);
            double n30 = y30 / Math.Pow(m00, 2.5);
            double n03 = y03 / Math.Pow(m00, 2.5);
            double n12 = y12 / Math.Pow(m00, 2.5);
            double n21 = y21 / Math.Pow(m00, 2.5);

            this.hu[0] = n20 + n02;
            this.hu[1] = Math.Pow((n20 - n02), 2) + 4 * Math.Pow(n11, 2);
            this.hu[2] = Math.Pow((n30 - 3 * n12), 2) + Math.Pow((3 * n21 - n03), 2);
            this.hu[3] = Math.Pow((n30 + n12), 2) + Math.Pow((n21 + n03), 2);
            this.hu[4] = (n30 - 3 * n12) * (n30 + n12) * (Math.Pow((n30 + n12), 2) - 3 * Math.Pow((n21 + n03), 2)) + (3 * n21 - n03) * (n21 + n03) * (3 * Math.Pow((n30 + n12), 2) - Math.Pow((n21 + n03), 2));
            this.hu[5] = (n20 - n02) * (Math.Pow((n30 + n12), 2) - Math.Pow((n21 + n03), 2)) + 4 * n11 * (n30 + n12) * (n21 + n03);
            this.hu[6] = (3 * n21 - n03) * (n30 + n12) * (Math.Pow((n30 + n12), 2) - 3 * Math.Pow((n21 + n03), 2)) + (3 * n12 - n30) * (n21 + n03) * (3 * Math.Pow((n30 + n12), 2) - Math.Pow((n21 + n03), 2));

        }

        public double[] getHu()
        {
            return this.hu;
        }

        private long getM(ushort[,] pixels,int m,int n)
        {
            long res = 0;
            for (int i = 0; i < pixels.GetLength(0); ++i)
                for (int j = 0; j < pixels.GetLength(1); ++j)
                {
                    res += pixels[i, j] * pow(i, m) * pow(j,n);
                }

            return res;
        }                       

        private long getM(int[,] pixels, int m, int n)
        {
            long res = 0;
            for (int i = 0; i < pixels.GetLength(0); ++i)
                for (int j = 0; j < pixels.GetLength(1); ++j)
                {
                    res += pixels[i, j] * pow(i, m) * pow(j, n);
                }

            return res;
        }

        private int pow(int value,int y)
        {
            if (y == 0)
                return 1;
            int res = value;
            for (int i = 0; i < y; ++i)
                res *= value;
            return res;
        }

    }
}
