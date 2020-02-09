using System;
using System.Collections.Generic;
using System.Collections;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Microsoft.Kinect;
using System.Windows.Media.Imaging;
using System.Windows;
using System.Xml;




public struct frameDetail 
{
    public double hu1;
    public double hu2;
    public double hu3;
    public double hu4;
    public double hu5;
    public double hu6;
    public double hu7;
    public int frameCount;//实际位置
    public int frameSameWith; //循环次数
};

public struct distance
{
    public double distanceFrame;
    public int frameNum;
    public int frameTrueNum;
};




namespace Microsoft.Samples.Kinect.BodyIndexBasics
{
    //手语特征类
    class GesFeature
    {
        //保存每一帧图像
        private List<WriteableBitmap> GestureBitmap;
        //手心位置
        private List<CameraSpacePoint> hand_left;
        private List<CameraSpacePoint> hand_right;
        //手型区域
        private List<Int32Rect> Rect_left;
        private List<Int32Rect> Rect_right;
        //**
      //**  private List<ushort[,]> lefthand_rect;
        private List<ushort[,]> righthand_rect;

        private List<ushort[,]>wholeHand_rect;

        private List<DepthSpacePoint> handleftTips;

        private List<List<Double>> featuresInFrame;
        private List<double> feature1L;

        private List<Tuple<ushort[,], ushort[,]>> hand_rect;
        //关键帧所在索引
        private List<int> keyFrameIndex;
        private List<int> normKeyFrameIndex;
        //是否检测了关键帧
        private bool dectKeyFrame;

        private List<Hu> Hu_left;
        private List<Hu> Hu_right;

        //腰的位置
        private double yaoV;
        //左右手关键帧的hu特征

        private int count;
        private int count1;
        private int countWholeHand;
        private int countTips;
        private int countKeyFrameFeatures;

        public int Count
        {
            get
            {
                return count;
            }
        }
        public int CountWholeHand 
        {

            get 
            {
                return countWholeHand;
            }
        }
        public int Count1
        {
            get
            {
                return count1;
            }
        }
        public int CountKeyFrameFeatures 
        {

            get 
            {
                return countKeyFrameFeatures;            
            }
        }

        public int CountTips
        {
            get
            {
                return countTips;
            }
        }


        public bool DectKeyFrame
        {                                                                           
            get
            {
                return this.dectKeyFrame;
            }
        }

        public void saveData(string name)
        {
            var xmlDoc = new XmlDocument();

            #region 修改配置文件
            xmlDoc.Load("Data.xml"); //加载xml文件
            XmlNodeList nodeList = xmlDoc.SelectSingleNode("names").ChildNodes;
            string saveTextName = (nodeList.Count + 1).ToString();
            XmlNode root = xmlDoc.SelectSingleNode("names");//查找<names>   
            XmlElement xe1 = xmlDoc.CreateElement("item");//创建一个<book>节点   
            

            XmlElement xesub2 = xmlDoc.CreateElement("name");
            xesub2.InnerText = saveTextName; //要改
            xe1.AppendChild(xesub2);

            XmlElement xesub1 = xmlDoc.CreateElement("cname");
            xesub1.InnerText = name;//设置手语中文名 
            xe1.AppendChild(xesub1);//添加到<item>节点中 

            root.AppendChild(xe1);//添加到<bookstore>节点中   
            xmlDoc.Save("Data.xml");
            #endregion



            #region 保存手型区域




            List<frameDetail> frameAll = new List<frameDetail>();


            int flagSameWith=0;
            int countFrameWhole = 0;
            int KeyFeaturesCount=0;
          //  int ksFrame = 0;


            //*********** 1.左肩到手心 脊柱中心到手心 2.右肩到手心 脊柱中心到手心 3.脖子到左手肘 脊柱中心到左手肘 4.脖子到右手肘脊柱中心到右手肘 
            // 5.左肩到左手肘 左手肘到左手心 6.右肩到右手肘 右手肘到右手心 7.左手腕到左手心 脖子到左手腕 8.右手腕到右手心 脖子到右手腕   9.脊柱中心到头部向量长度 10.头部到左手心的向量长度
            // 11.脊柱中心到左手的向量长度 12.脊柱中心到右手的向量长度  13.左右手心距离      14. 左手心和手腕的相对位置  15.右手心和手腕的相对位置  
            foreach(List<Double> a in featuresInFrame)
            {
                KeyFeaturesCount = KeyFeaturesCount + 1;
                using (System.IO.StreamWriter file = new System.IO.StreamWriter(@"D:\frametool\keysFeatures" + KeyFeaturesCount + ".txt"))
                {
                    for(int jj=0;jj<a.Count;jj++)
                    {
                    file.Write(a[jj]+" , ");
                    }
                }
            }


            foreach (ushort[,] handRectShow in wholeHand_rect)
            {
                countFrameWhole = countFrameWhole + 1;
                int flag=0;                //计算此时黑点的个数
                int width = handRectShow.GetLength(0);
                 int height = handRectShow.GetLength(1);
                 for (int a = 0; a < width; a++)
                 {
                     //file.Write("{");
                     for (int b = 0; b < height; b++)
                     {
                         if (handRectShow[a, b] == 0)
                         {

                             flag = flag + 1;
                         }
                     }
                 }
                if(flag!=width*height){  //如果黑点的个数和图中像素点个数相同则忽略这一帧的图像
                  
                 //   ksFrame = countFrameWhole;
                using (System.IO.StreamWriter file = new System.IO.StreamWriter(@"D:\frameTool\whole hand\hand_" + countFrameWhole + ".txt"))
                {
                    
                    

                    for (int i = 0; i < width; i++)
                    {
                        //file.Write("{");
                        for (int j = 0; j < height; j++)
                        {
                            if (handRectShow[i, j] == 1)
                            {
                                handRectShow[i, j] = 255; //白色
                                file.Write(handRectShow[i, j] + ", ");
                                //arrtest[i,j] = true;
                                //file.Write(arrtest[i, j] + ", ");
                            }
                            else
                            {
                                handRectShow[i, j] = 0;
                                //arrtest[i, j] = false;
                                file.Write(0 + ",   ");
                            }

                        }
                        //file.WriteLine("},");
                        file.WriteLine(";");
                        //file.WriteLine();
                    }

                  


                    // int countArea = 0;
                    //   int flag;

                    double m00 = 0.0, m10 = 0.0, m01 = 0.0, m11 = 0.0, m20 = 0.0, m02 = 0.0, m30 = 0.0, m03 = 0.0, m12 = 0.0, m21 = 0.0;
                    double xx = 0; double yy = 0;

                    double n02, n03, n11, n12, n21, n20, n30;
                    double u00, u02, u03, u11, u12, u21, u20, u30;


                    double temp1, temp2, temp3;

                    for (int i = 0; i <  handRectShow.GetLength(0); ++i)
                    {                    //????????
                        for (int j = 0; j < handRectShow.GetLength(1); ++j)
                        {

                            m00 = m00 + handRectShow[i, j];
                            m01 = m01 + i * handRectShow[i, j];
                            m02 = m02 + Math.Pow(i, 2) * handRectShow[i, j];
                            m03 = m03 + Math.Pow(i, 3) * handRectShow[i, j];
                            m10 = m10 + j * handRectShow[i, j];
                            m11 = m11 + j * i * handRectShow[i, j];
                            m12 = m12 + j * Math.Pow(i, 2) * handRectShow[i, j];
                            m20 = m20 + Math.Pow(j, 2) * handRectShow[i, j];
                            m21 = m21 + Math.Pow(j, 2) * i * handRectShow[i, j];
                            m30 = m30 + Math.Pow(j, 3) * handRectShow[i, j];
                        }
                    }
                    xx = Math.Round(m10 / m00, 2);//u10
                    yy = Math.Round(m01 / m00, 2);//u01

                    u00 = m00;
                    u02 = m02 - yy * m01;
                    u03 = m03 - 3 * yy * m02 + 2 * Math.Pow(yy, 2) * m01;
                    u11 = m11 - yy * m10;
                    u12 = m12 - 2 * yy * m11 - xx * m02 + 2 * Math.Pow(yy, 2) * m10;
                    u21 = m21 - 2 * xx * m11 - yy * m20 + 2 * Math.Pow(xx, 2) * m01; 
                    u20 = m20 - xx * m10;
                    u30 = m30 - 3 * xx * m20 + 2 * Math.Pow(xx, 2) * m10;

                    n02 = u02 / (Math.Pow(u00, 2));
                    n03 = u03 / (Math.Pow(u00, 2.5));
                    n11 = u11 / (Math.Pow(u00, 2)); 
                    n12 = u12 / (Math.Pow(u00, 2.5));
                    n20 = u20 / (Math.Pow(u00, 2));
                    n21 = u21 / (Math.Pow(u00, 2.5));
                    n30 = u30 / (Math.Pow(u00, 2.5));




                    double hu1, hu2, hu3, hu4, hu5, hu6, hu7, position;
                    //hu1 = n20 + n02;
                    //temp1 = n20 - n02;
                    //hu2 = Math.Pow(temp1, 2) + 4 * Math.Pow(n11, 2);
                    //temp1 = n30 - 3 * n12; temp2 = 3 * n21 - n03;
                    //hu3 = Math.Pow(temp1, 2) + Math.Pow(temp2, 2);
                    //temp1 = n30 + n12; temp2 = n21 + n03;
                    //hu4 = Math.Pow(temp1, 2) + Math.Pow(temp2, 2);
                    //temp3 = n21 + n03;//   temp3 = n21 + 3 * n03;
                    //hu5 = (n30 - 3 * n12) * (n30 + n12) * (Math.Pow(temp1, 2) - 3 * Math.Pow(temp3, 2)) +
                    //        (3 * n21 - n03) * (n21 + n03) * (3 * Math.Pow(temp1, 2) - Math.Pow(temp2, 2));
                    //temp3 = n12 + n03;
                    //hu6 = (n20 - n02) * (Math.Pow(temp1, 2) - Math.Pow(temp3, 2)) + 4 * n11 * temp1 * temp2;
                    //hu7 = (3 * n12 - n30) * (n30 + n12) * (Math.Pow(temp1, 2) - 3 * Math.Pow(temp2, 2)) +
                    //        (3 * n21 - n03) * (n21 + n03) * (3 * Math.Pow(temp1, 2) - Math.Pow(temp3, 2));
                    hu1 = n20 + n02;
                    hu2=Math.Pow((n20 - n02), 2) + 4 * Math.Pow(n11, 2);
                    hu3= Math.Pow((n30 - 3 * n12), 2) + Math.Pow((3 * n21 - n03), 2);
                    hu4=Math.Pow((n30 + n12), 2) + Math.Pow((n21 + n03), 2);
                    hu5=(n30 - 3 * n12) * (n30 + n12) * (Math.Pow((n30 + n12), 2) - 3 * Math.Pow((n21 + n03), 2)) + (3 * n21 - n03) * (n21 + n03) * (3 * Math.Pow((n30 + n12), 2) - Math.Pow((n21 + n03), 2));
                    hu6=(n20 - n02) * (Math.Pow((n30 + n12), 2) - Math.Pow((n21 + n03), 2)) + 4 * n11 * (n30 + n12) * (n21 + n03);
                    hu7 = (3 * n21 - n03) * (n30 + n12) * (Math.Pow((n30 + n12), 2) - 3 * Math.Pow((n21 + n03), 2)) + (3 * n12 - n30) * (n21 + n03) * (3 * Math.Pow((n30 + n12), 2) - Math.Pow((n21 + n03), 2));

                    //double[] hu = { hu1, hu2, hu3, hu4, hu5, hu6, hu7 };
                    //for (int i = 0; i < 6; i++)
                    //    for (int j = 0; j < 6; j++)
                    //        if (hu[j] > hu[j + 1])
                    //        {
                    //            position = hu[j]; hu[j] = hu[j + 1]; hu[j + 1] = position;
                    //        }
                    //double minEnd = hu[0], maxEnd = hu[6];



                    double mean, standard;

                    mean = (hu1 + hu2 + hu3 + hu4 + hu5+hu6+hu7 ) / 7;

                    standard = Math.Sqrt((Math.Pow((hu1 - mean), 2) + Math.Pow((hu2 - mean), 2) + Math.Pow((hu3 - mean), 2) + Math.Pow((hu4 - mean), 2) + Math.Pow((hu5 - mean), 2) + Math.Pow((hu6 - mean), 2) + Math.Pow((hu7 - mean), 2))/7) ;
                


                    double m1, m2, m3, m4, m5, m6, m7;

                    //m1 = hu1;
                    //m2 = hu2;
                    //m3 = hu3;
                    //m4 = hu4;
                    //m5 = hu5;
                    //m6 = hu6;
                    //m7 = hu7;

                    //m1 = (hu1 - minEnd) / (maxEnd - minEnd);
                    //m2 = (hu2 - minEnd) / (maxEnd - minEnd);
                    //m3 = (hu3 - minEnd) / (maxEnd - minEnd);
                    //m4 = (hu4 - minEnd) / (maxEnd - minEnd);
                    //m5 = (hu5 - minEnd) / (maxEnd - minEnd);
                    //m6 = (hu6 - minEnd) / (maxEnd - minEnd);
                    //m7 = (hu7 - minEnd) / (maxEnd - minEnd);

                    m1 = (hu1 - mean) / standard;
                    m2 = (hu2 - mean) / standard;
                    m3 = (hu3 - mean) / standard;
                    m4 = (hu4 - mean) / standard;
                    m5 = (hu5 - mean) / standard;
                    m6 = (hu6 - mean) / standard;
                    m7 = (hu7 - mean) / standard;


                    // string line = "rectHand.X=" + hand.X.ToString() + ";" + hand.Y.ToString() + ";" + hand.Z.ToString() + "  第" + countFrame1 + "  帧";
                    // file.WriteLine(line);
                    frameDetail frm;
                    frm.hu1 = m1; frm.hu2 = m2; frm.hu3 = m3; frm.hu4 = m4; frm.hu5 = m5; frm.hu6 = m6; frm.hu7 = m7; frm.frameCount = countFrameWhole; //frameAll是从0开始的
                    frm.frameSameWith = flagSameWith; flagSameWith = flagSameWith + 1;//帧在frm中的位置
                    frameAll.Add(frm);//frameAll全局变量 关键帧中的实际位置：countFrameCount

                    file.WriteLine();
                    //string line = "这是第" + countFrameWhole + "的图像数据" + "width=" + handRectShow.GetLength(0) + "height=" + handRectShow.GetLength(1);
                    //file.WriteLine(line);
                   // countFrameWhole++;
                }//using (System.IO.StreamWriter file = new System.IO.StreamWriter(@"D:\whole hand\hand_" + countFrameWhole + ".txt"))
                }//if
            }
            //  int a = frameAll.Count;


          
            //frameAll存入的是那些有白点的帧 但是帧的帧数是实际录入视频帧的位置?（frameCount）


            List<distance> distanceDetail = new List<distance>();
            distance ds;
            ds.distanceFrame = 0.0; ds.frameTrueNum = frameAll[0].frameCount;//fra6eAll[0]和自己做距离运算   distanceFrame 是距离   frameNum 是在视频流中所在的位置    frameAll是按顺序进行存储的只要符合要求就放入
            ds.frameNum = frameAll[0].frameSameWith;//**********
            //ds.frammeNum = 0;
            int turnToLast=0,startToLast=0;//?
            distanceDetail.Add(ds);
            double distanceNew = 0.0; //int frameNum1 = 0; ?
            int frameTrueNum1 = 0;//就是一个用来统计帧数的frameNum 和frameTrueNum

            //for (int i = 0; i < frameAll.Count - 1; i++)//这里是全部的帧  frame是个list从0开始
            for (int i = 1; i < 20; i++)   //20
            {
                distanceNew = 
                    Math.Pow((frameAll[i].hu1 - frameAll[0].hu1), 2) +      //frameAll【】存入的都是帧的实际位置以及每一帧hu的其他信息
                    Math.Pow((frameAll[i].hu2 - frameAll[0].hu2), 2) +
                    Math.Pow((frameAll[i].hu3 - frameAll[0].hu3), 2) +
                    Math.Pow((frameAll[i].hu4 - frameAll[0].hu4), 2) +    //frame=1存在list    frameAll的第0个位置
                    Math.Pow((frameAll[i].hu5 - frameAll[0].hu5), 2) +
                    Math.Pow((frameAll[i].hu6 - frameAll[0].hu6), 2) + 
                    Math.Pow((frameAll[i].hu7 - frameAll[0].hu7), 2);


                    distanceNew = Math.Sqrt(distanceNew);
                 
                    frameTrueNum1 = frameAll[i].frameCount;//frameCount关键帧实际的位置是什么
                    ds.frameTrueNum = frameTrueNum1;
                    ds.distanceFrame = distanceNew;   
                   // frameNum1 = frameNum1 + 1;   
                    ds.frameNum =frameAll[i].frameSameWith;       //********      
                    distanceDetail.Add(ds);
            }

            distance temprefresh;
            for (int i = 0; i <distanceDetail.Count-1; ++i)
            {//接口
                for (int j = distanceDetail.Count - 1; j >i; --j)
                {//接口
                    if (distanceDetail[j].distanceFrame < distanceDetail[j-1].distanceFrame)//距离大小的排序
                    {
                        temprefresh = distanceDetail[j]; distanceDetail[j] = distanceDetail[j-1]; distanceDetail[j-1] = temprefresh;
                    }
                }
            }


                double temption = 0.0; double result = 0.0; int resultend = -1; int turn =20; int start = 0;//这里设置接口*****************    30改为20  25

                List<int> keyShow = new List<int>(); int flagK = 0; int flagg = 0; List<int> part2 = new List<int>(); int startFlag = 0;

                List<int> keys = new List<int>();
               /**排序之后要进行的操作*/
            while(turn<frameAll.Count)
            {

                for (int i = 0; i < distanceDetail.Count; i++) 
                {
                    for (int m = 0; m <= i; m++)//前一个集合中元素个数      当i=19时候 说明第一个集合中已经有了20个元素
                    { 
                        for (int j = i + 1; j < distanceDetail.Count ; j++) //后一个集合中元素个数     此时第二个集合j=20 与 j<20不符合 所以不进入循环
                        {

                            result = result +
                                Math.Pow((frameAll[distanceDetail[m].frameNum ].hu1 - frameAll[distanceDetail[j].frameNum ].hu1), 2) +  //帧在frame中的位置 而不是frame真实的位置frameTrueNum
                                Math.Pow((frameAll[distanceDetail[m].frameNum ].hu2 - frameAll[distanceDetail[j].frameNum ].hu2), 2) +
                                Math.Pow((frameAll[distanceDetail[m].frameNum ].hu3 - frameAll[distanceDetail[j].frameNum ].hu3), 2) +
                                Math.Pow((frameAll[distanceDetail[m].frameNum ].hu4 - frameAll[distanceDetail[j].frameNum ].hu4), 2) +
                                Math.Pow((frameAll[distanceDetail[m].frameNum ].hu5 - frameAll[distanceDetail[j].frameNum ].hu5), 2) +
                                Math.Pow((frameAll[distanceDetail[m].frameNum ].hu6 - frameAll[distanceDetail[j].frameNum ].hu6), 2) +
                                Math.Pow((frameAll[distanceDetail[m].frameNum ].hu7 - frameAll[distanceDetail[j].frameNum ].hu7), 2);

                        }
                    }// for (int m = 0; m <= i; m++)
                    if ((19 - i) != 0)//19 17 
                    {
                        result = result / ((i + 1) * (20 - i - 1));//每次求出来的值 20 18
                        if (temption < result)
                        {
                            temption = result;
                            result = 0.0;
                        }
                        else  //目标函数下降退回上一步 在第二类别中 将所有的di按照时间排序 时间上最早出现的t对应的就是关键帧
                        {

                            //  resultend = distanceDetail[i].frameTrueNum;
                            // flagK = distanceDetail[i].frameNum; 
                            // keyShow.Add(resultend);
                            temption = 0.0;
                            result = 0.0;
                            flagg = i;//flagg是找到开始发生变化的位置 但是位置是打乱的要从第二个类别里面选出最早出现的帧作为关键帧
                            startFlag = i;
                            resultend = distanceDetail[flagg].frameTrueNum; //这里的resultend作为一个基准和之后得到的resultend做比较如果之后的小则替换
                            for (int t = flagg ; t < distanceDetail.Count; t++)
                            {
                                if (resultend > distanceDetail[t].frameTrueNum)
                                {
                                    resultend = distanceDetail[t].frameTrueNum;
                                    flagK = distanceDetail[t].frameNum;
                                    startFlag=t;
                                }

                            }


                            List<distance> sortDistance=new List<distance>();
                            for (int t = flagg; t < distanceDetail.Count; t++) 
                            {
                                sortDistance.Add(distanceDetail[t]);
                            }

                            distance temptionSort;
                            for (int s = 0; s < sortDistance.Count-1; ++s) //s 是i
                            {
                                for (int u = sortDistance.Count-1; u >s; --u)  //u 是 j
                                {

                                    if (sortDistance[u].frameTrueNum < sortDistance[u-1].frameTrueNum)
                                    {
                                        temptionSort = sortDistance[u]; sortDistance[u] = sortDistance[u-1]; sortDistance[u-1] = temptionSort;
                                    }
                                }
                            
                            }



                                using (System.IO.StreamWriter file = new System.IO.StreamWriter(@"D:\frameTool\frameTime\hand_" + resultend + ".txt"))
                                {
                                    for (int w = 0; w < sortDistance.Count - 1; w++) 
                                    {

                                        file.Write(sortDistance[w].frameTrueNum + "  ");
                                    
                                    }
                                    file.Write(";");
                                    for (int w = 0; w < sortDistance.Count - 1; w++)
                                    {

                                        file.Write(sortDistance[w].distanceFrame+ "  ");

                                    }
                                }

                                sortDistance.Clear();

                            start = distanceDetail[startFlag].frameNum;
                            turn = start + 19;//19
                            keyShow.Add(resultend);
                            keys.Add(resultend);
                            break;

                        }//else
                    }  //if ((14 - i) != 0)
                    else
                    {
                        temption = 0.0;
                        result = 0.0;
                        startFlag = distanceDetail.Count - 1;
                        resultend = distanceDetail[startFlag].frameTrueNum;
                        start = distanceDetail[startFlag].frameNum;
                        turn = start + 19;//19
                        keyShow.Add(resultend);
                        keys.Add(resultend);
                        break;
                    }
                }// for (int i = 0; i < distanceDetail.Count; i++) 
               
               
                  //  start =flagK;  // resultend = distanceDetail[i].frameTrueNum
                    //turn  =start+19;
                if (turn > frameAll.Count - 1)//位置是frameAll[frameAll.Count - 1].frameCount 4
                {
                    turnToLast = frameAll.Count - 1;   //最后一帧
                    startToLast = start;
                    distance dsnew2;
                   // double distanceFrameEnd = 0.0;
                    int frameTrueNum12= distanceDetail[startFlag].frameTrueNum;;
                    int frameNum12 = distanceDetail[startFlag].frameNum;
                    result = 0.0;
                    distanceDetail.Clear();
                    dsnew2.distanceFrame = 0.0;
                    dsnew2.frameTrueNum=frameTrueNum12;
                    dsnew2.frameNum = frameNum12;
                    distanceDetail.Add(dsnew2);
                    for (int c = startToLast+1; c <= turnToLast; c++) 
                    {
                        distanceNew =
                               Math.Pow((frameAll[c].hu1 - frameAll[startToLast ].hu1), 2) +
                               Math.Pow((frameAll[c].hu2 - frameAll[startToLast ].hu2), 2) +
                               Math.Pow((frameAll[c].hu3 - frameAll[startToLast ].hu3), 2) +
                               Math.Pow((frameAll[c].hu4 - frameAll[startToLast ].hu4), 2) +
                               Math.Pow((frameAll[c].hu5 - frameAll[startToLast ].hu5), 2) +
                               Math.Pow((frameAll[c].hu6 - frameAll[startToLast ].hu6), 2) +
                               Math.Pow((frameAll[c].hu7 - frameAll[startToLast ].hu7), 2);

                        distanceNew = Math.Sqrt(distanceNew);
                        dsnew2.distanceFrame = distanceNew; dsnew2.frameTrueNum = frameAll[c].frameCount;
                        dsnew2.frameNum = c;
                        distanceDetail.Add(dsnew2);

                    }

                        distance temprefresh3;
                        for (int p = 0; p < distanceDetail.Count - 1; p++)//接口
                            for (int q = p + 1; q < distanceDetail.Count; q++)//接口
                                if (distanceDetail[q].distanceFrame < distanceDetail[p].distanceFrame)//距离大小的排序
                                {
                                    temprefresh3 = distanceDetail[p]; distanceDetail[p] = distanceDetail[q]; distanceDetail[q] = temprefresh3;
                                }








                        for (int p = 0; p < distanceDetail.Count; p++) //i
                        {
                            for (int q = 0;q <= p; q++)//前一个集合中元素个数      当i=19时候 说明第一个集合中已经有了20个元素 q是m
                            {
                                for (int j = p + 1; j < distanceDetail.Count; j++) //后一个集合中元素个数     此时第二个集合j=20 与 j<20不符合 所以不进入循环
                                {

                                    result = result +
                                        Math.Pow((frameAll[distanceDetail[q].frameNum].hu1 - frameAll[distanceDetail[j].frameNum].hu1), 2) +  //帧在frame中的位置 而不是frame真实的位置frameTrueNum
                                        Math.Pow((frameAll[distanceDetail[q].frameNum].hu2 - frameAll[distanceDetail[j].frameNum].hu2), 2) +
                                        Math.Pow((frameAll[distanceDetail[q].frameNum].hu3 - frameAll[distanceDetail[j].frameNum].hu3), 2) +
                                        Math.Pow((frameAll[distanceDetail[q].frameNum].hu4 - frameAll[distanceDetail[j].frameNum].hu4), 2) +
                                        Math.Pow((frameAll[distanceDetail[q].frameNum].hu5 - frameAll[distanceDetail[j].frameNum].hu5), 2) +
                                        Math.Pow((frameAll[distanceDetail[q].frameNum].hu6 - frameAll[distanceDetail[j].frameNum].hu6), 2) +
                                        Math.Pow((frameAll[distanceDetail[q].frameNum].hu7 - frameAll[distanceDetail[j].frameNum].hu7), 2);

                                }
                            }// for (int m = 0; m <= i; m++)
                            if ((19- p) != 0)//19
                            {
                                result = result / ((p + 1) * (20 - p - 1));//每次求出来的值 20
                                if (temption < result)
                                {
                                    temption = result;
                                    result = 0.0;
                                }
                                else  //目标函数下降退回上一步 在第二类别中 将所有的di按照时间排序 时间上最早出现的t对应的就是关键帧
                                { 

                                    //  resultend = distanceDetail[i].frameTrueNum;
                                    // flagK = distanceDetail[i].frameNum; 
                                    // keyShow.Add(resultend);
                                    temption = 0.0;
                                    result = 0.0;
                                    flagg = p;//flagg是找到开始发生变化的位置 但是位置是打乱的要从第二个类别里面选出最早出现的帧作为关键帧
                                   //
                                    resultend = distanceDetail[flagg].frameTrueNum; //这里的resultend作为一个基准和之后得到的resultend做比较如果之后的小则替换      
                                    for (int t = flagg; t < distanceDetail.Count; t++)
                                    {
                                        if (resultend > distanceDetail[t].frameTrueNum)
                                        {
                                            resultend = distanceDetail[t].frameTrueNum;
                                            flagK = distanceDetail[t].frameNum;
                                            startFlag = t;
                                        }

                                    }
                                    //start = distanceDetail[startFlag].frameNum;
                                    //turn = start + 14;//19
                                    keyShow.Add(resultend);
                                    keys.Add(resultend);
                                    break;

                                }//else
                            }  //if ((14 - i) != 0)
                            //else
                            //{
                            //    temption = 0.0;
                            //    result = 0.0;
                            //    startFlag = distanceDetail.Count - 1;
                            //    resultend = distanceDetail[startFlag].frameTrueNum;
                            //    start = distanceDetail[startFlag].frameNum;
                            //    turn = start + 14;//19
                            //    keyShow.Add(resultend);
                            //    break;
                            //}

                        }
                    //
                    
                       break; 
                }
                else
                {   
                        //distanceDetail.Clear();
                        distance dsnew1;                      // start = distanceDetail[startFlag].frameNum;      turn = start + 14;
                     //   double distanceFrame11 = 0.0;  
                        int frameTrueNum11 = distanceDetail[startFlag].frameTrueNum; //15 flagg  错了吗   
                        int frameNum11 = distanceDetail[startFlag].frameNum;//10a flagg
                        distanceDetail.Clear();
                        dsnew1.distanceFrame = 0.0; 
                        dsnew1.frameTrueNum =frameTrueNum11;  //detailFrame     start=resultend;  // resultend = distanceDetail[i].frameTrueNum
                        dsnew1.frameNum = frameNum11;
                        distanceDetail.Add(dsnew1);
                    for (int c = start+1; c <=turn ; c++)  //
                        {
                            distanceNew =
                                Math.Pow((frameAll[c ].hu1 - frameAll[start].hu1), 2) +
                                Math.Pow((frameAll[c ].hu2 - frameAll[start].hu2), 2) +
                                Math.Pow((frameAll[c ].hu3 - frameAll[start].hu3), 2) +
                                Math.Pow((frameAll[c ].hu4 - frameAll[start].hu4), 2) +
                                Math.Pow((frameAll[c ].hu5 - frameAll[start].hu5), 2) +
                                Math.Pow((frameAll[c ].hu6 - frameAll[start].hu6), 2) +
                                Math.Pow((frameAll[c ].hu7 - frameAll[start].hu7), 2);
                           // frameNew = frameAll[c + 1].frameCount; 

                            distanceNew = Math.Sqrt(distanceNew);
                            dsnew1.distanceFrame = distanceNew; dsnew1.frameTrueNum = frameAll[c].frameCount;
                            dsnew1.frameNum=c;
                            distanceDetail.Add(dsnew1);
                        }


                      distance temprefresh2;
                        for (int p = 0; p < distanceDetail.Count - 1; p++)//接口
                            for (int q = p + 1; q < distanceDetail.Count; q++)//接口
                                if (distanceDetail[q].distanceFrame < distanceDetail[p].distanceFrame)//距离大小的排序
                                {
                                    temprefresh2 = distanceDetail[p]; distanceDetail[p] = distanceDetail[q]; distanceDetail[q] = temprefresh2;
                                }


                }

              }
        


            int [,]arKeys=new int[keys.Count,keys.Count];
            int maxt = keys[0] - 1;
            int it = 0;
            int jt = 0;
            for (int kt = 0; kt < keys.Count; kt++)
            {

                if (keys[kt] != maxt + 1)
                {
                    it++;
                    jt = 0;
                    maxt = keys[kt] - 1;
                }

                arKeys[it,jt] = keys[kt];
                maxt = keys[kt];
                jt++;  
            
            }

            int countt = 0;  List<int>keyst=new List<int>();  int tempt=0;

            int endt = frameAll[frameAll.Count - 1].frameCount - 5;
            for (int at = 0; at < keys.Count; at++) 
            {
                for (int bt = 0; bt < keys.Count; bt++) 
                {
                    if (arKeys[at, bt] != 0) 
                    {
                        countt++;
                     
                    }
                
                }
                tempt=at;//这里at行存储在temp
                if (countt != 0) 
                {
                    if (countt <= 2) 
                    {
                        for (int xt = 0; xt < countt; xt++) 
                        {
                            if (arKeys[tempt, xt] <= endt)  //或者<
                            {
                                keyst.Add(arKeys[tempt, xt]);
                            }
                        }
                    }//if (countt <= 2) 
                    else if (countt == 3)
                    {
                        if (arKeys[tempt, 1] <= endt)
                        {
                            keyst.Add(arKeys[tempt, 1]);//加入中间的数
                        }
                    }//  else if (countt == 3)
                    else 
                    {
                        for (int xt = countt -2; xt < countt; xt++) 
                        {
                            if (arKeys[tempt, xt] <= endt)
                            {
                                keyst.Add(arKeys[tempt, xt]);//加入倒数后两个数
                            }
                        }
                    }//else
                }// if (countt != 0) 
                countt = 0;
            }


            using (System.IO.StreamWriter file = new System.IO.StreamWriter(@"D:\frameTool\end\endkeys.txt" )) 
            {

                for (int wt = 0; wt < keyst.Count; wt++)
                {

                    file.Write(keyst[wt]+ "  ");

                }
            
            }


            keys.Clear();//清空最终的关键帧序列

            keyst.Clear();


            using (System.IO.StreamWriter file = new System.IO.StreamWriter(@"D:\frameTool\keyFrame\hand_Pos.txt"))
            {
                int length = keyShow.Count;
                for (int i = 0; i < length; i++)
                {
                   string line = "第：" + keyShow[i]+ "是关键帧";
                   file.WriteLine(line);
                }
                
            }

            using (System.IO.StreamWriter file = new System.IO.StreamWriter(@"D:\handleftTips\"+saveTextName+".txt"))
            {

                foreach (DepthSpacePoint dlp in handleftTips) 
                {
                    float lineTipsX = dlp.X;
                    float lineTipsY = dlp.Y;
                    file.WriteLine(lineTipsX+"   "+lineTipsY);// 直接追加文件末尾，换行 
                
                }
            
            
            }

            using (System.IO.StreamWriter file = new System.IO.StreamWriter(@"D:\frameTool\blackWhite\handCount.txt"))
            {
                int frameCount = 0;
                foreach (ushort[,] handRectShow in wholeHand_rect)
                {
                    int cou = 0;

                    int width = handRectShow.GetLength(0);
                    int height = handRectShow.GetLength(1);
                    //for (int a = 0; a < width; a++)
                    //{
                    //    //file.Write("{");
                    //    for (int b = 0; b < height; b++)
                    //    {
                    //        if (handRectShow[a, b] == 0)
                    //        {

                    //            cou = cou + 1;
                    //        }
                    //    }
                    //}
                    //if (cou != width * height)
                    //{
                        int black = 0; int white = 0;
                        for (int i = 0; i < width; i++)
                        {
                            //file.Write("{");
                            for (int j = 0; j < height; j++)
                            {
                                if (handRectShow[i, j] == 255)//255白点个数
                                {
                                    white++;

                                }
                                else
                                {
                                    black++;
                                }
                            }
                        }// for (int i = 0; i < width; i++)

                        frameCount++;
                        file.WriteLine("这是第：" + frameCount + "  帧的白点个数：" + white + "  黑点个数：" + black);
                    }
                
            }

            using (System.IO.StreamWriter file = new System.IO.StreamWriter(@"D:\data\Rect_right" + saveTextName + ".txt"))
            {
                //int widthR=0,heightR=0;
                //int widthL=0,heightL=0;
                int countFrame = 1;
                foreach (Tuple<ushort[,], ushort[,]> handrect in hand_rect)
                {


                    //string ad = handrect.Item1.ToString();
                    int heightL = handrect.Item1.GetUpperBound(0) + 1;
                    int widthL = handrect.Item1.GetUpperBound(1) + 1;
                    int heightR = handrect.Item2.GetUpperBound(0) + 1;
                    int widthR = handrect.Item2.GetUpperBound(1) + 1;
                    //string line = "rectl.width height=" + widthR;
                    string line = "rectl.width =" + heightL.ToString() + "    rect1.height=" + widthL.ToString() + "   rect2.widthR=" + heightR.ToString() + "   rectl2.heightR=" + widthR.ToString() + "     第" + countFrame + "   帧";
                    //+ "   rectl.width=" + rectL.Width.ToString() + "   rectl.height" + rectL.Height.ToString();
                    file.WriteLine(line);
                    countFrame++;
                }
            }

            //using (System.IO.StreamWriter file = new System.IO.StreamWriter(@"D:\data\Rect_right" + saveTextName + ".txt"))
            //{
            //    foreach (Int32Rect rectR in Rect_right)
            //    {
            //        string line = "rectR.X=" + rectR.X.ToString() + "    rectR.y=" + rectR.Y.ToString() + "   rectR.width=" + rectR.Width.ToString() + "   rectR.height" + rectR.Height.ToString();
            //        file.WriteLine(line);
            //    }
            //}
            #endregion


            #region 保存关键帧位置
            detectKeyFrame();
            using (System.IO.StreamWriter file = new System.IO.StreamWriter(@"keyFrame\" + saveTextName + ".txt"))
            {

                foreach (int KFindex in keyFrameIndex)
                {
                    string line = KFindex.ToString();
                    file.WriteLine(line);// 直接追加文件末尾，换行 
                }
            }
            #endregion

            #region 保存hu
            genHu();
            using (System.IO.StreamWriter file = new System.IO.StreamWriter(@"Hu\left_" + saveTextName + ".txt"))
            {

                foreach (var hand in Hu_left)
                {
                    string line = hand.h1.ToString() + ";" ;
                    line += hand.h2.ToString() + ";";
                    line += hand.h3.ToString() + ";";
                    line += hand.h4.ToString() + ";";
                    line += hand.h5.ToString() + ";";
                    line += hand.h6.ToString() + ";";
                    line += hand.h7.ToString();
                    file.WriteLine(line);// 直接追加文件末尾，换行 
                }
            }

            using (System.IO.StreamWriter file = new System.IO.StreamWriter(@"Hu\right_" + saveTextName + ".txt"))
            {

                foreach (var hand in Hu_right)
                {
                    string line = hand.h1.ToString() + ";";
                    line += hand.h2.ToString() + ";";
                    line += hand.h3.ToString() + ";";
                    line += hand.h4.ToString() + ";";
                    line += hand.h5.ToString() + ";";
                    line += hand.h6.ToString() + ";";
                    line += hand.h7.ToString();
                    file.WriteLine(line);// 直接追加文件末尾，换行 
                }
            }
            #endregion
        }

        //清除所有数据
        public void clear()
        {
            count1 = 0;
            count = 0;
            countWholeHand = 0;
            countKeyFrameFeatures = 0;
            this.dectKeyFrame = false;
            this.GestureBitmap.Clear();
            this.hand_left.Clear();
            this.hand_right.Clear();
            this.Rect_left.Clear();
            this.Rect_right.Clear();
            this.hand_rect.Clear();
            this.keyFrameIndex.Clear();
            this.normKeyFrameIndex.Clear();
            this.Hu_left.Clear();
            this.Hu_right.Clear();
            this.wholeHand_rect.Clear();
            this.righthand_rect.Clear();
            this.handleftTips.Clear();
            this.featuresInFrame.Clear();
            this.feature1L.Clear();
        }

        public GesFeature()
        {

            GestureBitmap = new List<WriteableBitmap>();
            hand_left = new List<CameraSpacePoint>();
            hand_right = new List<CameraSpacePoint>();
            Rect_left = new List<Int32Rect>();
            Rect_right = new List<Int32Rect>();
            hand_rect = new List<Tuple<ushort[,], ushort[,]>>();
//**        
            feature1L=new List<double>();
            handleftTips=new List<DepthSpacePoint>();

           //**********特征值的存储/
            featuresInFrame = new List<List<Double>>();

            wholeHand_rect = new List<ushort[,]>();
            righthand_rect=new List<ushort[,]>();
//**
            keyFrameIndex = new List<int>();
            normKeyFrameIndex = new List<int>();
            Hu_left = new List<Hu>();
            Hu_right = new List<Hu>();
            yaoV = 0;
            count = 0;
            count1 = 0;
            countWholeHand = 0;
            countKeyFrameFeatures = 0;
            countTips = 0;
            dectKeyFrame = false;
        }
        public void AddFeature(WriteableBitmap bitmap, CameraSpacePoint leftHand, CameraSpacePoint rightHand, Int32Rect leftRect, Int32Rect rightRect)
        {
            AddGestureBitmap(bitmap);
            AddleftHand(leftHand);
            AddrightHand(rightHand);
            AddleftRect(leftRect);
            AddrightRect(rightRect);
            this.count++;
        }

        public void addOnly(Double feature1L) 
        {

            AddOnly(feature1L);
            this.countKeyFrameFeatures++;
        }
        public void AddFeaturesInFrame(List<Double> featuresInframe) 
        {

            AddFeaturesInKeyFrame(featuresInframe);
            this.countKeyFrameFeatures++;
        }

        public void AddFeature(CameraSpacePoint leftHand, CameraSpacePoint rightHand, Tuple<ushort[,], ushort[,]> HandRect)
        {
            AddleftHand(leftHand);
            AddrightHand(rightHand);
            AddHandRect(HandRect);
            this.count++;
        }

        public  unsafe void AddFeature(ushort[,] handrectR)
        {
        
            addleftHandRect(handrectR);
            this.count1++;
        }
        public unsafe void AddFeature1(ushort[,] handRectShow) 
        {
            addwholeHand(handRectShow);


           this.countWholeHand++;
        }

        public unsafe void AddFeature1(DepthSpacePoint dLT) 
        {

            addleftHandTip(dLT);
            this.countTips++;
        }

        public void AddFeature(WriteableBitmap bitmap, CameraSpacePoint leftHand, CameraSpacePoint rightHand, Rect leftRect, Rect rightRect)
        {
            AddGestureBitmap(bitmap);
            AddleftHand(leftHand);
            AddrightHand(rightHand);
            AddleftRect(leftRect);
            AddrightRect(rightRect);
            this.count++;
        }

        public void setSpineBase(double csp)
        {
            this.yaoV = csp;
        }
        public Tuple<List<CameraSpacePoint>, List<CameraSpacePoint>> getPalmTraj()
        {
            return new Tuple<List<CameraSpacePoint>, List<CameraSpacePoint>>(this.hand_left, this.hand_right);
        }

        // 检测关键帧
        public void detectKeyFrame()
        {
            int[] crow = getCrow();
            int range = 8; //分成8个区域
            int len = crow.Length / range;
            int mid = (max(crow) + min(crow))/2;
            //得到候选关键帧
            for(int i = 0;i< range; ++i)
            {
                var res = max(crow,i* len, (i+1)* len);
                if (res.Item2 > mid)
                    keyFrameIndex.Add(res.Item1);
            }

            //TODO 进行筛选
            //先筛起始手势
            if(keyFrameIndex[0] < 2* len)
            {
                if(hand_left[keyFrameIndex[0]].Y < yaoV && hand_right[keyFrameIndex[0]].Y < yaoV)
                {
                    keyFrameIndex.RemoveAt(0);
                    if (keyFrameIndex[0] < 2 * len)
                    {
                        if (hand_left[keyFrameIndex[0]].Y < yaoV && hand_right[keyFrameIndex[0]].Y < yaoV)
                        {
                            keyFrameIndex.RemoveAt(0);
                        }
                    }
                }
            }
            //再筛终结手势
            if(keyFrameIndex[keyFrameIndex.Count - 1] > (range - 2) * len)
            {
                if (hand_left[keyFrameIndex[keyFrameIndex.Count - 1]].Y < yaoV && hand_right[keyFrameIndex[keyFrameIndex.Count - 1]].Y < yaoV)
                {
                    keyFrameIndex.RemoveAt(keyFrameIndex.Count - 1);

                    if (keyFrameIndex[keyFrameIndex.Count - 1] > (range - 2) * len)
                    {
                        if (hand_left[keyFrameIndex[keyFrameIndex.Count - 1]].Y < yaoV && hand_right[keyFrameIndex[keyFrameIndex.Count - 1]].Y < yaoV)
                        {
                            keyFrameIndex.RemoveAt(keyFrameIndex.Count - 1);
                        }
                    }
                }
            }
            dectKeyFrame = true;
        }

        public Tuple<List<CameraSpacePoint>, List<CameraSpacePoint>> Norm(List<CameraSpacePoint> left, List<CameraSpacePoint> right)
        {
            var normLeft = norm(left, 0);
            var normRight = norm(right, 1);
            return new Tuple<List<CameraSpacePoint>, List<CameraSpacePoint>>(normLeft.ToList(), normRight.ToList());
        }
        //flag = 0 为左手 flag = 1 为右手
        private CameraSpacePoint[] norm(List<CameraSpacePoint> sp,int flag)
        {
            CameraSpacePoint[] res = new CameraSpacePoint[sp.Count];
            int[] resf = new int[sp.Count];
            List<CameraSpacePoint> cp;
            if (flag == 0) //左手
            {  
                cp = this.hand_left;
            }
            else//右手
            {
                cp = this.hand_right;
            }
            //TODO 归一化操作
            var sCenter = culCenter(sp);
            var dCenter = culCenter(cp);
            //尺度
            float sRate = (float) (culDist(sCenter) / culDist(dCenter));
            //时间
            float tRate = (float)(sp.Count / cp.Count);

            for(int i = 0; i < cp.Count; ++i)
            {
                int index = (int)(i * tRate + 0.5);
                if(resf[index] == 0)
                {
                    res[index].X = sRate * (cp[i].X - cp[0].X) + sp[0].X;
                    res[index].Y = sRate * (cp[i].Y - cp[0].Y) + sp[0].Y;
                    res[index].Z = sRate * (cp[i].Z - cp[0].Z) + sp[0].Z;
                }
                else
                {
                    res[index].X = (res[index].X * resf[index] + sRate * (cp[i].X - cp[0].X) + sp[0].X) / (resf[index] + 1);
                    res[index].Y = (res[index].Y * resf[index] + sRate * (cp[i].Y - cp[0].Y) + sp[0].Y) / (resf[index] + 1);
                    res[index].Z = (res[index].Z * resf[index] + sRate * (cp[i].Z - cp[0].Z) + sp[0].Z) / (resf[index] + 1);
                }
                resf[index] += 1;
            }

            //TODO 关键帧的下标也要缩放
            foreach(var e in keyFrameIndex)
            {
                normKeyFrameIndex.Add((int)(e * tRate + 0.5));
            }

            //插值
            for(int i = 0; i < sp.Count;)
            {
                if(resf[i] == 0)
                {
                    var down = i - 1;
                    var up = i + 1;

                    while(resf[down] == 0 && down >= 0)
                    {
                        down -= 1;
                    }

                    while(resf[up] == 0 && up < sp.Count)
                    {
                        up += 1;
                    }

                    if (down < 0 || up == sp.Count)
                        continue;//暂不处理

                    float x_diff = (res[up].X - res[down].X) / (up - down);
                    float y_diff = (res[up].Y - res[down].Y) / (up - down);
                    float z_diff = (res[up].Z - res[down].Z) / (up - down);
                    for(int j = 1;j< (up - down); j++)
                    {
                        res[down + j].X = res[down].X + j * x_diff;
                        res[down + j].Y = res[down].Y + j * y_diff;
                        res[down + j].Z = res[down].Z + j * z_diff;
                        resf[down + j] = 1;
                    }
                    i = up - 1;
                }
                i++;
            }

            if(resf[0] == 0)
            {
                var k = 0;
                while(resf[k] == 0)
                {
                    k++;
                }
                float x_diff = (res[k+1].X - res[k].X);
                float y_diff = (res[k+1].Y - res[k].Y);
                float z_diff = (res[k+1].Z - res[k].Z);

                for(int j = k-1;j>= 0; --j)
                {
                    res[j].X = res[j + 1].X - x_diff;
                    res[j].Y = res[j + 1].Y - y_diff;
                    res[j].Z = res[j + 1].Z - z_diff;
                    resf[j] = 1;
                }
            }

            if(resf[resf.Length-1] == 0)
            {
                var k = resf.Length - 1;
                while (resf[k] == 0)
                {
                    k--;
                }
                float x_diff = (res[k].X - res[k - 1].X);
                float y_diff = (res[k].Y - res[k - 1].Y);
                float z_diff = (res[k].Z - res[k - 1].Z);

                for (int j = k + 1; j < resf.Length; ++j)
                {
                    res[j].X = res[j - 1].X + x_diff;
                    res[j].Y = res[j - 1].Y + y_diff;
                    res[j].Z = res[j - 1].Z + z_diff;
                    resf[j] = 1;
                }
            }
            
            return res;
        }

        public int[] getNormKeyFrameIndex()
        {
            return normKeyFrameIndex.ToArray();
        }
        private void genHu()
        {
            foreach(var e in keyFrameIndex)
            {
                //Hu_left.Add(new Hu(GestureBitmap[e], Rect_left[e]));
                //Hu_right.Add(new Hu(GestureBitmap[e], Rect_right[e]));

                Hu_left.Add(new Hu(hand_rect[e].Item1));
                Hu_right.Add(new Hu(hand_rect[e].Item2));
            }
        }

        public Tuple<Hu[],Hu[]> getHu()
        {
            genHu();
            return new Tuple<Hu[], Hu[]>(Hu_left.ToArray(),Hu_right.ToArray());
        }

        private CameraSpacePoint culCenter(List<CameraSpacePoint> sp)
        {
            return culCenter(sp.ToList());
        }

        private CameraSpacePoint culCenter(CameraSpacePoint[] sp)
        {
            CameraSpacePoint center = new CameraSpacePoint();
            foreach(var s in sp)
            {
                center.X += s.X;
                center.Y += s.Y;
                center.Z += s.Z;
            }

            center.X /= sp.Length;
            center.Y /= sp.Length;
            center.Z /= sp.Length;

            return center;
        }

        private int max(int[] arr)
        {
            int max = arr[0];
            for (int i = 1; i < arr.Length; ++i)
                max = max < arr[i] ? arr[i] : max;
            return max;
        }

        private Tuple<int,int> max(int[] arr, int down, int up)
        {
            if (down < 0 || down > up)
                throw new Exception("max函数输入错误！");
            if (up > arr.Length)
                up = arr.Length;

            int max = arr[down];
            int index = down;
            for(int i = down;i< up; ++i)
            {
                if (arr[i] > max)
                {
                    max = arr[i];
                    index = i;
                }
            }
            return new Tuple<int, int>(index,max);
        }

        private int min(int[] arr)
        {
            int min = arr[0];
            for (int i = 1; i < arr.Length; ++i)
                min = min > arr[i] ? arr[i] : min;
            return min;
        }

        private int[] getCrow()
        {
            if (hand_left.Count != hand_right.Count)
                throw new Exception("左右手特征个数不匹配！");
            int[] crowS = new int[hand_left.Count];
            int[] crowl = getSingleCrow(hand_left);
            int[] crowr = getSingleCrow(hand_right);
            for (int i = 0; i < crowS.Length; ++i)
                crowS[i] = crowl[i] + crowr[i];
            return crowS;
        }

        private int[] getSingleCrow(List<CameraSpacePoint> single)
        {
            int[] crow = new int[single.Count];
            double sumD = 0;
            for (int i = 1; i < single.Count; ++i)
                sumD += culDist(single[i-1],single[0]);
            double muD = 2*sumD / (single.Count - 1); //平均距离的两倍

            //计算每个点的密集度
            for(int i = 0; i < single.Count; ++i)
            {
                //向前
                int ip = i;
                int ib = i;
                int value = 1;
                while(--ip >= 0)
                {
                    if (culDist(single[ip], single[ip + 1]) < muD)
                        value++;
                    break;
                }

                //向后
                while(++ib < single.Count)
                {
                    if (culDist(single[ib], single[ib - 1]) < muD)
                        value++;
                    break;
                }
                crow[i] = value;
            }

            return crow;
        }

        private double culDist(CameraSpacePoint sp)
        {
            return culDist(sp,sp);
        }
        private double culDist(CameraSpacePoint p1, CameraSpacePoint p2)
        {
            return Math.Sqrt((p1.X - p2.X) * (p1.X - p2.X) + (p1.Y - p2.Y) * (p1.Y - p2.Y) + (p1.Z - p2.Z) * (p1.Z - p2.Z));
        }

        private void AddGestureBitmap(WriteableBitmap bitmap)
        {
            if (count != GestureBitmap.Count)
                throw new Exception("图像数量不对！");
            this.GestureBitmap.Add(bitmap);
        }
        private void addleftHandRect(ushort[,] lfr)
        {
            
            if (count1 != righthand_rect.Count )
                throw new Exception("左手选定区域数量不对");//所有count数量都不对
            this.righthand_rect.Add(lfr);
        }
      
        private void addrightHandRect(ushort[,] rfr)
        {

            if (count1 != righthand_rect.Count)
                throw new Exception("左手选定区域数量不对");//所有count数量都不对
            this.righthand_rect.Add(rfr);
        }

        private void AddleftHand(CameraSpacePoint csp)
        {
            if (count != hand_left.Count)
                throw new Exception("左手手心特征数量不对！");
            this.hand_left.Add(csp);
        }

        private void AddrightHand(CameraSpacePoint csp)
        {
            if (count != hand_right.Count)
                throw new Exception("右手手心特征数量不对！");
            this.hand_right.Add(csp);
        }
        private void addwholeHand(ushort[,] frwhole)
        {
            if (countWholeHand != wholeHand_rect.Count)
               throw new Exception("手选定区域数量不对");//所有count数量都不对
            this.wholeHand_rect.Add(frwhole);
        }

        private void addleftHandTip(DepthSpacePoint dTL) 
        {
            if (countTips != handleftTips.Count)
                throw new Exception("手选定区域数量不对");
            this.handleftTips.Add(dTL);
        }





        private void AddHandRect(Tuple<ushort[,], ushort[,]> handRect)
        {
            if(count != hand_rect.Count)
                throw new Exception("手部区域特征数量不对！");
            this.hand_rect.Add(handRect);
        }
        private void AddleftRect(Int32Rect rect)
        {
            if (count != Rect_left.Count)
                throw new Exception("左手区域特征数量不对！");
            this.Rect_left.Add(rect);
        }

        private void AddOnly(Double feature1) 
        {
            if (countKeyFrameFeatures != featuresInFrame.Count)
                throw new Exception("帧信息不匹配");
            this.feature1L.Add(feature1);
        }
        private void AddFeaturesInKeyFrame(List<Double> list) 
        {
            if (countKeyFrameFeatures != featuresInFrame.Count)
                throw new Exception("帧的各个信息匹配不上！");
            this.featuresInFrame.Add(list);

        }

        private void AddrightRect(Int32Rect rect)
        {
            if (count != Rect_right.Count)
                throw new Exception("右手区域特征数量不对！");
            this.Rect_right.Add(rect);
        }

        private void AddleftRect(Rect rect)
        {
            if (count != Rect_left.Count)
                throw new Exception("左手区域特征数量不对！");
            this.Rect_left.Add(new Int32Rect((int)rect.X, (int)rect.Y, (int)rect.Width, (int)rect.Height));
        }

        private void AddrightRect(Rect rect)
        {
            if (count != Rect_right.Count)
                throw new Exception("右手区域特征数量不对！");
            this.Rect_right.Add(new Int32Rect((int)rect.X, (int)rect.Y, (int)rect.Width, (int)rect.Height));
        }


     
    }
}
