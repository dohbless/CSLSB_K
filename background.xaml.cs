using Microsoft.Kinect;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Globalization;
using System.IO;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Xml;
//这里是一堆引用
namespace Microsoft.Samples.Kinect.BodyIndexBasics//大概是骨骼深度数据处理的意思吧
{
    /// <summary>
    /// background.xaml 的交互逻辑
    /// </summary>
    public partial class background : Window, INotifyPropertyChanged
    {//更改了INotifyProperty？？什么鬼
        struct Pointt
        {
            public int x, y;
            public Pointt(int _x = 0, int _y = 0)
            { x = _x; y = _y;
            }
        }
        //这里是点的结构体 二维
        struct spPoint
        {
            public float x, y ,z;
            public spPoint(float _x = 0, float _y = 0,float _z=0) { x = _x; y = _y;z = _z; }
        }
        //新添加的 三维

        int time = 0;
        private DateTime start1;
        //这定义的是时间戳吗？
        //unfortunetaly 没被引用

        //private bool timeFlag = false;
        private bool recordFlag = false;
        //不知道
        private const int MapDepthToByte = 8000 / 256;
        //没看懂，8000/256并不是整数
        //过除以MapDepthToByte，大小缩小MapDepthToByte倍，化为256以内，
        //用于8阶灰度值的显示。其中不可视的近处与远处默认为0。

        private const double HandSize = 30; //手的半径
        //捕捉的半径吧
        private const double ClipBoundsThickness = 0;
        //剪辑的边界厚度，Thickness of clip edge rectangles
        private const double JointThickness = 3;
        //关节点厚度
        private CoordinateMapper coordinateMapper = null;
        //格式转换
        //话说厚度是啥玩意？能吃不？
        //word厚度？
        //private MyKalman lHandKalman;
        //private MyKalman rHandKalman;
        //经过卡尔曼滤波的
        private GesFeature gestureFeature;
        //手势
        //在别的两个文件中处理
        private const float InferredZPositionClamp = 0.1f;
        //Constant for clamping Z values of camera space points from being negative
        //这个是为了固定Z轴的，大概是为了避免什么诡异的现象出现吧
        #region Brush
        /// <summary>
        /// Brush used for drawing hands that are currently tracked as closed
        /// </summary>
        private readonly Brush handClosedBrush = new SolidColorBrush(Color.FromArgb(128, 0, 0, 255));

        /// <summary>
        /// Brush used for drawing hands that are currently tracked as opened
        /// </summary>
        private readonly Brush handOpenBrush = new SolidColorBrush(Color.FromArgb(128, 255, 0, 0));

        /// <summary>
        /// Brush used for drawing hands that are currently tracked as in lasso (pointer) position
        /// </summary>
        private readonly Brush handLassoBrush = new SolidColorBrush(Color.FromArgb(128, 0, 255, 0));

        /// <summary>
        /// Brush used for drawing joints that are currently tracked
        /// </summary>
        private readonly Brush trackedJointBrush = new SolidColorBrush(Color.FromArgb(255, 255, 192, 0));

        /// <summary>
        /// Brush used for drawing joints that are currently inferred
        /// </summary>        
        private readonly Brush inferredJointBrush = Brushes.Yellow;

        /// <summary>
        /// Pen used for drawing bones that are currently inferred
        /// </summary>        
        private readonly Pen inferredBonePen = new Pen(Brushes.Gray, 1);

        #endregion

        private FrameDescription colorFrameDescription = null;
        private WriteableBitmap colorBitmap = null;
        private FrameDescription depthFrameDescription = null;
        //private WriteableBitmap depthBitmap = null;
        //private WriteableBitmap bodyIndexBitmap = null;
        private WriteableBitmap displayBitmap = null;
        //以上是对Frame即帧的元素的描述，bitmap是位图？
        private int displayWidth;
        private int displayHeight;
        //我猜显然应该是画什么玩意的长和宽
        private byte[] depthPixels = null;
        //像素矩阵
        private uint[] bodyIndexPixels = null;
        //深度像素矩阵？
        //是的 但主要是人的

        private CameraSpacePoint[] cameraPoints = null;
        //摄像头系（x，y，z）
        private ColorSpacePoint[] colorPoints = null;
        //在color空间中的（x，y）？
        private DrawingGroup drawingGroup;
        //表示可作为单个绘图进行运算的集合

        private DrawingImage imageSource;
        //对图像数据使用drawing的？对象？

        /// <summary>
        /// Size of the RGB pixel in the bitmap
        /// </summary>
        private const int BytesPerPixel = 4;


        private Body[] bodies = null;
        //人集。。。好扯。。

        private static readonly uint[] BodyColor =
        {
            0x0000FF00,
            0x0000FF00,
            0x0000FF00,
            0x0000FF00,
            0x0000FF00,
            0x0000FF00,
        };//类初始化。。

        /// <summary>
        /// Active Kinect sensor
        /// </summary>
        private KinectSensor kinectSensor = null;
        //kinect传感器设备
        private MultiSourceFrameReader multiFrameSourceReader = null;
        //用来做单帧矩阵运算的
        private List<Tuple<JointType, JointType>> bones;
        //bones列表，列表中元素为bones由一堆二元组表示
        //but 为啥二元组

        private List<Pen> bodyColors;
        //bones的colors的列表
        /// <summary>
        /// Current status text to display
        /// </summary>
        private string statusText = null;
        //当前用来展示信息的玩意吧
        //保存左右手得到的位置信息
        private List<CameraSpacePoint> hand_left = new List<CameraSpacePoint>();
        private List<CameraSpacePoint> hand_right = new List<CameraSpacePoint>();
        //用来存camera数据吧，真想发表情@@@@
        /// <summary>
        /// Initializes a new instance of the MainWindow class.
        /// </summary>
        /// //私以为这里为为background拟遭一个展示例
    
    public background()
        {
            this.InitializeComponent();
            // initialize the components (controls) of the window


            initData();
            // get the kinectSensor object
            this.kinectSensor = KinectSensor.GetDefault();
            //通过默认传感器取得something吧就那样了
            //显示this为background的交互逻辑，莫非background的交互逻辑（或许就是那个窗口）成精了？？？
            //lHandKalman = new MyKalman(6, 3);// new MyKalman();
            ////卡尔曼滤波又不归我管
            //rHandKalman = new MyKalman(6, 3);// new MyKalman();
            //那么我就不管了^_^
            gestureFeature = new GesFeature();
            //建一个gesturefeature对象？这也不归我管^-^

            // get the coordinate mapper
            this.coordinateMapper = this.kinectSensor.CoordinateMapper;
            //应该是坐标图吧

            // open the reader for the depth frames
            //this.multiFrameSourceReader = this.kinectSensor.OpenMultiSourceFrameReader(FrameSourceTypes.Body | FrameSourceTypes.Color);
            this.multiFrameSourceReader = this.kinectSensor.OpenMultiSourceFrameReader(FrameSourceTypes.Depth | FrameSourceTypes.Body | FrameSourceTypes.BodyIndex);
            //也就是不读color数据而读深度数据，骨骼数据和多源数据？话说index到底是啥？啥玩意啊****

            // wire handler for frame arrival
            this.multiFrameSourceReader.MultiSourceFrameArrived += this.Reader_MultiSourceFrameArrived;
            //在下表示看不懂英文注释，however这大概是计数器？用来记录到达的多源数据帧数？

            //this.colorFrameDescription = this.kinectSensor.ColorFrameSource.FrameDescription;
            this.depthFrameDescription = this.kinectSensor.DepthFrameSource.FrameDescription;
            //话说depthDescription是啥玩意？

            // create the bitmap to display
            //this.colorBitmap = new WriteableBitmap(this.colorFrameDescription.Width, this.colorFrameDescription.Height, 96.0, 96.0, PixelFormats.Bgr32, null);
            this.depthPixels = new byte[this.depthFrameDescription.Width * this.depthFrameDescription.Height];
            //明天到期是真的虚，深度像素点有这么多位啊，大概depthDescription是用来存数据的吧，转成pixel。。。
            this.bodyIndexPixels = new uint[this.depthFrameDescription.Width * this.depthFrameDescription.Height];
            //话说bodyindex到底是啥？深度数据？
            //据称为玩家索引流，到底是啥影响不大吧
            //this.depthBitmap = new WriteableBitmap(this.depthFrameDescription.Width, this.depthFrameDescription.Height, 96.0, 96.0, PixelFormats.Bgr32, null);
            //this.bodyIndexBitmap = new WriteableBitmap(this.depthFrameDescription.Width, this.depthFrameDescription.Height, 96.0, 96.0, PixelFormats.Bgr32, null);
            this.displayBitmap = new WriteableBitmap(this.depthFrameDescription.Width, this.depthFrameDescription.Height, 96.0, 96.0, PixelFormats.Bgr32, null);

            













            //建立writeableBitmap函数并用something初始化吧
            this.displayHeight = this.depthFrameDescription.Height;
            this.displayWidth = this.depthFrameDescription.Width;
            //和深度数据一样的画什么玩意的长和宽啊
            this.cameraPoints = new CameraSpacePoint[this.depthFrameDescription.Width * this.depthFrameDescription.Height];
            this.colorPoints = new ColorSpacePoint[this.depthFrameDescription.Width * this.depthFrameDescription.Height];
            //一样的大小啊。。果然是一样的坐标系呢
            #region 人体区域
            // a bone defined as a line between two joints
            //定义了人体的关节位置
            this.bones = new List<Tuple<JointType, JointType>>();

            // Torso
            this.bones.Add(new Tuple<JointType, JointType>(JointType.Head, JointType.Neck));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.Neck, JointType.SpineShoulder));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.SpineMid));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineMid, JointType.SpineBase));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.ShoulderRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.ShoulderLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineBase, JointType.HipRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineBase, JointType.HipLeft));

            // Right Arm
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ShoulderRight, JointType.ElbowRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ElbowRight, JointType.WristRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristRight, JointType.HandRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HandRight, JointType.HandTipRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristRight, JointType.ThumbRight));

            // Left Arm
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ShoulderLeft, JointType.ElbowLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ElbowLeft, JointType.WristLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristLeft, JointType.HandLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HandLeft, JointType.HandTipLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristLeft, JointType.ThumbLeft));

            // Right Leg
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HipRight, JointType.KneeRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.KneeRight, JointType.AnkleRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.AnkleRight, JointType.FootRight));

            // Left Leg
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HipLeft, JointType.KneeLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.KneeLeft, JointType.AnkleLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.AnkleLeft, JointType.FootLeft));
            #endregion
            //这个是给出人体区域，差不多就是初始化了所有骨头吧
            #region 能检测的六个人的颜色，都不相同，我的大多数只会用到第一个颜色
            // populate body colors, one for each BodyIndex
            this.bodyColors = new List<Pen>();
            this.bodyColors.Add(new Pen(Brushes.Red, 6));
            this.bodyColors.Add(new Pen(Brushes.Orange, 6));
            this.bodyColors.Add(new Pen(Brushes.Green, 6));
            this.bodyColors.Add(new Pen(Brushes.Blue, 6));
            this.bodyColors.Add(new Pen(Brushes.Indigo, 6));
            this.bodyColors.Add(new Pen(Brushes.Violet, 6));
            #endregion
            //大概也就是初始化了所有人吧。。。maybe。。。。。。。。。。。。。
            // set IsAvailableChanged event notifier
            //this.kinectSensor.IsAvailableChanged += this.Sensor_IsAvailableChanged;
            //记录传感器变化，？？？？莫非是人物丢失，人物变化什么的？
            // open the sensor
            this.kinectSensor.Open();
            //终于开了
            // set the status text
            this.bStatusText = this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText
                                                            : Properties.Resources.NoSensorStatusText;
            //如果能跑就显示这些文本，however为啥你疯狂报错呢？换别的文件也不这样啊，凭啥啊
            // Create the drawing group we'll use for drawing
            this.drawingGroup = new DrawingGroup();
            //用来画图的
            // Create an image source that we can use in our image control
            this.imageSource = new DrawingImage(this.drawingGroup);
            //画图的控件吧，差不多意思一下
            // use the window object as the view model in this simple example
            this.DataContext = this;
            //没看懂，应该是程序问题，不学C#真的惨，咋学呢？？？？？？？
          


            this.ShowMsg.Content = "请选择录入手语的模式！";
            //问题不大，可以comment掉
            //为啥你们都报错呢？嗯？一个初始化加一个初始化
            //initData();

        }




        //接上文
        private void initData()
        {
            var dataS = new List<Gesture>();
            //这是一个局部的列表变量，我的fgo应该还是要请人肝好了

            var xmlDoc = new XmlDocument();
            //xml文件对象
            xmlDoc.Load("Data.xml"); //加载xml文件
            XmlNodeList nodeList = xmlDoc.SelectSingleNode("names").ChildNodes;
            //获取所有人的数据？话说C#是只有对象吗？我甚至觉得电学不错了
            foreach (XmlNode xn in nodeList)
            {
                //对于xml文件中的每一个单个节点（上面根据names这个key提取的）
                XmlElement xe = (XmlElement)xn;
                //建立元素，啥玩意
                dataS.Add(new Gesture() { id = 1, name = xe.LastChild.InnerText.ToString() });
                //这就加了？话说咋搞的数据？gesfeature？我想去（哭）
            }
            this.gestureList.ItemsSource = dataS;
            this.gestureList.DisplayMemberPath = "name";
            //显示没被定义，话说我也没找到，啥玩意
            //将这些数据添加到gesturelist中去，用name作为关键词吧
        }
        /// <summary>
        /// INotifyPropertyChangedPropertyChanged event to allow window controls to bind to changeable data
        /// //大概意思是可以将窗口控制绑定在一个变量上，话说你前面没错吗？
        /// </summary>
        public event PropertyChangedEventHandler PropertyChanged;
        //好吧，你爱绑绑，我没话说
        public ImageSource ImageSource
        {
            get
            {
                //return this.imageSource;
                return this.displayBitmap;
            }
            //这是啥，对象的函数？就一个，能在后面加几个吗而且返回的是displayBitmao了...
            //真是令人忧伤
        }

       



        public string bStatusText
        {
            get
            {
                return this.statusText;
            }
            //这是返回当前的text吗

            set
            {
                if (this.statusText != value)
                {
                    this.statusText = value;

                    ////notify any bound elements that the text has changed
                    //if (this.PropertyChanged != null)
                    //{
                    //    this.PropertyChanged(this, new PropertyChangedEventArgs("StatusText"));
                    //}
                }
            }
            //这是在变化当前的text吗？？？
            //那那个b是啥意思？？（319）
        }

        /// <summary>
        /// Execute shutdown tasks
        /// </summary>
        /// 是时候shutdown了（哭）
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        /// //这是真的看不懂啊，但是应该不是什么高级的东西
        private void MainWindow_Closing(object sender, System.ComponentModel.CancelEventArgs e)
        {
            if (this.multiFrameSourceReader != null)
            {
                // MultiSourceFrameReder is IDisposable
                this.multiFrameSourceReader.Dispose();
                this.multiFrameSourceReader = null;
            }

            if (this.kinectSensor != null)
            {
                this.kinectSensor.Close();
                this.kinectSensor = null;
            }
            //总是是什么没关都关了，参量我也看不懂，大概就那样吧，眼睛又累了
        }


        /// <summary>
        /// Handles the event which the sensor becomes unavailable (E.g. paused, closed, unplugged).
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        //private void Sensor_IsAvailableChanged(object sender, IsAvailableChangedEventArgs e)
        //{
        //    // on failure, set the status text
        //    //this.bStatusText = this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText
        //    //                                                : Properties.Resources.SensorNotAvailableStatusText;
        //}
        //处理突发事件，但是被//了。。。令人忧伤

        private void Reader_MultiSourceFrameArrived(object sender, MultiSourceFrameArrivedEventArgs e)//这e似乎是一个注册的事件
        {

            //读取多源数据，以便于同时处理
            MultiSourceFrame multiSourceFrame = e.FrameReference.AcquireFrame();//获取 排他 有效载荷
            //存下获取的帧吧，应该。。有效帧？

            // If the Frame has expired by the time we process this event, return.
            //如果这个帧失效了就不用处理了，return吧
            if (multiSourceFrame == null)
            {
                return;
            }
            #region 深度视频流处理模块
            bool DepthdataReceived = false;
            using (DepthFrame depthFrame = multiSourceFrame.DepthFrameReference.AcquireFrame())
            {
                if (depthFrame != null)//显然获取了深度元素帧才能做
                {
                    // the fastest way to process the body index data is to directly access 
                    // the underlying buffer
                    //然后这里的函数是复制的吧（吐血）
                    //buffer：缓冲器 来我给你加个buff

                    //深度数帧depthFrame接受事案参数传来的深度帧，然后将其锁住得一帧数据给KinectBuff，
                    //检查数据是否大小匹配，然后对其执行转化函数ProcessDepthFrameDate(),
                    //转化结束即可使用ReaderDepthPixels更新数据
                    using (Microsoft.Kinect.KinectBuffer depthBuffer = depthFrame.LockImageBuffer())
                    {  //app access是啥玩意
                        // verify data and write the color data to the display bitmap
                        //话说这里咋又用了color data呢，可是depthFrame里哪来的color data
                        //（接上）所以彩色data就是在bitmap里面的
                        if (((this.depthFrameDescription.Width * this.depthFrameDescription.Height) == (depthBuffer.Size / this.depthFrameDescription.BytesPerPixel))/*验证未丢失数据吧*/ &&
                            (this.depthFrameDescription.Width == this.displayBitmap.PixelWidth) && (this.depthFrameDescription.Height == this.displayBitmap.PixelHeight)/*且数据没错*/)
                        {
                            // Note: In order to see the full range of depth (including the less reliable far field depth)
                            // we are setting maxDepth to the extreme potential depth threshold
                            ushort maxDepth = ushort.MaxValue;
                            //设立最大阈值啊，sad ，i am very sad
                            // If you wish to filter by reliable depth distance, uncomment the following line:
                            maxDepth = depthFrame.DepthMaxReliableDistance;
                            //我觉得comment挺好的（sad）
                            ProcessDepthFrameData(depthBuffer.UnderlyingBuffer, depthBuffer.Size, depthFrame.DepthMinReliableDistance, maxDepth);
                            //显然这是一个处理数据的函数但是她在哪呢？
                            DepthdataReceived = true;
                        }
                    }
                }
            }
            //if (DepthdataReceived)
            //{
                
            //}
            #endregion



            #region Body流处理
            bool BodydataReceived = false;
            //为啥这里不comment了？
            using (BodyFrame bodyFrame1 = multiSourceFrame.BodyFrameReference.AcquireFrame())
            {
                if (bodyFrame1 != null)
                {
                    if (this.bodies == null)
                    {
                        this.bodies = new Body[bodyFrame1.BodyCount];
                    }
                    //获取骨骼帧啊
                    // The first time GetAndRefreshBodyData is called, Kinect will allocate each Body in the array.
                    // As long as those body objects are not disposed and not set to null in the array,
                    // those body objects will be re-used.
                    //英文单词看的懂，合在一起就看不懂了（^-^)
                    bodyFrame1.GetAndRefreshBodyData(this.bodies);
                    BodydataReceived = true;
                }
            }
            #endregion 

            #region BodyIndex 流处理
            bool BodyindexdataReceived = false;
            using (BodyIndexFrame bodyIndexFrame = multiSourceFrame.BodyIndexFrameReference.AcquireFrame())
            {
                //也就是说bodyindex未索引流对吧
                if (bodyIndexFrame != null)
                {
                    // the fastest way to process the body index data is to directly access 
                    // the underlying buffer
                    using (Microsoft.Kinect.KinectBuffer bodyIndexBuffer = bodyIndexFrame.LockImageBuffer())
                    {
                        // verify data and write the color data to the display bitmap
                        //这个color data又来搞我心态
                        if (((this.depthFrameDescription.Width * this.depthFrameDescription.Height) == bodyIndexBuffer.Size) &&
                            (this.depthFrameDescription.Width == this.displayBitmap.PixelWidth) && (this.depthFrameDescription.Height == this.displayBitmap.PixelHeight))
                        {
                            //判断条件为啥是一样的？
                            //这和depth到底是什么关系？
                            //共用一个坐标系吗？
                            this.ProcessBodyIndexFrameData(bodyIndexBuffer.UnderlyingBuffer, bodyIndexBuffer.Size);
                            BodyindexdataReceived = true;
                            //默认只处理一个人了
                            //猜测这里是简单的基本处理
                        }
                    }
                }
            }

            

            #endregion
            //可惜我不会日语，总之这些是处理数据的了
            //我可能不信，这个函数还没完，多功能数据帧还在搞事。。。
            if (BodydataReceived)
            {


                //int penIndex = 0;//检测到的第几个人
                //可以同时检测到六个人，循环检测到的六个人，不过一般只有第一个
                //now，轮到人体了
                foreach (Body body in this.bodies)
                {

                    if (body.IsTracked)
                    {


                        IReadOnlyDictionary<JointType, Joint> joints = body.Joints;
                        //猜测为joint和jointtype一一对应？？
                        //joint数据，话说这个函数和下面的有啥区别吗
                        Dictionary<JointType, Point> jointPoints = new Dictionary<JointType, Point>();
                        //关节类型和深度空间位置一一对应
                        //建立joint点数据表（字典序）
                        foreach (JointType jointType in joints.Keys)
                        {
                            //对于joint里面的每个数据(这joints还是读取来的呢）
                            // sometimes the depth(Z) of an inferred joint may show as negative
                            // clamp down to 0.1f to prevent coordinatemapper from returning (-Infinity, -Infinity)
                            //所以那个0.1f还是作为修正项啊，话说打出来修真搞得我想找小说来看了
                            CameraSpacePoint position = joints[jointType].Position;
                            //这。。应该是右赋值左吧
                            //是的 每一个jointtype对应一个position
                            if (position.Z < 0)
                            {
                                position.Z = InferredZPositionClamp;
                                //可以可以，这很数据
                            }

                            DepthSpacePoint depthSpacePoint = this.coordinateMapper.MapCameraPointToDepthSpace(position);//将(x,y,z)转换成深度照片中的(x,y)
                            //这函数是自带的，这转换是挺玄学的，原来怎么转的来着忘了
                            jointPoints[jointType] = new Point(depthSpacePoint.X, depthSpacePoint.Y);
                            //所以jointPoints就是深度数据咯
                        }

                        // kalman 滤波估计的手心位置 红色 使用 (x,y,d)预测 CameraSpacePoint
                        var kLp = joints[JointType.HandLeft].Position;
                        //不使用自带的函数，应该吧
                        //深度空间的位置
                        var dLp = this.coordinateMapper.MapCameraPointToDepthSpace(kLp);
                        //这个深度空间位置真是令人sad，所以前面到底转的是啥jointsPoints？
                        var kRp = joints[JointType.HandRight].Position;//经过卡尔曼调整过的
                        var dRp = this.coordinateMapper.MapCameraPointToDepthSpace(kRp);

                        var kLT = joints[JointType.HandTipLeft].Position; //左手指尖
                        var dLT = this.coordinateMapper.MapCameraPointToDepthSpace(kLT);

                        var kRT =joints[JointType.HandTipRight].Position;//右手指尖
                        var dRT = this.coordinateMapper.MapCameraPointToDepthSpace(kRT);


                        //显然这里是画图的大小《^-^》
                        //手势区域
                        float LW_len = 130 - 50 * kLp.Z;
                        float RW_len = 130 - 50 * kRp.Z;
                        //确定的搜索框

                        //我有点明白这图长啥样子了。。。我有点傻，当然是这样
                        Int32Rect Lrect = new Int32Rect((int)(dLp.X - LW_len / 2), (int)(dLp.Y - LW_len / 2), (int)(LW_len), (int)(LW_len));
                        Int32Rect Rrect = new Int32Rect((int)(dRp.X - RW_len / 2), (int)(dRp.Y - RW_len / 2), (int)(RW_len), (int)(RW_len));
                        //不知道减号换加号有没有什么区
                        var handRect = RenderDisplayPixels(Lrect, dLp, Rrect, dRp);
                        //现在画布搭起来了
                        ushort[,] handRectR = DisplayRectR(Rrect, dRp);
                        ushort[,] handRectL = DisplayRectL(Lrect, dLp);
                        //找到了当前二值图像最小的邻接矩阵




                        var shoulder_left = joints[JointType.ShoulderRight].Position;//左肩膀坐标
                        var shoulder_right = joints[JointType.ShoulderLeft].Position;//右肩膀坐标
                        var K_elbow_left = joints[JointType.ElbowLeft].Position;//卡尔曼左手肘
                        var K_elbow_right = joints[JointType.ElbowRight].Position;//卡尔曼右手肘
                        var K_wrist_left = joints[JointType.WristLeft].Position;//卡尔曼左手腕
                        var K_wrist_right = joints[JointType.WristRight].Position;//卡尔曼右手腕
                        var neck = joints[JointType.Neck].Position;//脖子坐标
                        var spin_mid = joints[JointType.SpineMid].Position;//脊柱中心
                        var head = joints[JointType.Head].Position;//头部坐标
                        //不是做手的为啥要知道这么多的坐标啊，真是令人忧伤的故事
                        //目测是copy源码的

                        var SpineMid = this.coordinateMapper.MapCameraPointToDepthSpace(joints[JointType.SpineMid].Position);
                        var SpineBase = this.coordinateMapper.MapCameraPointToDepthSpace(joints[JointType.SpineBase].Position);
                        //  int yaoY=(int)(SpineBase.Y)-30;
                        //spine，脊柱，英语似乎该还需要持续锻炼
                        //哈哈赞同

                        int yaoY = (int)(SpineBase.Y + SpineMid.Y) / 2;
                        //这个腰就很灵性，虽然不知道和手有什么关系
                        //似乎我们可以转姿势识别了
                        //真是泪目


                        double feature1L, feature1R, fi1L, fi1R, feature2L, feature2R, fi2L, fi2R, feature3L, feature3R, fi3L, fi3R, feature4L, feature4R, fi4L, fi4R, r1, r2, r3, r4, r5, cosfi, norm;
                        //这些意义不明的定义让我感到亲切（真是醉了）
                        //根据row977 这些基本上都是向量 共计  大概13个吧
                        List<Double> listFeature = new List<Double>();
                        // CameraSpacePoint spPoint = new CameraSpacePoint();
                        //好吧，listfeature出来时干嘛的？ ？？？？？？？？
                        #region  //这是第一组 左右肩膀到手心 和 脊柱中心到手心的夹角
                        spPoint shoulderLtohand;//左肩膀到手心

                        shoulderLtohand.x = kLp.X - shoulder_left.X;
                        shoulderLtohand.y = kLp.Y - shoulder_left.Y;
                        shoulderLtohand.z = kLp.Z - shoulder_left.Z;
                        //这是在相机空间里对吧

                        spPoint spinLtohand;//脊柱中心到左手心

                        spinLtohand.x = kLp.X - spin_mid.X;
                        spinLtohand.y = kLp.Y - spin_mid.Y;
                        spinLtohand.z = kLp.Z - spin_mid.Z;

                        cosfi = shoulderLtohand.x * spinLtohand.x + shoulderLtohand.y * spinLtohand.y + shoulderLtohand.z * spinLtohand.z;
                        //从定义式上来看应该是模，但是为啥是cosfi？
                        norm = (shoulderLtohand.x * shoulderLtohand.x + shoulderLtohand.y * shoulderLtohand.y + shoulderLtohand.z * shoulderLtohand.z)
                             * (spinLtohand.x * spinLtohand.x + spinLtohand.y * spinLtohand.y + spinLtohand.z * spinLtohand.z);

                        cosfi /= Math.Sqrt(norm);

                        if (cosfi >= 1.0)
                            feature1L = 0;
                        else if (cosfi <= -1.0)
                            feature1L = Math.PI;
                        //真是亲切的求夹角方式
                        else
                        {
                            fi1L = Math.Acos(cosfi);
                            if (180 * fi1L / Math.PI < 180)
                            {
                                feature1L = 180 * fi1L / Math.PI;
                            }
                            else
                            {
                                feature1L = 360 - 180 * fi1L / Math.PI;
                            }
                        }
                        //所以feature1L就是左肩到脊柱的夹角咯
                        listFeature.Add(Math.Round(feature1L, 2));
                        //按照小数点后两位舍入

                        spPoint shoulderRtohand;//右肩膀到手心
                        shoulderRtohand.x = kRp.X - shoulder_right.X;
                        shoulderRtohand.y = kRp.Y - shoulder_right.Y;
                        shoulderRtohand.z = kRp.Z - shoulder_right.Z;


                        spPoint spinRtohand;//脊柱中心到右手心
                        spinRtohand.x = kRp.X - spin_mid.X;
                        spinRtohand.y = kRp.Y - spin_mid.Y;
                        spinRtohand.z = kRp.Z - spin_mid.Z;


                        cosfi = shoulderRtohand.x * spinRtohand.x + shoulderRtohand.y * spinRtohand.y + shoulderRtohand.z * spinRtohand.z;
                        norm = (shoulderRtohand.x * shoulderRtohand.x + shoulderRtohand.y * shoulderRtohand.y + shoulderRtohand.z * shoulderRtohand.z)
                           * (spinRtohand.x * spinRtohand.x + spinRtohand.y * spinRtohand.y + spinRtohand.z * spinRtohand.z);

                        cosfi /= Math.Sqrt(norm);

                        if (cosfi >= 1.0)
                            feature1R = 0;
                        else if (cosfi <= -1.0)
                            feature1R = Math.PI;
                        else
                        {
                            fi1R = Math.Acos(cosfi);
                            if (180 * fi1R / Math.PI < 180)
                            {
                                feature1R = 180 * fi1R / Math.PI;
                            }
                            else
                            {
                                feature1R = 360 - 180 * fi1R / Math.PI;
                            }
                        }

                        listFeature.Add(Math.Round(feature1R, 2));
                        //所以feature1R就是右肩到脊柱的夹角咯
                        #endregion


                        #region//脖子到手肘 脊柱中心到手肘
                        spPoint neckLtoelbow;
                        neckLtoelbow.x = K_elbow_left.X - neck.X;
                        neckLtoelbow.y = K_elbow_left.Y - neck.Y;
                        neckLtoelbow.z = K_elbow_left.Z - neck.Z;

                        spPoint spinLtoelbow;
                        spinLtoelbow.x = K_elbow_left.X - spin_mid.X;
                        spinLtoelbow.y = K_elbow_left.Y - spin_mid.Y;
                        spinLtoelbow.z = K_elbow_left.Z - spin_mid.Z;

                        cosfi = neckLtoelbow.x * spinLtoelbow.x + neckLtoelbow.y * spinLtoelbow.y + neckLtoelbow.z * spinLtoelbow.z;
                        norm = (neckLtoelbow.x * neckLtoelbow.x + neckLtoelbow.y * neckLtoelbow.y + neckLtoelbow.z * neckLtoelbow.z)
                             * (spinLtoelbow.x * spinLtoelbow.x + spinLtoelbow.y * spinLtoelbow.y + spinLtoelbow.z * spinLtoelbow.z);

                        cosfi /= Math.Sqrt(norm);

                        if (cosfi >= 1.0)
                            feature2L = 0;
                        else if (cosfi <= -1.0)
                            feature2L = Math.PI;

                        else
                        {
                            fi2L = Math.Acos(cosfi);
                            if (180 * fi2L / Math.PI < 180)
                            {
                                feature2L = 180 * fi2L / Math.PI;
                            }
                            else
                            {
                                feature2L = 360 - 180 * fi2L / Math.PI;
                            }
                        }
                        listFeature.Add(Math.Round(feature2L, 2));



                        spPoint neckRtoelbow;
                        neckRtoelbow.x = K_elbow_right.X - neck.X;
                        neckRtoelbow.y = K_elbow_right.Y - neck.Y;
                        neckRtoelbow.z = K_elbow_right.Z - neck.Z;

                        spPoint spinRtoelbow;
                        spinRtoelbow.x = K_elbow_right.X - spin_mid.X;
                        spinRtoelbow.y = K_elbow_right.Y - spin_mid.Y;
                        spinRtoelbow.z = K_elbow_right.Z - spin_mid.Z;

                        cosfi = neckRtoelbow.x * spinRtoelbow.x + neckRtoelbow.y * spinRtoelbow.y + neckRtoelbow.z * spinRtoelbow.z;
                        norm = (neckRtoelbow.x * neckRtoelbow.x + neckRtoelbow.y * neckRtoelbow.y + neckRtoelbow.z * neckRtoelbow.z)
                             * (spinRtoelbow.x * spinRtoelbow.x + spinRtoelbow.y * spinRtoelbow.y + spinRtoelbow.z * spinRtoelbow.z);

                        cosfi /= Math.Sqrt(norm);

                        if (cosfi >= 1.0)
                            feature2R = 0;
                        else if (cosfi <= -1.0)
                            feature2R = Math.PI;

                        else
                        {
                            fi2R = Math.Acos(cosfi);
                            if (180 * fi2R / Math.PI < 180)
                            {
                                feature2R = 180 * fi2R / Math.PI;
                            }
                            else
                            {
                                feature2R = 360 - 180 * fi2R / Math.PI;
                            }
                        }
                        listFeature.Add(Math.Round(feature2R, 2));


                        #endregion
                        //虽然不知道有什么用，为什么要这样划分角度呢，但是一定是有道理的吧，明天画张图看看到底哪些角变成了向量了
                        //另外我觉得我这里的波浪线绝对是很古怪啊，有什么用啊？是不是少了什么结构体的定义啊？我想放到明天去翻了
                        #region//肩到手肘 手肘到手心
                        spPoint shoulderLtoelbow;
                        shoulderLtoelbow.x = K_elbow_left.X - shoulder_left.X;
                        shoulderLtoelbow.y = K_elbow_left.Y - shoulder_left.Y;
                        shoulderLtoelbow.z = K_elbow_left.Z - shoulder_left.Z;

                        spPoint elbowLtohand; //手肘到手心
                        elbowLtohand.x = kLp.X - K_elbow_left.X;
                        elbowLtohand.y = kLp.Y - K_elbow_left.Y;
                        elbowLtohand.z = kLp.Z - K_elbow_left.Z;

                        cosfi = shoulderLtoelbow.x * elbowLtohand.x + shoulderLtoelbow.y * elbowLtohand.y + shoulderLtoelbow.z * elbowLtohand.z;
                        norm = (shoulderLtoelbow.x * shoulderLtoelbow.x + shoulderLtoelbow.y * shoulderLtoelbow.y + shoulderLtoelbow.z * shoulderLtoelbow.z)
                             * (elbowLtohand.x * elbowLtohand.x + elbowLtohand.y * elbowLtohand.y + elbowLtohand.z * elbowLtohand.z);

                        cosfi /= Math.Sqrt(norm);

                        if (cosfi >= 1.0)
                            feature3L = 0;
                        else if (cosfi <= -1.0)
                            feature3L = Math.PI;

                        else
                        {
                            fi3L = Math.Acos(cosfi);
                            if (180 * fi3L / Math.PI < 180)
                            {
                                feature3L = 180 * fi3L / Math.PI;
                            }
                            else
                            {
                                feature3L = 360 - 180 * fi3L / Math.PI;
                            }
                        }
                        listFeature.Add(Math.Round(feature3L, 2));


                        spPoint shoulderRtoelbow;
                        shoulderRtoelbow.x = K_elbow_right.X - shoulder_right.X;
                        shoulderRtoelbow.y = K_elbow_right.Y - shoulder_right.Y;
                        shoulderRtoelbow.z = K_elbow_right.Z - shoulder_right.Z;

                        spPoint elbowRtohand; //手肘到手心
                        elbowRtohand.x = kRp.X - K_elbow_right.X;
                        elbowRtohand.y = kRp.Y - K_elbow_right.Y;
                        elbowRtohand.z = kRp.Z - K_elbow_right.Z;

                        cosfi = shoulderRtoelbow.x * elbowRtohand.x + shoulderRtoelbow.y * elbowRtohand.y + shoulderRtoelbow.z * elbowRtohand.z;
                        norm = (shoulderRtoelbow.x * shoulderRtoelbow.x + shoulderRtoelbow.y * shoulderRtoelbow.y + shoulderRtoelbow.z * shoulderRtoelbow.z)
                             * (elbowRtohand.x * elbowRtohand.x + elbowRtohand.y * elbowRtohand.y + elbowRtohand.z * elbowRtohand.z);

                        cosfi /= Math.Sqrt(norm);

                        if (cosfi >= 1.0)
                            feature3R = 0;
                        else if (cosfi <= -1.0)
                            feature3R = Math.PI;

                        else
                        {
                            fi3R = Math.Acos(cosfi);
                            if (180 * fi3R / Math.PI < 180)
                            {
                                feature3R = 180 * fi3R / Math.PI;
                            }
                            else
                            {
                                feature3R = 360 - 180 * fi3R / Math.PI;
                            }
                        }
                        listFeature.Add(Math.Round(feature3R, 2));


                        #endregion



                        #region //手腕到手心 脖子到手腕
                        spPoint wristLtohand;
                        wristLtohand.x = kLp.X - K_wrist_left.X;
                        wristLtohand.y = kLp.Y - K_wrist_left.Y;
                        wristLtohand.z = kLp.Z - K_wrist_left.Z;

                        spPoint neckLtowrist;
                        neckLtowrist.x = K_wrist_left.X - neck.X;
                        neckLtowrist.y = K_wrist_left.Y - neck.Y;
                        neckLtowrist.z = K_wrist_left.Z - neck.Z;


                        cosfi = wristLtohand.x * neckLtowrist.x + wristLtohand.y * neckLtowrist.y + wristLtohand.z * neckLtowrist.z;
                        norm = (wristLtohand.x * wristLtohand.x + wristLtohand.y * wristLtohand.y + wristLtohand.z * wristLtohand.z)
                             * (neckLtowrist.x * neckLtowrist.x + neckLtowrist.y * neckLtowrist.y + neckLtowrist.z * neckLtowrist.z);


                        cosfi /= Math.Sqrt(norm);

                        if (cosfi >= 1.0)
                            feature4L = 0;
                        else if (cosfi <= -1.0)
                            feature4L = Math.PI;

                        else
                        {
                            fi4L = Math.Acos(cosfi);
                            if (180 * fi4L / Math.PI < 180)
                            {
                                feature4L = 180 * fi4L / Math.PI;
                            }
                            else
                            {
                                feature4L = 360 - 180 * fi4L / Math.PI;
                            }
                        }
                        listFeature.Add(Math.Round(feature4L, 2));





                        spPoint wristRtohand;
                        wristRtohand.x = kRp.X - K_wrist_right.X;
                        wristRtohand.y = kRp.Y - K_wrist_right.Y;
                        wristRtohand.z = kRp.Z - K_wrist_right.Z;

                        spPoint neckRtowrist;
                        neckRtowrist.x = K_wrist_right.X - neck.X;
                        neckRtowrist.y = K_wrist_right.Y - neck.Y;
                        neckRtowrist.z = K_wrist_right.Z - neck.Z;


                        cosfi = wristRtohand.x * neckRtowrist.x + wristRtohand.y * neckRtowrist.y + wristRtohand.z * neckRtowrist.z;
                        norm = (wristRtohand.x * wristRtohand.x + wristRtohand.y * wristRtohand.y + wristRtohand.z * wristRtohand.z)
                             * (neckRtowrist.x * neckRtowrist.x + neckRtowrist.y * neckRtowrist.y + neckRtowrist.z * neckRtowrist.z);


                        cosfi /= Math.Sqrt(norm);

                        if (cosfi >= 1.0)
                            feature4R = 0;
                        else if (cosfi <= -1.0)
                            feature4R = Math.PI;

                        else
                        {
                            fi4R = Math.Acos(cosfi);
                            if (180 * fi4R / Math.PI < 180)
                            {
                                feature4R = 180 * fi4R / Math.PI;
                            }
                            else
                            {
                                feature4R = 360 - 180 * fi4R / Math.PI;
                            }
                        }
                        listFeature.Add(Math.Round(feature4R, 2));

                        #endregion

                        //现在长度开始数字化了
                        #region //各个向量模比例计算

                        //脊柱中心到头部向量长度
                        double a = Math.Sqrt((head.X - spin_mid.X) * (head.X - spin_mid.X) + (head.Y - spin_mid.Y) * (head.Y - spin_mid.Y) + (head.Z - spin_mid.Z) * (head.Z - spin_mid.Z));
                        //

                        double b = Math.Sqrt((kLp.X - head.X) * (kLp.X - head.X) + (kLp.Y - head.Y) * (kLp.Y - head.Y) + (kLp.Z - head.Z) * (kLp.Z - head.Z));
                        //头部到右手心的向量长度
                        double c = Math.Sqrt((kRp.X - head.X) * (kRp.X - head.X) + (kRp.Y - head.Y) * (kRp.Y - head.Y) + (kRp.Z - head.Z) * (kRp.Z - head.Z));
                        //脊柱中心到左手的向量长度

                        double d = Math.Sqrt((kLp.X - spin_mid.X) * (kLp.X - spin_mid.X) + (kLp.Y - spin_mid.Y) * (kLp.Y - spin_mid.Y) + (kLp.Z - spin_mid.Z) * (kLp.Z - spin_mid.Z));

                        //脊柱中心到右手的向量长度
                        double e1 = Math.Sqrt((kRp.X - spin_mid.X) * (kRp.X - spin_mid.X) + (kRp.Y - spin_mid.Y) * (kRp.Y - spin_mid.Y) + (kRp.Z - spin_mid.Z) * (kRp.Z - spin_mid.Z));


                        // 左右手心距离
                        double f = Math.Sqrt((kRp.X - kLp.X) * (kRp.X - kLp.X) + (kRp.Y - kLp.Y) * (kRp.Y - kLp.Y) + (kRp.Z - kLp.Z) * (kRp.Z - kLp.Z)); ;

                        //将数按指定的位数舍入
                        r1 = Math.Round(b / a, 2);
                        r2 = Math.Round(c / a, 2);
                        r3 = Math.Round(d / a, 2);
                        r4 = Math.Round(e1 / a, 2);
                        r5 = Math.Round(f / a, 2);

                        //另外你这里为啥是e1啊，注册事件用的e吗？太扯了吧，太不和谐了吧
                        listFeature.Add(r1);
                        listFeature.Add(r2);
                        listFeature.Add(r3);
                        listFeature.Add(r4);
                        listFeature.Add(r5);
                        //OK，现在我们的向量组又多了5个
                        #endregion




                        #region
                        //手心和手腕的相对位置
                        var K_wrist_leftD = this.coordinateMapper.MapCameraPointToDepthSpace(K_wrist_left);//转换到深度图像
                        if (K_wrist_leftD.Y - dLp.Y >= 0)
                            //如果左手腕在右手腕的右边（相对一张图片里）
                            listFeature.Add(1);
                        else
                            listFeature.Add(0);




                        var K_wrist_rightD = this.coordinateMapper.MapCameraPointToDepthSpace(K_wrist_right);//转换到深度图像
                        if (K_wrist_rightD.Y - dRp.Y >= 0)
                            listFeature.Add(1);
                        else
                            listFeature.Add(0);
                        //这是，表示正反？应该吧，筚路蓝缕此言不虚
                        #endregion
                        // ushort[,] handRectShow = DisplayOnlyhand(Lrect,dLp,kLp,Rrect,dRp,kRp);
                        ushort[,] handRectShow = DisplayOnlyhand(Lrect, dLp, kLp, Rrect, dRp, kRp, yaoY);


                        //这里加个腰是啥子意思哟
                        //同，but handRectShow是两只手的矩阵的意思
                        //听起来还是不习惯


                        if (recordFlag)
                        {
                            if (BodyindexdataReceived)
                            {
                                this.displayBitmap.WritePixels
                               (
                               new Int32Rect(0, 0, this.displayBitmap.PixelWidth, this.displayBitmap.PixelHeight),
                               this.bodyIndexPixels,
                               this.displayBitmap.PixelWidth * (int)BytesPerPixel,
                               0
                               );
                                //每一帧4字节 
                            }
                            //这里注释的我很忧伤

                            if (this.displayBitmap != null)
                            {
                                // create a png bitmap encoder which knows how to save a .png file
                                BitmapEncoder encoder = new PngBitmapEncoder();

                                // create frame from the writable bitmap and add to encoder
                                encoder.Frames.Add(BitmapFrame.Create(this.displayBitmap));

                                

                                //string myPhotos = Environment.GetFolderPath(Environment.SpecialFolder.MyPictures);
                                string myPhotos = @"D:/kinectkeep/";
                                string path = Path.Combine(myPhotos, "KinectScreenshot-BodyIndex-" + time + ".png");
                                time++;
                                // write the new file to disk
                                
                                    // FileStream is IDisposable
                                    using (FileStream fs = new FileStream(path, FileMode.Create))
                                    {
                                        encoder.Save(fs);
                                    }         

                            }

                            //深度feature增加了啊，但是这些函数是什么鬼，莫非还是关我的事（哭）！！！
                            // gestureFeature.AddFeature(displayBitmap, kLp, kRp, Lrect, Rrect);
                            gestureFeature.AddFeature(kLp, kRp, handRect);

                            gestureFeature.AddFeature1(dLT);


                            gestureFeature.AddFeature(handRectR);
                            gestureFeature.AddFeature1(handRectShow);
                            var yaoV = (joints[JointType.SpineMid].Position.Y + joints[JointType.SpineBase].Position.Y) / 2;
                            //脊椎中部和脊椎基部的中点？？？
                            gestureFeature.setSpineBase(yaoV);

                            gestureFeature.AddFeaturesInFrame(listFeature);
                        }
                    }
                }

            }
        }

        //11：30 洗澡去了 byebye
        private ushort[,] DisplayOnlyhand(Int32Rect Lrect, DepthSpacePoint dLp, CameraSpacePoint kLp, Int32Rect Rrect, DepthSpacePoint dRp, CameraSpacePoint kRp, int yaoY)
        {
            //这个函数应该是用来显示手的。。。吧
            //我来看看到底腰是干嘛用的
            double dths = 0.07;//深度阈值
            int width = this.displayWidth;
            int height = this.displayHeight;
            float LW_len = 130 - 50 * kLp.Z;
            float RW_len = 130 - 50 * kRp.Z;
            //显示区域的大小
            ushort[,] displayHand = new ushort[width, height];

            for (int i = (int)(dLp.X - LW_len / 2); i <= (int)(dLp.X + LW_len / 2); i++)
            {
                //double转int，对每一个宽度遍历
                //width
                for (int j = (int)(dLp.Y - LW_len / 2); j <= (int)(dLp.Y + LW_len / 2); j++) 
                {
                    //height
                    if (j < yaoY)
                    {
                        //这是啥意思？在下半身？？？？
                        int index = j * this.displayWidth + i;
                        int hand_index = (int)(dLp.Y) * this.displayWidth + (int)(dLp.X);
                        if (overPixels(index) || overPixels(hand_index))
                            continue;
                        //overPixel是干嘛的？检测越界？？？？？好像的确是的
                        var diff = Math.Abs(cameraPoints[index].Z - cameraPoints[hand_index].Z);
                        //两者为同一个点，话说为啥要这样呢，因为double转int的原因？
                        if (diff < dths && bodyIndexPixels[index] != 0x00000000)
                        {
                            displayHand[i, j] = 1;
                            //左手区域，此点为1
                        }
                    }
                }
            } //for


            for (int i = (int)(dRp.X - RW_len / 2); i <= (int)(dRp.X + RW_len / 2); i++)
            {//width
                for (int j = (int)(dRp.Y - RW_len / 2); j <= (int)(dRp.Y + RW_len / 2); j++) //height
                {
                    if (j < yaoY)
                    {
                        int index = j * this.displayWidth + i;
                        int hand_index = (int)(dRp.Y) * this.displayWidth + (int)(dRp.X);
                        if (overPixels(index) || overPixels(hand_index))
                            continue;
                        var diff = Math.Abs(cameraPoints[index].Z - cameraPoints[hand_index].Z);
                        if (diff < dths)
                        {
                            displayHand[i, j] = 1;
                            //等等这个真的不会把上面的信息覆盖掉吗？？？？？
                            //也就是说dRp.X-RW_len/2>dLp.X+RW_len/2..或者反过来？？
                        }
                    }
                }
            } //for




            return displayHand;
        }

        #region
        // 画手心位置  去掉了private
        //话说去掉了private是啥意思，可以去给别的函数调用来画图了？谁调用啊，你后面咋不去掉private了？
        void DrawHand(HandState handState, Point handPosition, DrawingContext drawingContext)
        {
            switch (handState)
            {
                case HandState.Closed:
                    drawingContext.DrawEllipse(this.handClosedBrush, null, handPosition, HandSize, HandSize);
                    break;

                case HandState.Open:
                    drawingContext.DrawEllipse(this.handOpenBrush, null, handPosition, HandSize, HandSize);
                    break;

                case HandState.Lasso:
                    drawingContext.DrawEllipse(this.handLassoBrush, null, handPosition, HandSize, HandSize);
                    break;

                default:
                    drawingContext.DrawEllipse(this.handOpenBrush, null, handPosition, HandSize, HandSize);
                    //话说你这画的是圆吧
                    break;
                    //默认是open的吗，手也就这几种了。。。
            }

        }

        // Draws a body 画出身体
        private void DrawBody(IReadOnlyDictionary<JointType, Joint> joints, IDictionary<JointType, Point> jointPoints, DrawingContext drawingContext, Pen drawingPen)
        {
            #region 画出骨骼连线
            // Draw the bones 
            foreach (var bone in this.bones)
            {
                this.DrawBone(joints, jointPoints, bone.Item1, bone.Item2, drawingContext, drawingPen);
            }
            #endregion

            #region 画出骨骼节点 已注释
            //Draw the joints
            foreach (JointType jointType in joints.Keys)
            {
                Brush drawBrush = null;

                TrackingState trackingState = joints[jointType].TrackingState;

                if (trackingState == TrackingState.Tracked)
                {
                    drawBrush = this.trackedJointBrush;
                }
                else if (trackingState == TrackingState.Inferred)
                {
                    drawBrush = this.inferredJointBrush;
                }

                if (drawBrush != null)
                {
                    drawingContext.DrawEllipse(drawBrush, null, jointPoints[jointType], JointThickness, JointThickness);
                }
            }
            #endregion
        }

        // Draws one bone of a body (joint to joint) 被上一个函数调用
        //这个this看的真让人忧伤
        private void DrawBone(IReadOnlyDictionary<JointType, Joint> joints, IDictionary<JointType, Point> jointPoints, JointType jointType0, JointType jointType1, DrawingContext drawingContext, Pen drawingPen)
        {
            Joint joint0 = joints[jointType0];
            Joint joint1 = joints[jointType1];

            // If we can't find either of these joints, exit
            if (joint0.TrackingState == TrackingState.NotTracked ||
                joint1.TrackingState == TrackingState.NotTracked)
            {
                return;
            }

            // We assume all drawn bones are inferred unless BOTH joints are tracked
            Pen drawPen = this.inferredBonePen;
            if ((joint0.TrackingState == TrackingState.Tracked) && (joint1.TrackingState == TrackingState.Tracked))
            {
                drawPen = drawingPen;
            }
            //可以理解，但还是感觉有点不对劲
            //没钱玩什么fgo
            drawingContext.DrawLine(drawPen, jointPoints[jointType0], jointPoints[jointType1]);
            //Pen ：描述如何绘制形状的轮廓
        }

        private void DrawClippedEdges(Body body, DrawingContext drawingContext)
        {
            //画人体边缘吗？？大概是，我想去学日语了
            FrameEdges clippedEdges = body.ClippedEdges;
            //这个边缘定义的好高级的样子
            if (clippedEdges.HasFlag(FrameEdges.Bottom))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, this.displayHeight - ClipBoundsThickness, this.displayWidth, ClipBoundsThickness));
            }
            
            if (clippedEdges.HasFlag(FrameEdges.Top))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, this.displayWidth, ClipBoundsThickness));
            }

            if (clippedEdges.HasFlag(FrameEdges.Left))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, ClipBoundsThickness, this.displayHeight));
            }

            if (clippedEdges.HasFlag(FrameEdges.Right))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(this.displayWidth - ClipBoundsThickness, 0, ClipBoundsThickness, this.displayHeight));
            }//这四个我没看懂我承认，问题在于FrameEdge是怎么理解的
        }

        //保存图片太卡，不如直接录视频
        private void SavePic()
        {
            var rtb = new RenderTargetBitmap(displayWidth, displayHeight, 96, 96, PixelFormats.Default);
            //rtb.Render(needPic);
            //needPic是啥我也不知道
            BitmapEncoder encoder = new PngBitmapEncoder();
            // create frame from the writable bitmap and add to encoder
            encoder.Frames.Add(BitmapFrame.Create(rtb));

            string time = System.DateTime.UtcNow.ToString("hh'-'mm'-'ss", CultureInfo.CurrentUICulture.DateTimeFormat);
            string myPhotos = @"D:\kinectkeep\";

            string path = System.IO.Path.Combine(myPhotos, "Gesture-" + time + ".png");
            //我服了，以上的应该是读取图片了，可惜还是看不懂
            // write the new file to disk
            try
            {
                // FileStream is IDisposable
                using (FileStream fs = new FileStream(path, FileMode.Create))
                {
                    encoder.Save(fs);
                }
                //保存图片到电脑，啥玩意没看懂
                //this.StatusText = string.Format(CultureInfo.CurrentCulture, Properties.Resources.SavedScreenshotStatusTextFormat, path);
            }
            catch (IOException)
            {
                //this.StatusText = string.Format(CultureInfo.CurrentCulture, Properties.Resources.FailedScreenshotStatusTextFormat, path);
            }


        }

        //对每一帧深度图像可视化
        private unsafe void ProcessDepthFrameData(IntPtr depthFrameData, uint depthFrameDataSize, ushort minDepth, ushort maxDepth)
        {
            //处理深度数据帧 maxDepth和minDepth 为深度阈值吧
            //这个unsafe是啥意思不知道，为了使用指针吗
            // depth frame data is a 16 bit value
            ushort* frameData = (ushort*)depthFrameData;
            //强制深度帧数据转为ushort型数组，frameDate指针指向它


            var dPixels = new ushort[this.depthFrameDescription.Width * this.depthFrameDescription.Height];
            //像素大小
            for (int i = 0; i < dPixels.Length; ++i)
                dPixels[i] = frameData[i];
            //将深度空间转为相机空间
            this.coordinateMapper.MapDepthFrameToCameraSpace(dPixels, cameraPoints);

            //深度空间转换为彩色空间
            //为啥要转彩色空间呢
            this.coordinateMapper.MapDepthFrameToColorSpace(dPixels, colorPoints);
            //是的 其实并无用

            // convert depth to a visual representation
            //将深度图像可视化
            for (int i = 0; i < (int)(depthFrameDataSize / this.depthFrameDescription.BytesPerPixel); ++i)
            {
                //深度帧各深度（像素）点逐个转化为灰度值
                // Get the depth for this pixel
                ushort depth = frameData[i];

                // To convert to a byte, we're mapping the depth value to the byte range.
                // Values outside the reliable depth range are mapped to 0 (black).

                if (depth < minDepth)
                {
                    this.depthPixels[i] = 0;
                }
                else if (depth >= maxDepth)
                {
                    this.depthPixels[i] = 255;
                }
                else
                {
                    this.depthPixels[i] = (byte)(depth / MapDepthToByte);
                }
                //大概明白，完全不懂了
                //大概是将每个像素点13位深度数据转换为对应0~256
                //this.depthPixels[i] = (byte)(depth >= minDepth && depth <= maxDepth ? (depth / MapDepthToByte) : 0);
            }
        }


        //private void RenderDepthPixels()
        //{
        //    this.depthBitmap.WritePixels(
        //        new Int32Rect(0, 0, this.depthBitmap.PixelWidth, this.depthBitmap.PixelHeight),
        //        this.depthPixels,
        //        this.depthBitmap.PixelWidth,
        //        0);
        //}


        private unsafe void ProcessBodyIndexFrameData(IntPtr bodyIndexFrameData, uint bodyIndexFrameDataSize)
        {
            byte* frameData = (byte*)bodyIndexFrameData;
            //好想八卦
            // convert body index to a visual representation
            //可视化
            for (int i = 0; i < (int)bodyIndexFrameDataSize; ++i)
            {
                // the BodyColor array has been sized to match
                // BodyFrameSource.BodyCount
                if (frameData[i] < BodyColor.Length)
                {
                    // this pixel is part of a player,
                    // display the appropriate color
                    this.bodyIndexPixels[i] = BodyColor[frameData[i]];
                }
                else
                {
                    // this pixel is not part of a player
                    // display black
                    this.bodyIndexPixels[i] = 0x00000000;
                    //果然是位图啊
                }
            }
        }

        //private void RenderBodyIndexPixels()
        //{
        //    this.bodyIndexBitmap.WritePixels(
        //        new Int32Rect(0, 0, this.bodyIndexBitmap.PixelWidth, this.bodyIndexBitmap.PixelHeight),
        //        this.bodyIndexPixels,
        //        this.bodyIndexBitmap.PixelWidth * (int)BytesPerPixel,
        //        0);
        //}


        #endregion
        // 主要修改这个函数


        //测试cameraPoints
        //private ushort[,] DisplayRectRcolor(Int32Rect rRect,DepthSpacePoint rightHanP) {

        //ths是角度的意思吗

        //    double dths = 0.07;//深度阈值
        //    uint[] rdata = new uint[rRect.Width * rRect.Height];
        //    ushort[,] rd = new ushort[rRect.Width, rRect.Height];
        //    for (int i = 0; i < rRect.Width; ++i)
        //    {
        //        for (int j = 0; j < rRect.Height; ++j)
        //        {
        //            int index = (j + rRect.Y) * this.displayWidth + i + rRect.X;
        //            int hand_index = (int)(rightHanP.Y) * this.displayWidth + (int)(rightHanP.X);
        //            if (overPixels(index) || overPixels(hand_index))
        //                continue;
        //            var diff = Math.Abs(cameraPoints[index].Z - cameraPoints[hand_index].Z);

        //              ColorSpacePoint p=colorPoints[index];
        //            int colorX=(int)Math.Floor(p.X+0.5);
        //            int colorY=(int)Math.Floor(p.Y+0.5);
        //            int colorIndex = (colorX + colorY * colorFrameDescription.Width);
        //            //加入肤色模型
        //所以这部分到底有没有被废弃掉呢？


        //            //if (diff > dths)
        //            //    displayPixels[index] = 0xFFFFFFFF;
        //            if (diff < dths && bodyIndexPixels[index] != 0x00000000)
        //            {
        //                rd[i, j] = 1;
        //                //ldata[j * lRect.Width + i] = 0x0000FF00;
        //            }

        //        }
        //    }

        //}


        private ushort[,] DisplayRectR(Int32Rect rRect, DepthSpacePoint rightHanP/*二维深度空间里的点*/)
        {
            double dths = 0.07;//深度阈值
            //int len = this.depthPixels.Length;
            //uint[] displayPixels = this.bodyIndexPixels;
            //这个被划掉的部分好像在哪里看到过？？？1040？
            uint[] rdata = new uint[rRect.Width * rRect.Height];
            ushort[,] rd = new ushort[rRect.Width, rRect.Height];
            ushort[,] rdcopy = new ushort[rRect.Width, rRect.Height];
            for (int i = 0; i < rRect.Width; ++i)
            {
                for (int j = 0; j < rRect.Height; ++j)
                {
                    int index = (j + rRect.Y) * this.displayWidth + i + rRect.X;
                    int hand_index = (int)(rightHanP.Y) * this.displayWidth + (int)(rightHanP.X);
                    if (overPixels(index) || overPixels(hand_index))
                        continue;
                    var diff = Math.Abs(cameraPoints[index].Z - cameraPoints[hand_index].Z);
                    //if (diff > dths)
                    //    displayPixels[index] = 0xFFFFFFFF;
                    if (diff < dths && bodyIndexPixels[index] != 0x00000000)
                    {
                        rd[i, j] = 1;
                        //ldata[j * lRect.Width + i] = 0x0000FF00;
                    }

                }
            }
            //rd表示当前点是否是黑的？
            //好像就是抄的1040！___
            //函数可以加在这里处理8连通域

            bool[,] arr = new bool[rRect.Width, rRect.Height];


            for (int p = 0; p < rRect.Width; p++) //从零列开始
            {
                for (int q = 0; q < rRect.Height; q++)
                {
                    if (rd[p, q] == 1)
                        //把rd转换为arr的true和false??干嘛呢这是在
                        arr[p, q] = true;
                    else
                        arr[p, q] = false;
                }
            }

            //arr数组和rd数组其实一样的吧
            //如果1和true等价
            //bingo

            List<List<Pointt>> ans = get(arr);
            //所以图啥呢？去噪？
            //同问2333
            //翻了下论文 是的（连通域面积小于300的当做噪声去掉）

            int tot = 0;

            int max = 0;

            bool[,] temple = new bool[rRect.Width, rRect.Height];//每一步的临时数组
            bool[,] temple1 = new bool[rRect.Width, rRect.Height];//最终的临时数组
            //templelast
            //可临时数组不是temparray吗
            foreach (List<Pointt> c in ans)
            {
                for (int p = 0; p < rRect.Width; p++)
                {
                    for (int q = 0; q < rRect.Height; q++)
                    {
                        temple[p, q] = false;
                    }
                }
                //先初始化为false
                tot = 0;

                foreach (Pointt p in c)
                {
                    temple[p.x, p.y] = true;
                    ++tot;
                }
                //计数一共多少个点 在连通域内的temple标记为true

                if (max < tot)
                {
                    max = tot;
                    for (int i = 0; i < arr.GetLength(0); i++)
                    {
                        for (int j = 0; j < arr.GetLength(1); j++)
                        {
                            temple1[i, j] = temple[i, j];
                        }

                    }
                    tot = 0;
                }
            }//foreach (List<Pointt> c in ans)
            //成了个0/1图了，不，数组了 即邻接矩阵？？

            ushort[,] rdnew = new ushort[rRect.Width, rRect.Height];
            //我终于感觉我有点看懂C#这个语言了（哭）
            for (int p = 0; p < rRect.Width; p++)
            {
                for (int q = 0; q < rRect.Height; q++)
                {

                    if (temple1[p, q] == true)
                        rdnew[p, q] = 1;
                    else
                        rdnew[p, q] = 0;
                }
            }
            //复制到rdnew上干嘛的？
            //邻接矩阵常规操作

            int rwidthRefresh = rRect.Width;//暂时还未更新
            int rheightRefresh = rRect.Height;//兄弟我还不知道你要更新啥
            
            int countleft = 0;//要不要来个countright？
            //还真有
            Boolean flagleft = false;
            //PS：从这里开始的四个不好意思没看懂，似乎是检测有没有什么问题的，从四个边界检测？？
            //这些是来统计从各个方向数第几个向量开始有元素的，sx
            for (int m = 0; m < rRect.Width; m++) //从零列开始
            {
                for (int n = 0; n < rRect.Height; n++) //从这里开始ld改成了ldnew
                {

                    if (rdnew[n, m] != 0)//ld
                    {
                        //按1算。。行数减一就是所求   
                        flagleft = true;
                        break;
                    }
                }
                if (flagleft == true)
                    break;
                else
                    countleft++;
                //检测从左第一列开始到第几列开始有元素
            }
            

            int counttop = 0;
            Boolean flagtop = false;
            for (int i = 0; i < rRect.Height; i++) //从零行开始
            {
                for (int j = 0; j < rRect.Width; j++)
                {
                    if (rdnew[i, j] != 0) //ld
                    {
                        flagtop = true;
                        break;

                    }

                }
                if (flagtop == true)
                    break;
                else
                    counttop++;

            }

            int countright = 0;
            Boolean flagright = false;
            for (int p = rRect.Width - 1; p >= 0; p--)  //从最后一列开始
            {

                for (int q = 0; q < rRect.Height; q++)
                {
                    if (rdnew[q, p] != 0) //ld pq
                    {
                        flagright = true;
                        break;

                    }
                }
                if (flagright == true)
                    break;
                else
                    countright++;
                //从右边起第一列开始到第几列有元素

            }


            int counttail = 0;
            Boolean flagtail = false;                        //从最底行开始
            for (int p = rRect.Height - 1; p >= 0; p--)
            {
                for (int q = 0; q < rRect.Width; q++)
                {
                    if (rdnew[p, q] != 0)//ld
                    {
                        flagtail = true;
                        break;

                    }
                }
                if (flagtail == true)
                    break;
                else
                    counttail++;
            }



            ////再次对左边界进行处理

            //int countleft2 = 0;
            //Boolean flagleft2 = false;
            //for (int m = 0; m < lRect.Width; m++) //从零列开始
            //{
            //    for (int n = 0; n < lRect.Height; n++) //从这里开始ld改成了ldnew
            //    {

            //        if (ldnew[n, m] != 0)//ld
            //        {
            //            //按1算。。行数减一就是所求   
            //            flagleft2 = true;
            //            break;
            //        }
            //    }
            //    if (flagleft2 == true)
            //        break;
            //    else
            //        countleft2++;
            //}

            ushort[,] rdnew1;
            //ushort[,] ldnew1;
            if (rRect.Width - countright - countleft < 0 || rRect.Height - counttop - counttail < 0)
            {
                rdnew1 = null;
            }
            //此时没有元素
            else
            {
                rdnew1 = new ushort[rRect.Width - countright - countleft, rRect.Height - counttop - counttail];
                //掌握核心力量
                //嗯！
                // ushort[,] ldnew1 = new ushort[lRect.Width-countright-countleft, lRect.Height-counttop-counttail];//以前的ldnew改成了ldnew1

                if (rRect.Width - countright - countleft >= 0 && rRect.Height - counttop - counttail >= 0)
                {
                    //这个if加上去是干嘛的???有用？
                    for (int a = 0; a < rRect.Width - countright - countleft; a++) //lRect.Height - counttop-
                    {
                        for (int b = 0; b < rRect.Height - counttop - counttail; b++) // lRect.Width - countright - countleft
                        {
                            //这注释是不是打错了什么。
                            rdnew1[a, b] = rdnew[b + counttop, a + countleft];//分别为以前的ldnew 和ld

                            //if (counttop == 0 && countleft == 0)
                            //    ldnew[a, b] = ld[a + counttop, b + countleft];
                            //else if (counttop == 0)
                            //    ldnew[a, b] = ld[a + counttop, b + countleft - 1];
                            //else if (countleft == 0)
                            //    ldnew[a, b] = ld[a + counttop - 1, b + countleft];
                            //else
                            //    ldnew[a, b] = ld[a + counttop - 1, b + countleft - 1];
                        }
                    }
                }
                //这是搞啥，平移吗
                //else
                //    ldnew1 = null;
            }
            return rdnew1; //这里不再返回ld而是一个更小范围的矩形，//这倒是，平移
        }



        private ushort[,] DisplayRectL(Int32Rect lRect, DepthSpacePoint leftHanP)
        {
            //这是不是在上面出现过？
            //败者食尘吗？
            //还不带重复利用的我擦
            double dths = 0.07;//深度阈值
            //int len = this.depthPixels.Length;
            //uint[] displayPixels = this.bodyIndexPixels;
            uint[] rdata = new uint[lRect.Width * lRect.Height];
            ushort[,] rd = new ushort[lRect.Width, lRect.Height];
            ushort[,] rdcopy = new ushort[lRect.Width, lRect.Height];
            for (int i = 0; i < lRect.Width; ++i)
            {
                for (int j = 0; j < lRect.Height; ++j)
                {
                    int index = (j + lRect.Y) * this.displayWidth + i + lRect.X;
                    int hand_index = (int)(leftHanP.Y) * this.displayWidth + (int)(leftHanP.X);
                    if (overPixels(index) || overPixels(hand_index))
                        continue;
                    var diff = Math.Abs(cameraPoints[index].Z - cameraPoints[hand_index].Z);
                    //if (diff > dths)
                    //    displayPixels[index] = 0xFFFFFFFF;
                    if (diff < dths && bodyIndexPixels[index] != 0x00000000)
                    {
                        rd[i, j] = 1;
                        //ldata[j * lRect.Width + i] = 0x0000FF00;
                    }

                }
            }
            //函数可以加在这里处理8连通域

            bool[,] arr = new bool[lRect.Width, lRect.Height];


            for (int p = 0; p < lRect.Width; p++) //从零列开始
            {
                for (int q = 0; q < lRect.Height; q++)
                {
                    if (rd[p, q] == 1)
                        arr[p, q] = true;
                    else
                        arr[p, q] = false;
                }
            }




            List<List<Pointt>> ans = get(arr);
            int tot = 0;

            int max = 0;

            bool[,] temple = new bool[lRect.Width, lRect.Height];//每一步的临时数组
            bool[,] temple1 = new bool[lRect.Width, lRect.Height];//最终的临时数组

            foreach (List<Pointt> c in ans)
            {
                for (int p = 0; p < lRect.Width; p++)
                {
                    for (int q = 0; q < lRect.Height; q++)
                    {
                        temple[p, q] = false;
                    }
                }
                tot = 0;

                foreach (Pointt p in c)
                {
                    temple[p.x, p.y] = true;
                    ++tot;
                }


                if (max < tot)
                {
                    max = tot;
                    for (int i = 0; i < arr.GetLength(0); i++)
                    {
                        for (int j = 0; j < arr.GetLength(1); j++)
                        {
                            temple1[i, j] = temple[i, j];
                        }

                    }
                    tot = 0;
                }
            }//foreach (List<Pointt> c in ans)


            ushort[,] rdnew = new ushort[lRect.Width, lRect.Height];
            for (int p = 0; p < lRect.Width; p++)
            {
                for (int q = 0; q < lRect.Height; q++)
                {

                    if (temple1[p, q] == true)
                        rdnew[p, q] = 1;
                    else
                        rdnew[p, q] = 0;

                }
            }



            int rwidthRefresh = lRect.Width;//暂时还未更新
            int rheightRefresh = lRect.Height;

            int countleft = 0;
            Boolean flagleft = false;
            for (int m = 0; m < lRect.Width; m++) //从零列开始
            {
                for (int n = 0; n < lRect.Height; n++) //从这里开始ld改成了ldnew
                {

                    if (rdnew[n, m] != 0)//ld
                    {
                        //按1算。。行数减一就是所求   
                        flagleft = true;
                        break;
                    }
                }
                if (flagleft == true)
                    break;
                else
                    countleft++;
            }


            int counttop = 0;
            Boolean flagtop = false;
            for (int i = 0; i < lRect.Height; i++) //从零行开始
            {
                for (int j = 0; j < lRect.Width; j++)
                {
                    if (rdnew[i, j] != 0) //ld
                    {
                        flagtop = true;
                        break;

                    }

                }
                if (flagtop == true)
                    break;
                else
                    counttop++;

            }


            int countright = 0;
            Boolean flagright = false;
            for (int p = lRect.Width - 1; p >= 0; p--)  //从最后一列开始
            {

                for (int q = 0; q < lRect.Height; q++)
                {
                    if (rdnew[q, p] != 0) //ld pq
                    {
                        flagright = true;
                        break;

                    }
                }
                if (flagright == true)
                    break;
                else
                    countright++;

            }


            int counttail = 0;
            Boolean flagtail = false;                        //从最底行开始
            for (int p = lRect.Height - 1; p >= 0; p--)
            {
                for (int q = 0; q < lRect.Width; q++)
                {
                    if (rdnew[p, q] != 0)//ld
                    {
                        flagtail = true;
                        break;

                    }
                }
                if (flagtail == true)
                    break;
                else
                    counttail++;
            }



            ////再次对左边界进行处理

            //int countleft2 = 0;
            //Boolean flagleft2 = false;
            //for (int m = 0; m < lRect.Width; m++) //从零列开始
            //{
            //    for (int n = 0; n < lRect.Height; n++) //从这里开始ld改成了ldnew
            //    {

            //        if (ldnew[n, m] != 0)//ld
            //        {
            //            //按1算。。行数减一就是所求   
            //            flagleft2 = true;
            //            break;
            //        }
            //    }
            //    if (flagleft2 == true)
            //        break;
            //    else
            //        countleft2++;
            //}

            ushort[,] rdnew1;
            //ushort[,] ldnew1;
            if (lRect.Width - countright - countleft < 0 || lRect.Height - counttop - counttail < 0)
            {
                rdnew1 = null;
            }
            else
            {
                //ushort[,] ldnew1;
                rdnew1 = new ushort[lRect.Width - countright - countleft, lRect.Height - counttop - counttail];
                // ushort[,] ldnew1 = new ushort[lRect.Width-countright-countleft, lRect.Height-counttop-counttail];//以前的ldnew改成了ldnew1

                if (lRect.Width - countright - countleft >= 0 && lRect.Height - counttop - counttail >= 0)
                {

                    for (int a = 0; a < lRect.Width - countright - countleft; a++) //lRect.Height - counttop
                    {
                        for (int b = 0; b < lRect.Height - counttop - counttail; b++) // lRect.Width - countright - countleft
                        {

                            rdnew1[a, b] = rdnew[b + counttop, a + countleft];//分别为以前的ldnew 和ld

                            //if (counttop == 0 && countleft == 0)
                            //    ldnew[a, b] = ld[a + counttop, b + countleft];
                            //else if (counttop == 0)
                            //    ldnew[a, b] = ld[a + counttop, b + countleft - 1];
                            //else if (countleft == 0)
                            //    ldnew[a, b] = ld[a + counttop - 1, b + countleft];
                            //else
                            //    ldnew[a, b] = ld[a + counttop - 1, b + countleft - 1];
                        }
                    }
                }
                //else
                //    ldnew1 = null;


            }
            return rdnew1; //这里不再返回ld而是一个更小范围的矩形
        }
        //新添加
        static List<List<Pointt>> get(bool[,] arr)
        {
            //该算法是找八联通域
            int N = arr.GetLength(0);
            int M = arr.GetLength(1);
            //N行M列？
            int[,] v = new int[N, M];
            List<List<Pointt>> ret = new List<List<Pointt>>();
            //所以返回的就是ret咯
            for (int i = 0; i < N; ++i) for (int j = 0; j < M; ++j)
                    if (v[i, j] == 0 && arr[i, j])
                    {
                        //不是你v也就一个初始化操作吧
                        //哦，后面变了
                        v[i, j] = 1;
                        Queue<Pointt> q = new Queue<Pointt>();
                        q.Enqueue(new Pointt(i, j));
                        //将点（i，j）入队
                        //C#这么喜欢重载的吗？还是都是这样？
                        List<Pointt> l = new List<Pointt>();
                        while (q.Count > 0)
                        {
                            Pointt curr = q.Dequeue();
                            //取q中队尾元素加到l里面
                            l.Add(curr);
                            for (int dx = -1; dx <= 1; ++dx) for (int dy = -1; dy <= 1; ++dy)
                                {
                                    if (dx == 0 && dy == 0) continue;
                                    //若此时点不发生位移
                                    int nx = curr.x + dx, ny = curr.y + dy;
                                    if (nx < 0 || ny < 0 || nx >= N || ny >= M) continue;
                                    //若此时不越界
                                    if (v[nx, ny] == 1 || !arr[nx, ny]) continue;
                                    //若当前点为在v里面0但是实际上是1
                                    v[nx, ny] = 1;
                                    //更改当前值
                                    
                                    q.Enqueue(new Pointt(nx, ny));
                                    //把这个点加入队列
                                }
                        }
                        //也就是说一个while的八邻域循环找了所有的八邻接点
                        ret.Add(l);
                    }
            return ret;
        }


        private Tuple<ushort[,], ushort[,]> RenderDisplayPixels(Int32Rect lRect, DepthSpacePoint leftHanP, Int32Rect rRect, DepthSpacePoint rightHanP)
        {

            //double k = 35;
            double dths = 0.07;//深度阈值
            //好吧我跳到这里来了，虽然不知道有没有用
            //我绝对要去买眼药水和垃圾袋
            uint[] displayPixels = this.bodyIndexPixels;
            uint[] ldata = new uint[lRect.Width * lRect.Height];
            //一个是一维数组 一个是二维数组？？
            ushort[,] ld = new ushort[lRect.Width, lRect.Height];
            //又浮现出了熟悉的感觉，但是我不打算去上面找了
            for (int i = 0; i < lRect.Width; ++i)
            {
                for (int j = 0; j < lRect.Height; ++j)
                {
                    int index = (j + lRect.Y) * this.displayWidth + i + lRect.X;
                    int hand_index = (int)(leftHanP.Y) * this.displayWidth + (int)(leftHanP.X);
                    if (overPixels(index) || overPixels(hand_index))
                        //防止矩形框标记到图像外
                        continue;
                    var diff = Math.Abs(cameraPoints[index].Z - cameraPoints[hand_index].Z);
                    //手心到相机中心水平距离
                    //if (diff > dths)
                    //    displayPixels[index] = 0xFFFFFFFF;
                    if (diff < dths && bodyIndexPixels[index] != 0x00000000)//黑色
                        //如果手心距离没有超过阈值且bodeindex检测到了
                    {
                        ld[i, j] = 1;
                        //其实我还没搞明白到底是在干啥，如果说有直观的感受就好了
                        ldata[j * lRect.Width + i] = 0x0000FF00;//绿色

                    }

                }
            }

            uint[] rdata = new uint[rRect.Width * rRect.Height];
            ushort[,] rd = new ushort[rRect.Width, rRect.Height];
            for (int i = 0; i < rRect.Width; ++i)
            {
                for (int j = 0; j < rRect.Height; ++j)
                {
                    int index = (j + rRect.Y) * this.displayWidth + i + rRect.X;
                    int hand_index = (int)(rightHanP.Y) * this.displayWidth + (int)(rightHanP.X);
                    if (overPixels(index) || overPixels(hand_index))
                        continue;
                    var diff = Math.Abs(cameraPoints[index].Z - cameraPoints[hand_index].Z);
                    //if (diff > dths)
                    //    displayPixels[index] = 0xFFFFFFFF;
                    if (diff < dths && bodyIndexPixels[index] != 0x00000000)
                    {
                        rd[i, j] = 1;
                        rdata[j * rRect.Width + i] = 0x0000FF00;
                    }

                }
            }


            this.displayBitmap.WritePixels(
                new Int32Rect(0, 0, this.displayBitmap.PixelWidth, this.displayBitmap.PixelHeight),
                displayPixels,
                this.displayBitmap.PixelWidth * (int)BytesPerPixel,
                0);
            //这个函数是干嘛的我也不是很懂，他说更新就更新吧
            //借楼 大概就是把所有的pixel重新封装到位图里 从而再次被展示
            //int hh = 35;
            //var cut = lRect;// new Int32Rect(leftHand.Item1 - 40, leftHand.Item2 - 40, 2 * hh, 2 * hh);
            //这些注释里面的数字是哪里来的啊
            var stride = displayBitmap.Format.BitsPerPixel * lRect.Width / 8;
            //uint[] data = new uint[cut.Height * stride];
            //displayBitmap.CopyPixels(cut, data, stride, 0);
            //dealImgData(data);
            //切割下来的什么东西吗
            this.leftPic.Source = BitmapSource.Create(lRect.Width, lRect.Height, 0, 0, PixelFormats.Bgr32, null, ldata, stride);
            //这个legtPic我不知道是啥但是上面好像有什么相似的东西
            //cut = rRect; //new Int32Rect(rightHand.Item1 - 40, rightHand.Item2 - 40, 2 * hh, 2 * hh);
            stride = displayBitmap.Format.BitsPerPixel * rRect.Width / 8;
            //displayBitmap.CopyPixels(cut, data, stride, 0);
            //dealImgData(data);
            this.rightPic.Source = BitmapSource.Create(rRect.Width, rRect.Height, 0, 0, PixelFormats.Bgr32, null, rdata, stride);
            //Tuple<ushort[], ushort[]> res = new Tuple<ushort[], ushort[]>(ld,rd);
            return new Tuple<ushort[,], ushort[,]>(ld, rd);
        }
        //暂且不管了，先就这样算了

        private void RenderDisplayPixels(Tuple<int, int, float> leftHand, Tuple<int, int, float> rightHand)
        {

            //这啥，重载？为啥和上面开沟一样的啊，不返回上面东西吗
            //这个函数似乎并没有被调用
            double k = 35;
            double dths = 1;//深度阈值
            int len = this.depthPixels.Length;
            uint[] displayPixels = this.bodyIndexPixels;

            int leftC = 0;
            int rightC = 0;
            if (leftHand.Item1 > 0 && leftHand.Item2 > 0 && leftHand.Item1 < displayWidth && leftHand.Item2 < this.displayHeight)
                leftC = this.depthPixels[leftHand.Item2 * this.displayWidth + leftHand.Item1];
            if (rightHand.Item1 > 0 && rightHand.Item2 > 0 && rightHand.Item1 < this.displayWidth && rightHand.Item2 < this.displayHeight)
                rightC = this.depthPixels[rightHand.Item2 * this.displayWidth + rightHand.Item1];
            var maxC = leftC > rightC ? leftC : rightC;
            //我从来都是掉尾啊
            for (int i = 0; i < this.displayHeight; ++i)
            {
                for (int j = 0; j < this.displayWidth; j++)
                {
                    int index = i * this.displayWidth + j;
                    if ((DateTime.Now - start1).Seconds < 0) //倒计时
                    {
                        displayPixels[index] = (uint)(depthPixels[index] << 16 | depthPixels[index] << 8 | depthPixels[index]);
                    }
                    else
                    {
                        if (bodyIndexPixels[index] == 0x00000000)
                        {
                            displayPixels[index] = (uint)(depthPixels[index] << 16 | depthPixels[index] << 8 | depthPixels[index]);
                            continue;
                        }



                        if (depthPixels[index] > (maxC + dths))
                        {
                            displayPixels[index] = (uint)(depthPixels[index] << 16 | depthPixels[index] << 8 | depthPixels[index]); continue;
                        }


                        var v1 = Math.Sqrt((j - leftHand.Item1) * (j - leftHand.Item1) + (i - leftHand.Item2) * (i - leftHand.Item2));
                        var v2 = Math.Sqrt((j - rightHand.Item1) * (j - rightHand.Item1) + (i - rightHand.Item2) * (i - rightHand.Item2));
                        if ((v1 > (k / leftHand.Item3)) && (v2 > (k / rightHand.Item3)))
                        {
                            displayPixels[index] = (uint)(depthPixels[index] << 16 | depthPixels[index] << 8 | depthPixels[index]); continue;
                        }
                    }
                }

            }

            this.displayBitmap.WritePixels(
                new Int32Rect(0, 0, this.displayBitmap.PixelWidth, this.displayBitmap.PixelHeight),
                displayPixels,
                this.displayBitmap.PixelWidth * (int)BytesPerPixel,
                0);

            int hh = 35;
            var cut = new Int32Rect(leftHand.Item1 - 40, leftHand.Item2 - 40, 2 * hh, 2 * hh);
            var stride = displayBitmap.Format.BitsPerPixel * cut.Width / 8;
            uint[] data = new uint[cut.Height * stride];
            displayBitmap.CopyPixels(cut, data, stride, 0);
            dealImgData(data);
            this.leftPic.Source = BitmapSource.Create(2 * hh, 2 * hh, 0, 0, PixelFormats.Bgr32, null, data, stride);

            cut = new Int32Rect(rightHand.Item1 - 40, rightHand.Item2 - 40, 2 * hh, 2 * hh);
            displayBitmap.CopyPixels(cut, data, stride, 0);
            dealImgData(data);
            this.rightPic.Source = BitmapSource.Create(2 * hh, 2 * hh, 0, 0, PixelFormats.Bgr32, null, data, stride);


        }
        //此处未看，做此标记，不知道会不会有调用的一天
        private void dealImgData(uint[] data)
        {
            for (int i = 0; i < data.Length; ++i)
            {
                if (data[i] != 0x0000FF00)
                    data[i] = 0x00000000;
            }
        }

        private bool overPixels(int index)
        {
            if (index > this.displayBitmap.PixelHeight * this.displayBitmap.PixelWidth || index < 0)
                return true;
            return false;
        }

        private void ButtonStart_Click(object sender, RoutedEventArgs e)
        {
            
            this.showMsg.Content = "请选择录入手语的模式！";
            if (this.check1.IsChecked == true && this.check2.IsChecked == true)
                this.showMsg.Content = "手语的录入模式不能选两种！";
            else if (this.check1.IsChecked == true)
            {
                recordFlag = true;
                //ListBoxItem item = (ListBoxItem)gestureList.ContainerFromElement((DependencyObject)gestureList.SelectedItems[0]);
                this.showMsg.Content = "你选择了再次添加手语模板：" + ((Gesture)gestureList.SelectedItems[0]).name;
            }
            else if (this.check2.IsChecked == true)
            {
                recordFlag = true;
                this.showMsg.Content = "你选择了添加新手语模板：" + gestureName.Text.ToString();
            }
           
        }

        private void ButtonEnd_Click(object sender, RoutedEventArgs e)
        {
            //this.showMsg.ClearValue();
            this.showMsg.Content = "已经结束录制，正在提取并保存特征数据。";
            recordFlag = false;

            string gesName = null;
            if (this.check1.IsChecked == true)
            {
                gesName = ((Gesture)gestureList.SelectedItems[0]).name;
                //一般只有一个被选中的
                this.showMsg.Content = "保存特征数据成功！" + ((Gesture)gestureList.SelectedItems[0]).name;
                initData();
            }
            else if (this.check2.IsChecked == true)
            {
                gesName = gestureName.Text.ToString();
                this.showMsg.Content = "保存特征数据成功！" + gesName;
            }
            gestureFeature.saveData(gesName);

            gestureFeature.clear();
            gestureFeature = null;
            
        }

        private void GestureList_SelectionChanged(object sender, SelectionChangedEventArgs e)
        {

        }

        






        //private void Sensor_IsAvailableChanged(object sender, IsAvailableChangedEventArgs e)
        //{
        //    // on failure, set the status text
        //    this.StatusText = this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText
        //                                                    : Properties.Resources.SensorNotAvailableStatusText;
        //}


    }

    class Gesture
    {
        public int id { get; set; }
        public string name { get; set; }
    }

}
