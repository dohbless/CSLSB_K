using System;
using Emgu.CV;
using Emgu.CV.Structure;
using System.Windows;
using Microsoft.Kinect;

namespace Microsoft.Samples.Kinect.BodyIndexBasics
{
    class MyKalman
    {
        private Kalman kalman;
        private Matrix<float> state;
        private Matrix<float> transitionMatrix;
        private Matrix<float> measurementMatrix;
        private Matrix<float> processNoise;
        private Matrix<float> measurementNoise;
        private Matrix<float> errorCovariancePost;
        private int m, n;// 默认为 m = 4， n = 2；

        public MyKalman():this(4, 2)
        {
 
        }

        //手势区域卡尔曼滤波初始化
        public MyKalman(int m,int n)
        {
            this.m = m;
            this.n = n;
            kalman = new Kalman(m, n, 0);

            if (m == 6)
                InitHand();
            else
                InitGesture();

            kalman.CorrectedState = state;
            kalman.TransitionMatrix = this.transitionMatrix;
            kalman.MeasurementNoiseCovariance = this.measurementNoise;
            kalman.ProcessNoiseCovariance = this.processNoise;
            kalman.ErrorCovariancePost = this.errorCovariancePost;
            kalman.MeasurementMatrix = this.measurementMatrix;
        }

        //手心卡尔曼滤波的参数初始化
        public void InitHand()
        {
            state = new Matrix<float>(6, 1);
            // x-pos
            state[0, 0] = 0f;
            // y-pos
            state[1, 0] = 0f;
            // z-pos
            state[2, 0] = 0f;
            // x-velocity
            state[3, 0] = 0f;
            // y-velocity
            state[4, 0] = 0f;
            // z-velocity
            state[5, 0] = 0f;
            transitionMatrix = new Matrix<float>(new float[,]
                    {
                         // x-pos, y-pos, x-velocity, y-velocity
                        {1, 0, 0, 1, 0, 0}, 
                        {0, 1, 0, 0, 1, 0},
                        {0, 0, 1, 0, 0, 1},
                        {0, 0, 0, 1, 0, 0},
                        {0, 0, 0, 0, 1, 0},
                        {0, 0, 0, 0, 0, 1},
                    });
            measurementMatrix = new Matrix<float>(new float[,]
                    {
                        { 1, 0, 0, 0, 0, 0 },
                        { 0, 1, 0, 0, 0, 0 },
                        { 0, 0, 1, 0, 0, 0 }
                    });
            measurementMatrix.SetIdentity();
            //Linked to the size of the transition matrix
            processNoise = new Matrix<float>(6, 6);
            //The smaller the value the more resistance to noise 
            processNoise.SetIdentity(new MCvScalar(1.0e-4));
            //Fixed accordiong to input data 
            measurementNoise = new Matrix<float>(3, 3);
            measurementNoise.SetIdentity(new MCvScalar(1.0e-4));
            //Linked to the size of the transition matrix
            errorCovariancePost = new Matrix<float>(6, 6); 
            errorCovariancePost.SetIdentity();
        }

        //手型卡尔曼滤波的参数初始化
        public void InitGesture()
        {
            state = new Matrix<float>(4, 1);
            state[0, 0] = 0f; // x-pos
            state[1, 0] = 0f; // y-pos
            state[2, 0] = 0f; // x-velocity
            state[3, 0] = 0f; // y-velocity
            transitionMatrix = new Matrix<float>(new float[,]
                    {
                        {1, 0, 1, 0},  // x-pos, y-pos, x-velocity, y-velocity
                        {0, 1, 0, 1},
                        {0, 0, 1, 0},
                        {0, 0, 0, 1}
                    });
            measurementMatrix = new Matrix<float>(new float[,]
                    {
                        { 1, 0, 0, 0 },
                        { 0, 1, 0, 0 }
                    });
            measurementMatrix.SetIdentity();
            //Linked to the size of the transition matrix
            processNoise = new Matrix<float>(4, 4);
            //The smaller the value the more resistance to noise
            processNoise.SetIdentity(new MCvScalar(1.0e-4));
            //Fixed accordiong to input data 
            measurementNoise = new Matrix<float>(2, 2); 
            measurementNoise.SetIdentity(new MCvScalar(1.0e-4));
            //Linked to the size of the transition matrix
            errorCovariancePost = new Matrix<float>(4, 4); 
            errorCovariancePost.SetIdentity();
        }

        public Matrix<float> GetMeasurement()
        {
            Matrix<float> measurementNoise = new Matrix<float>(n, 1);
            measurementNoise.SetRandNormal(new MCvScalar(), new MCvScalar(Math.Sqrt(measurementNoise[0, 0])));
            return measurementMatrix * state + measurementNoise;
        }

        public void GoToNextState()
        {
            Matrix<float> processNoise = new Matrix<float>(m, 1);
            processNoise.SetRandNormal(new MCvScalar(), new MCvScalar(processNoise[0, 0]));
            state = transitionMatrix * state + processNoise;
        }

        public Point Update(Point pt)
        {
            state[0, 0] = (float)pt.X;
            state[1, 0] = (float)pt.Y;
            Matrix<float> prediction = kalman.Predict();
            Point predictPoint = new Point(prediction[0, 0], prediction[1, 0]);
            Point measurePoint = new Point(GetMeasurement()[0, 0],GetMeasurement()[1, 0]);
            Matrix<float> estimated = kalman.Correct(GetMeasurement());
            Point estimatedPoint = new Point(estimated[0, 0], estimated[1, 0]);
            GoToNextState();
            //Point[] results = new Point[2];
            //results[0] = predictPoint; //预测位置
            //results[1] = estimatedPoint; //估计位置
            //px = predictPoint.X;
            //py = predictPoint.Y;
            //cx = estimatedPoint.X;
            //cy = estimatedPoint.Y;
            return estimatedPoint;
        }

        public DepthSpacePoint Update(DepthSpacePoint pt)
        {
            state[0, 0] = pt.X;
            state[1, 0] = pt.Y;
            Matrix<float> prediction = kalman.Predict();
            Point predictPoint = new Point(prediction[0, 0], prediction[1, 0]);
            Point measurePoint = new Point(GetMeasurement()[0, 0], GetMeasurement()[1, 0]);
            Matrix<float> estimated = kalman.Correct(GetMeasurement());
            DepthSpacePoint estimatedPoint = new DepthSpacePoint();

            estimatedPoint.X = estimated[0, 0];
            estimatedPoint.Y = estimated[1, 0];

            GoToNextState();
            //Point[] results = new Point[2];
            //results[0] = predictPoint; //预测位置
            //results[1] = estimatedPoint; //估计位置
            //px = predictPoint.X;
            //py = predictPoint.Y;
            //cx = estimatedPoint.X;
            //cy = estimatedPoint.Y;
            
            return estimatedPoint;
        }

        //通过深度信息(x,y,d)去预测
        public CameraSpacePoint Update(CameraSpacePoint csp)
        {
            state[0, 0] = csp.X;
            state[1, 0] = csp.Y;
            state[2, 0] = csp.Z;
            Matrix<float> prediction = kalman.Predict();

            CameraSpacePoint predictPoint = new CameraSpacePoint();
            CameraSpacePoint measurePoint = new CameraSpacePoint();
            CameraSpacePoint estimatedPoint = new CameraSpacePoint();

            //Point predictPoint = new Point(prediction[0, 0], prediction[1, 0]);
            predictPoint.X = prediction[0, 0];
            predictPoint.Y = prediction[1, 0];
            predictPoint.Z = prediction[2, 0];

            var _measure = GetMeasurement();
            //Point measurePoint = new Point(GetMeasurement()[0, 0], GetMeasurement()[1, 0]);
            measurePoint.X = _measure[0, 0];
            measurePoint.Y = _measure[1, 0];
            measurePoint.Z = _measure[2, 0];

            Matrix<float> estimated = kalman.Correct(_measure);
            //Point estimatedPoint = new Point(estimated[0, 0], estimated[1, 0]);
            estimatedPoint.X = estimated[0, 0];
            estimatedPoint.Y = estimated[1, 0];
            estimatedPoint.Z = estimated[2, 0];
            GoToNextState();
            //Point[] results = new Point[2];
            //results[0] = predictPoint; //预测位置
            //results[1] = estimatedPoint; //估计位置
            //px = predictPoint.X;
            //py = predictPoint.Y;
            //cx = estimatedPoint.X;
            //cy = estimatedPoint.Y;
            return estimatedPoint;
        }
    }
}
