# Continuous-Sign-Language-Sentence-Based-on-Kinect
## 关键动作的识别
### 深度学习入门
- https://www.bilibili.com/video/av16001891?from=search&seid=16798140708617054978
- 参考书目 <<TensorFlow 技术解析和实战>>
### 制作自己的数据集
#### - TFRecords读取数据方式 
https://www.cnblogs.com/xianhan/p/9146485.html    
https://blog.csdn.net/rookie_wei/article/details/81275663
#### 简易数据集 
https://blog.csdn.net/jesmine_gu/article/details/81155787
## 神经网络训练
https://blog.csdn.net/dy_guox/article/details/79111949
### 
- python xmltocsv.py
-  Create train data:
  python generatetfrecords.py --csv_input=data/dataset_train.csv  --output_path=train.record
  python xmltorecod.py --csv_input=data/dataset_train.csv  --output_path=train.record
   python xmltorecod.py --csv_input=data/test_2.csv  --output_path=test.record
   python xmltorecod.py --csv_input=data/train_4.csv  --output_path=train.record
  -(！！！一定要将main里面的路径改为自己的图片们)对应文件夹的images下
 - change config
 ssd_mobilenet_v1_coco.config 
 - start train
 # From the tensorflow/models/research/ directory
python ./legacy/train.py --logtostderr --train_dir=training/ --pipeline_config_path=training/ssd_mobilenet_v1_coco.config
