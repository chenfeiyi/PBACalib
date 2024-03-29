# PBACalib

  ### Description

- This is the original work on livox-camera extrinsic calibration. The corresponding paper is " Feiyi. Chen, Liang. Li, Shuyang. Zhang, Jin. Wu and Lujia. Wang, "[PBACalib: Targetless Extrinsic Calibration for High-Resolution LiDAR-Camera System Based on Plane-Constrained Bundle Adjustment," in IEEE Robotics and Automation Letters, 2022.](https://ieeexplore.ieee.org/abstract/document/9968080/)".  You can visit our [website](https://ram-lab.com/file/site/fusionportable/calibration/pbacalib/) to find more details and the supplementary 
- This work is implemented by Matlab.
<img src="matlab/figures/overview.png" width = "60%" alt="Overview" align=center />

### Project structure

**├─matlab**                &emsp;&emsp;  The code to perform calibration  
**│  ├─colmap**              &emsp; Related tools to process the data exported from colamp  
**│  ├─LM_solver   
│  │  ├─jocbian  
│  │  └─obj   
│  └─utils   
├─ros_ws**                &emsp;&emsp;   The related cpp code to collect the data for the calibration  
**└─shell**                &emsp;&emsp;&emsp;      Shell scripts to perform SFM, which will call the exec files in colmap   

### Data Preparation

- Calibration Scene
  - First find a calibration scene, which is a plane with arbitrary texture. The calibration accuracy performs better when 1) texture is rich 2) the plane is strictly flat 3) the background is clean.
  - The example scenes are shown as follows  
     ![image](matlab/figures/cali_scene.png)
  
- Collection tools
  - We supply tools to collect images and point could. For our sensors operate in ROS
  framework, we write c++ tools to subscribe ROS topic and save data.
  - The tool is in folder **ros_ws**, which is a ros workspace. Run following command to build and execuate the tool
    ```shell
      catkin_make
      rosrun livox_cam_tools liv_map_cam_recorder
    ```
    It will print help notes to tell you what parameters you need to specify, as follows
    ![image](matlab/figures/helpnotes.png)
- Undistorted images
  Run matlab file **undist_imgs.m** to undistort all images. Please change the intrinsic, distortion matrix and data file path.
- Default data structure 
  ```
  data/img/1.png (raw images)
  data/img/2.png (raw images) 
  ...
  ----------------------------
  data/img_un/1.png (undistorted images)
  data/img_un/2.png (undistorted images) 
  ...
  ----------------------------
  data/pcd/1.pcd (raw pcds)
  data/pcd/2.pcd (raw pcds)
  
  ```
### Estimate camera poses using structure from motion
 We use [colmap](https://github.com/colmap/colmap/releases) to conduct SfM and export model files as txt into folder "models". Then the default data structure is shown as follows
 ```
  data/img/1.png (raw images)
  data/img/2.png (raw images) 
  ...
--------------------------
  data/img_un/1.png (undistorted images)
  data/img_un/2.png (undistorted images) 
  ...
--------------------------
  data/pcd/1.pcd (raw pcds)
  data/pcd/2.pcd (raw pcds)
  ...
--------------------------
  models/cameras.txt
  models/images.txt
  models/points3D.txt
  models/project.ini
 ```
 Please read readme files in colmap to learn how to conduct SfM

### Calibration 
  Run "main_cali_real.m" file in matlab folder to calibrate the extrinsics between camera and dense LiDAR. Please modify the parameters in  "main_cali_real.m", which  contains

  ```
  K = [897.4566,0,635.4040;
    0,896.7992,375.3149;
      0,0,1];
  D = [-0.4398 0.2329 -0.0011 2.0984e-04 -0.0730];

  TInit = [0.0324   -0.9994    0.0130   -0.0152
      0.0215   -0.0123   -0.9997    0.0695
      0.9992    0.0327    0.0211   -0.0132
      0         0         0    1.0000];
  data_path = "/home/cfy/Documents/livoxBACali/data/real/scene2/";
  pcd_folder = data_path+"pcd";
  img_folder = data_path+"img_un";
  ```
  The extrinsics and projection result will show automatically when finished.

### Data

- simulation environment: based on gazebo, we published on this [repo](https://github.com/chenfeiyi/LivoxCamSimu)

- the collected real and simulation data is placed on the [google drive](https://drive.google.com/drive/folders/15Ev3qeRZoKknBvDcjrhP_d7PFisJME6W?usp=sharing)

If you use this project for your research, please cite:

```
@ARTICLE{chen2022pbacalib,
  author={Chen, Feiyi and Li, Liang and Zhang, Shuyang and Wu, Jin and Wang, Lujia},
  journal={IEEE Robotics and Automation Letters}, 
  title={PBACalib: Targetless Extrinsic Calibration for High-Resolution LiDAR-Camera System Based on Plane-Constrained Bundle Adjustment}, 
  year={2023},
  volume={8},
  number={1},
  pages={304-311},
  doi={10.1109/LRA.2022.3226026}}
```



### TODO
- Please feel free to report issue

