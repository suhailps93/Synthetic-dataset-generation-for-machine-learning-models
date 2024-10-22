# Synthetic dataset generation for machine learning models
One of the most important problems that is faced by a machine learning, is the time and effort required for collection and preparation of training data. This package generates synthetic datasets for training object recognition models. A huge dataset of fake simulated images of any object scanned by a depth camera is generated, so that machine learning models could be trained on a variety of data to make them more robust. The project package performs following tasks 
- Scan the object to collect point clouds of the object using RGBD camera from different angles.
- Stitch the point clouds together to create a 3D point cloud of the object scanned.
- Create a 3d model from the 3d point cloud using surface reconstruction
- Generate fake images of the object by simulating different lighting conditions, pose, scale etc of the object using Gazebo.
- Add random backgrouds to the collected images.
- Prepare the data as a compatible input to a machine learning model.
- Train a Tensorflow object detection model on the images generated.
- Detect the object and its location in a camera feed after training.

##### Step 1: Scan the object 
(ASUS Xtion Pro Live RGBD sensor is used to scan the object. For demontration purposes the object was placed on top of a turntable that is rotated with hand to make sure all parts of the object is being scanned. The [record.cpp](https://github.com/SuhailPallathSulaiman/Synthetic-dataset-generation-for-machine-learning-models/blob/master/src/record.cpp) program helps in scanning. Once the code is run, a pointcloud visualiser will pop up showing the output of the depth camera. The program provides three option

  - Crop the output :- The user will have the option to crop the output by inputting X, Y, Z limits, so that only the object of interest is scanned.
  - Start Saving:- Once user is happy with adjusting the cropbox, he can start saving the poinclouds and rotate the object in front of the sensor so that all sides of the object is scanned. The user can pause the saving anytime and resume after that. 
  - Pause Saving:- This option can be used to pause saving the pointclouds. 

The output from the scan program will be a bunch of pointclouds as shown below.
![alt text](https://github.com/SuhailPallathSulaiman/Synthetic-dataset-generation-for-machine-learning-models/blob/master/images/Demos/scan_output.gif)

##### Step 2: Stitch the pointclouds 

The [3D_stiching.py](https://github.com/SuhailPallathSulaiman/Synthetic-dataset-generation-for-machine-learning-models/blob/master/src/3D_stiching.py)  file reads these pointclouds as input and gives out a merged 3D pointcloud of the object scanned as shown below. 

The user have optional control overfollowing things while stiching
- Choose between RANSAC and coarse ICP algorithm for global registration. 
- Choose between fine ICP and color ICP algorithm for local registration.
- Choose how individual pointclouds are merged ie., Pairwise or merging to a base pointcloud
- Input the fitness value below which the merge should be rejected

The final output will be saved in both .pcd and .ply file formats. 
![alt text](https://github.com/SuhailPallathSulaiman/Synthetic-dataset-generation-for-machine-learning-models/blob/master/images/Demos/merge_output.gif)

Process of 3D stitching on a human is shown below
![alt text](https://github.com/SuhailPallathSulaiman/Synthetic-dataset-generation-for-machine-learning-models/blob/master/images/Demos/human_stiching.gif)

##### Step 3: Surface reconstruction
The code for performing surface reconstruction is yet to be implemented. For time being Meshlab is used. The .ply file is imported in to meshlab and ball pivoting surface reconstruction algorithm is used to reconstruct the surface. A texture map is also generated in the form of a .png file so that the mesh will have color when opened in simulation software like Gazebo. The reconstructed mesh is then exported as a COLLADA file. An example is shown below.
![alt text](https://github.com/SuhailPallathSulaiman/Synthetic-dataset-generation-for-machine-learning-models/blob/master/images/Demos/surface_reconstruction_output.gif)
##### Step 4: Generate Simulated Images.
A Gazebo model is created using the COLLADA file. The model can be created for any .dae file by just changing a path in a skeleton model. Automation of generating Gazebo model from .dae file is in progress.

The image_grabber ROS package helps in generating the fake simulated images from the model. The image_grabber.launch file performs the following tasks.
- Opens a empty world 
- Spawn the object model of which the fake images have to be generated.
- Spawn a camera model. Once the camera model is spawned it starts saving the images in its vision.
- Starts the set_model_state.py node which changes the pose of both the camera and the object in such a way that the camera covers all the angles of the object. The gazebo world in action is shown below.

![alt text](https://github.com/SuhailPallathSulaiman/Synthetic-dataset-generation-for-machine-learning-models/blob/master/images/Demos/gazebo_simulation.gif)

Example output images fromthe Gazebo simulationis shown below
![alt text](https://github.com/SuhailPallathSulaiman/Synthetic-dataset-generation-for-machine-learning-models/blob/master/images/Demos/gazebo_output.png)

##### Step 5: Add random backgrounds to Images and prepare the dataset.
Gazebo give images with empty backgrounds, which is not a good dataset for training object recognition models. The bounding_n_background.py program performs the following tasks.
- Reads the output images from gazebo, crops out only the object of interest from the images using OpenCV, and paste the cropped objects to random images provided as input, so that the machine learning model will be more robust in recognising the object within a lot of other objects.Some example images are shown below
![alt text](https://github.com/SuhailPallathSulaiman/Synthetic-dataset-generation-for-machine-learning-models/blob/master/images/Demos/bg_output.png)
- This program also generate a .csv file which have all the data required for training the machine learning model like the bounding box of the object in the image (which is found using OpenCV), dimensions of the image and the label for the object (The name with which the trained model should identify the object)

Now the generate_tfrecord.py program reads the .csv files and generates this data in tfrecord file format, which is the input type that is used for training tensorflow object recognition API.


##### Step 6: Training a machine learning  model.
Tensorflow object detection API was used for demonstration purpose. The train.py program starts the training process. 
The graph showing the total loss with number of steps during the training process is shown below.

![alt text](https://github.com/SuhailPallathSulaiman/Synthetic-dataset-generation-for-machine-learning-models/blob/master/images/Demos/tensorflow_total_loss.png)

##### Step 7: Live detection
The object_detection .py file opens a video recording for testing the Machine learning model after training. The training is not yet perfect, but still the model is detecting the object most of the time. This can be made more robust by training on a huge dataset generated using the package with more simulated conditions incorporated 
[![IMAGE ALT TEXT HERE](http://img.youtube.com/vi/w1X17ET0MNY/0.jpg)](http://www.youtube.com/watch?v=w1X17ET0MNY)
