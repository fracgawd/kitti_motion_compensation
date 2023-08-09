# KITTI Motion Compensation Library 
Correction of measurement motion distortion is critical for accurate algorithm development and benchmarking. The Kitti Motion Compensation package provides an interface to motion compensate the Kitti dataset Velodyne lidar pointcloud using vehicle odometry and timing data. 

A nice way to visualize this effect is to project the pointcloud onto the image before and after motion compensation. As you can see pointclouds line up much better after motion compensation.

"Wow" | Look at that bike! |  The compensated person is much happier
:-------------------------:|:-------------------------:|:-------------------------:
![](assets/frame_1_left_scene.gif) |  ![](assets/frame_1_right_scene.gif) | ![](assets/frame_2_right_scene.gif)

The authors of the Kitti dataset explictly write in their development kit readme:

    IMPORTANT NOTE: 
    Note that the velodyne scanner takes depth measurements
    continuously while rotating around its vertical axis (in contrast to the cameras,
    which are triggered at a certain point in time). This means that when computing
    point clouds you have to 'untwist' the points linearly with respect to the velo-
    dyne scanner location at the beginning and the end of the 360° sweep.

I can't describe the problem much better than the authors of KITTI. As one the most influential robotics datasets on the planet, you would imagine there would be a pleathora of information about how to "untwist" (motion compensate) the dataset. Well it turns out there is not, and this package is an answer to that unfilled need. 

Note that the KITTI authors were very smart and trigger the cameras when the lidar is "pointing forward" towards the front of the car. This means that points directly in front of the car will line up perfectly, and only at the edges of the images do you really explicitly notice the motion distortion at the speeds they drive at. This was genius in one way that it made the dataset easy to use but unfortunate in another in that it let untold number of researchers ignore or at least not notice the problem.

## Notes
Future work:
* Projection error visualization
* Rotational velocity calculation
* CMake clean up
* Install instructions/demos

## Packages to Install 

    sudo apt install libgtest-dev libeigen3-dev libopencv-dev
    
## Kitti Benchmarks
Monocular depth estimation is a classic task, particularly self-supervised monocular depth estimation. I took as an example the [Monodepth2](https://github.com/nianticlabs/monodepth2) repo, one of the greatest contributions made to the open source depth estimation community. 

Inside the repo they provide the code to download the [KITTI Raw](https://www.cvlibs.net/datasets/kitti/raw_data.php) dataset. They also provide the code to extract the "ground truth" depth maps, and the code to calculate the depth error metrics between the predicted depth and the "ground truth" lidar depth. What they do not do however is motion compensate the pointcloud. 

If we do a very simple experiment where we calculate the error metrics using the original un-motion compensated lidar pointclouds and the motion compensated pointclouds we see the difference. I admit, it isn't earth shattering, but the "sq rel" error decreases from 0.730 to 0.725, the "rmse" from 4.392 to 4.386 and the "a1" bumps from 0.907 to 0.908 (increased a1/a2/a3 is better). Getting a almost 7% decrease in "sq rel" error for free is pretty nice if you ask me. 

The implications are even bigger for supervised monocular depth estimation methods, which for some reason were never able to resolve small objects. Small objects, particularly in the near road area to the left and right of the vehicle are where motion compensation has the most noticable effect, and could explain why some methods that didn't motion compensate saw poorer performance than they could have gotten otherwise. 

| Mode | abs rel | sq rel | rmse | rmse log | a1 | a2 | a3 |
| :---:   | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| Original | 0.092 | 0.730 | 4.392 | 0.171 | 0.907 | 0.964 | 0.982 |
| Motion Compensated | 0.092 | 0.725 | 4.386 | 0.170 | 0.908 | 0.964 | 0.982 |

Tested using the monodepth2 weights "mono+stereo_640x192" on 2011_09_26 runs 0001, 0005, 0091, 0104 and 0117. The scores are very good because the data was used for training also. The point here is to show that using motion compensation gets you better scores, not test the monodepth training itself so the splits are not as important.
