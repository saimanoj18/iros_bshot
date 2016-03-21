# iros_bshot

B-SHOT : A Binary Feature Descriptor for Fast and Efficient Keypoint Matching on 3D Point Clouds

# Source Code

1. PCL needs to be installed on the system.

2. In the CMakeLists.txt file, make sure that the code finds PCL installed in your system
# Adding PCL library
#set(PCL_DIR "/home/sai/workspace/pcl-pcl-1.7.2/build")
find_package(PCL 1.7.2 REQUIRED)

3. We provide a default implementation for SHOT matching and B-SHOT matching and sample point clouds are provided with the source code.


