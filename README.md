# RGBD_SLAM
# Summary
1. Implemented feature detection with ORB and matching with FLANN and tested Brute-Matcher in OpenCV.
2. Estimated camera poses by PnP with RANSAC and optimized estimation by minimizing re-projection error in g2o.
3. Built a local map for adding features and updating pose estimation then tested results on TUM dataset.
4. Created two dense maps by Octomap and by PCL with depth filter and statistical outliers removal filter.

# Result
[Click here to watch the result video](https://youtu.be/6-rgJaZmCC0)

[![tum_result](/result/tum_result.png)](https://youtu.be/6-rgJaZmCC0)

# PCL map
![tum_result](/result/pcl_result.png)]

# Octomap
![tum_result](/result/octomap_result.png)
