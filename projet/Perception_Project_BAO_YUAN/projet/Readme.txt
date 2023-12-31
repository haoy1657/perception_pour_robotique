The report contains :
A report explaining the methods used and the analysis of results
A Zip file containing the :
-Video2Frames, function that convert the video into 460 images

-initial_position_camera, function for estimating the initial camera position and orientation in 3D space using a set of known world coordinates and corresponding pixel coordinates of points, utilizing matrix decomposition and the camera's intrinsic parameters (more precisely, dlt approach to the pnp problem is used to estimate the initial camera position from the pixel coordinates and 3D coordinates of the six points)

-feature_extraction, this function extracts and matches feature points between two images using SURF, and returns their homogeneous coordinates (the coordinates of the matched pixels)

-epipolar_geometry, the code estimates camera pose and epipolar geometry from feature points by solving for rotation, translation, and minimizing reprojection error.

-homography, the code initially computes the homography matrix and the essential matrix from a set of corresponding feature points between two images. These matrices are then utilized to determine the camera motion and calculate the reprojection error. The camera motion that minimizes the reprojection error is selected. Additionally, the code employs triangulation with the homography and essential matrices to estimate the depths of the feature points, ultimately choosing the solution that maximizes the number of feature points with positive depths

-camera_track, this function computes the camera trajectory by combining rotation and translation matrices for each successive image, and it stores the positions in the "track" variable.

-filter_low_pass, this function is used to smooth the final trajectory

-plot_error, this function plots the reprojection error over time, with the red dashed line representing the mean error. It provides a visual representation of the error in the context of frame transitions from i-1 to i

-plot3_position_camera,this function generates a 3D plot of the camera's position track over time, with markers denoting the camera's positions. It visualizes the camera's movement in 3D space.

-mainbox_full_result_display, this programme is a presentation of the overall camera track (460 frames of video).To avoid long run times, we saved the results in advance, so that we could plot the trajectories directly, saving the data in a mat file starting with databox

-mainbox_20frames, this code estimates the camera trajectory for the first twenty frames of a video and provides a visual representation of the camera's movement

-OpenSurf, this code provides a MATLAB implementation of the SURF algorithm for detecting and describing interest points in an image, which can be used for various computer vision tasks like object recognition and image matching


ATTENTION:  
To run this code, this project requires the installation of "computer vision toolbox" and "signal processing toobox "


-Finally a video, explaining the results by running the codes