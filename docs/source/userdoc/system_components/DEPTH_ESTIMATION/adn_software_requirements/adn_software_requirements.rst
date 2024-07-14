=====================
Software Requirements
=====================

.. req:: Capturing Stereo Camera Frames
   :id: REQ_1719139176
   :status: open

   The component must capture and synchronize frames from two Raspberry Pi Module 3 cameras at a minimum resolution of 768x432 pixels and 
   a minimum frame rate of 30 FPS, with synchronization precision within 0.03 seconds and in RGB format.

.. req:: Stereo Camera Calibration
   :id: REQ_1719135324
   :status: open

   The component must be able to calibrate the stereo camera by computing camera matrices, distortion coefficients, relative rotation and 
   translation between cameras, and rectification and undistortion maps. The mean reprojection error for both cameras must not exceed 0.5 pixels.

.. req:: Image Preprocessing
   :id: REQ_1719140161
   :status: open

   The system must perform necessary preprocessing steps such as rectification and undistortion of the stereo images to ensure that epipolar lines are 
   horizontally aligned and that the optical centers of each camera have the same pixel coordinates in the rectified images.

.. req:: Disparity Map
   :id: REQ_1719139720
   :status: open

   The component must compute a disparity map for two synchronized frames. 

   The disparity map must have a mean absolute error (MAE) of less than 2 pixels and a root mean square error (RMSE) of less than 3 pixels, for valid 
   disparities. Disparity map accuracy should be validated using one of the public stereo datasets, e.g. Middlebury dataset.

   The disparity map must contain enough valid disparities to recognize objects within a range of 0.5 to 4 meters. The exceptions are cases that are
   genarally difficult to detect with cameras, e.g. textureless regions, low lighting conditions and repetative textures.

.. req:: 3D Environment Map
   :id: REQ_1719139935
   :status: open
   
   The component must compute a 3D map from a disparity map and camera parameters.