=====================
Software Requirements
=====================

.. req:: Capturing Stereo Camera Frames
   :id: REQ_1719139176
   :status: open

   The component must capture and synchronize frames from two Raspberry Pi Module 3 cameras at a minimum resolution of 768x432 pixels and 
   a minimum frame rate of 30 FPS, with synchronization precision within 0.03 seconds.

.. req:: Stereo Camera Calibration
   :id: REQ_1719135324
   :status: open

   The component must be able to calibrate the stereo camera by computing camera matrices, distortion coefficients, relative rotation and 
   translation between cameras, and rectification and undistortion maps. The calibration accuracy must be within 0.5 pixels for reprojection error.


.. req:: Disparity Map
   :id: REQ_1719139720
   :status: open

   The component must compute a disparity map for two synchronized frames. The disparity map must have an average absolute error (AAE) of less than 
   1 pixel and an end point error (EPE) of less than 2 pixels, with no unidentified objects or invalid disparities. Disparity map accuracy should be 
   validated using one of the public stereo datasets, e.g. Middlebury dataset.

.. req:: 3D Environment Map
   :id: REQ_1719139935
   :status: open

   The component must compute a 3D map from a disparity map and camera parameters. The 3D map must have a maximum reprojection error of less than 
   3 cm in point-to-point distance when compared to ground truth for objects within a range of 0.1 to 10 meters.

.. req:: Image Preprocessing
   :id: REQ_1719140161
   :status: open

   The system must perform necessary preprocessing steps such as rectification and undistortion of the stereo images to ensure that epipolar lines are 
   aligned on the stereo images.