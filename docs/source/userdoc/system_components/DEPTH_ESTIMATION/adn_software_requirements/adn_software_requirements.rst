=====================
Software Requirements
=====================

.. req:: Capturing Stereo Camera Frames
   :id: REQ_1719139176
   :status: done

   The component must capture and synchronize frames from two Raspberry Pi Module 3 cameras at a minimum resolution of 768x432 pixels and 
   a minimum frame rate of 30 FPS, with synchronization precision within 0.03 seconds and in RGB format.

.. req:: Stereo Camera Calibration
   :id: REQ_1719135324
   :status: done

   The component must be able to calibrate the stereo camera by computing camera matrices, distortion coefficients, relative rotation and 
   translation between cameras, and rectification and undistortion maps. The mean reprojection error for both cameras must not exceed 0.5 pixels.

.. req:: Image Preprocessing
   :id: REQ_1719140161
   :status: done

   The system must perform necessary preprocessing steps such as rectification and undistortion of the stereo images to ensure that epipolar lines are 
   horizontally aligned and that the optical centers of each camera have the same pixel coordinates in the rectified images.

.. req:: Disparity Map
   :id: REQ_1719139720
   :status: done

   The component must compute a disparity map for two synchronized frames. 

   The disparity map must have a mean absolute error (MAE) of less than 2 pixels and a root mean square error (RMSE) of less than 3 pixels, for valid 
   disparities. Disparity map accuracy should be validated using one of the public stereo datasets, e.g. Middlebury dataset.

   The disparity map must contain enough valid disparities to recognize objects within a range of 0.5 to 4 meters. The exceptions are cases that are
   genarally difficult to detect with cameras, e.g. textureless regions, low lighting conditions and repetative textures.

.. req:: 3D Environment Map
   :id: REQ_1719139935
   :status: done
   
   The component must compute a 3D map from a disparity map and camera parameters.
   
.. req:: Ultrasonic Sensor Integration
   :id: REQ_1720960001
   :status: open

   The component must integrate a minimum of six ultrasonic sensors to provide additional depth information. Three sensors should be placed at the front 
   and three at the back of the vehicle. More sensors are recommended to improve coverage and accuracy.

   The ultrasonic sensors must have a range of 0.1 meters to 3 meters and a field of view of at least 15 degrees. The data from these sensors must be 
   captured at a minimum rate of 20 Hz and synchronized with the stereo camera frames within 0.03 seconds.

.. req:: Sensor Fusion
   :id: REQ_1720960203
   :status: open

   The system must perform fusion of the depth information from stereo camera disparity maps and ultrasonic sensor data to produce a more 
   accurate and robust depth estimation by leveraging the strengths of each sensor.

   Ultrasonic sensors provide reliable short-range measurements, typically unaffected by ambient light conditions or the texture of objects, but have limited
   coverage due to narrow field of view.

   Stereo cameras provide precise long-range measurements and detailed object structure data, but perform poorly with textureless regions, low lighting 
   conditions and repetative textures.