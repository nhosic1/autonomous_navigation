=====================
Software Requirements
=====================

.. req:: Camera Pose Estimation
   :id: REQ_1720394989
   :status: open

   The component must estimate the 6-DoF (Degrees of Freedom) pose of the camera by analyzing sequential stereo camera frames.

   The maximum translational error between two sequential frames must not exceed 3% of the distance traveled.

   The maximum rotational error between two sequential frames must not exceed 0.3 degrees.

   The specified error limits apply under typical operating conditions, which include:

   - Environments with adequate lighting.
   - Surfaces with sufficient texture for stereo vision.
   - Normal operational speed and motion without abrupt changes.

.. req:: 9-DoF Sensor Pose Estimation
   :id: REQ_1720958085
   :status: open

   The component must estimate the 6-DoF (Degrees of Freedom) pose of the vehicle by using data from a 9-DoF sensor.

.. req:: Sensor Fusion
   :id: REQ_1720958266
   :status: open

   The component must incorporate odometry data fusion from the stereo cameras and the 9-DoF sensor to produce a more 
   accurate and robust estimation of the vehicle's trajectory by leveraging the strengths of each sensor. 

   9-DoF sensors are prone to drift over time but provide high-frequency motion data, while visual odometry relies on 
   environmental features to maintain long-term accuracy but can be affected by environmental conditions.

.. req:: Environment Mapping
   :id: REQ_1720959113
   :status: open

   The component must perform continuous 3D mapping of the environment using stereo camera frames (:need:`REQ_1719139935`), 
   simultaneously with the pose estimation. 
   
   This will allow the vehicle to correct the accumulated drift of the estimated trajectory by detecting when the vehicle 
   revisits a previously seen location (loop closure).

   The 3D map can be used for faster and more efficient navigation in the same environment.