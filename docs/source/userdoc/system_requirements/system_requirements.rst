===================
System Requirements
===================

.. req:: Vehicle Control
   :id: REQ_1719061531
   :status: open

   The system must control the acceleration, velocity and steering of a 4-wheel vehicle with front wheel steering mechanism.

.. req:: Vehicle Velocity
   :id: REQ_1719051919
   :status: in progress

   The system must be able to achieve a maximum vehicle velocity of at least 0.3 m/s. Higher velocities are desirable, provided other 
   requirements are met.

.. req:: Reaction Time
   :id: REQ_1719051473
   :status: in progress

   The processing time before the vehicle reacts to new sensor data must not exceed 0.3 seconds on the chosen hardware.

.. req:: Vehicle Responsiveness
   :id: REQ_1720349019
   :status: in progress

   The maximum distance the vehicle can travel without responding to an obstacle or a command, considering the reaction time and the velocity, 
   must not exceed 0.1 meters.

.. req:: Goal Location
   :id: REQ_1719055585
   :status: open

   The system must allow the user to provide the navigation goal location using 2D Cartesian coordinates through a user interface or API.

.. req:: Navigation
   :id: REQ_1720353870
   :status: open

   The system must navigate the vehicle to the goal location in both known and unknown environments.

.. req:: Navigation Precision
   :id: REQ_1719056579
   :status: open

   The final 2D location of the vehicle's center must not differ from the goal location by more than 0.5 meters of point-to-point distance.

.. req:: Vehicle Trajectory in Known Environments
   :id: REQ_1719056980
   :status: open

   The system must ensure that the vehicle moves along an optimal trajectory that minimizes the traveled distance while avoiding obstacles in a 
   known environment.

.. req:: Object Detection
   :id: REQ_1719059659
   :status: in progress

   The system must detect objects in the vehicle's way within a range of 0.1 to 4 meters in an unknown environment using onboard sensors.
   Greater range is desirable, provided other requirements are met.

.. req:: Collision Avoidance
   :id: REQ_1719056827
   :status: in progress

   The system must ensure that the vehicle avoids any collision with detected obstacles while moving towards the goal location. The exception is dynamic
   obstacles that change positions within the processing duration.

.. req:: Environment Mapping
   :id: REQ_1719060958
   :status: in progress

   The system must create and actively update a 3D map of an unknown environment from detected objects in real-time while moving towards the goal location.

.. req:: User Feedback and Monitoring
   :id: REQ_1719070003
   :status: open

   The system must provide real-time user feedback on the vehicle's status, including velocity, position, and potential system issues.
   This feedback should be accessible through a user interface or API.
