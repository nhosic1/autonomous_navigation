===================
System Requirements
===================

.. req:: Automobile Drive Control
   :id: REQ_1719061531
   :status: open

   The system must be able to control the acceleration, velocity and steering of the vehicle with automobile drive.

.. req:: Vehicle Velocity
   :id: REQ_1719051919
   :status: in progress

   The system must support a vehicle velocity of 0.3 m/s or greater.

.. req:: Real-Time Performance
   :id: REQ_1719051473
   :status: in progress

   The system must perform autonomous navigation in real-time. The overall processing duration must not exceed 0.3 seconds on the chosen hardware.

.. req:: Goal Location
   :id: REQ_1719055585
   :status: open

   The system must allow the user to provide the navigation goal location using 2D Cartesian coordinates through a user interface or API.

.. req:: Navigation Precision
   :id: REQ_1719056579
   :status: open

   The final 2D location of the vehicle's center must not differ from the goal location by more than 0.5 meters of point-to-point distance.

.. req:: Object Detection
   :id: REQ_1719059659
   :status: in progress

   The system must be able to detect surrounding objects in an unknown environment using onboard sensors.

.. req:: Collision Avoidance
   :id: REQ_1719056827
   :status: in progress

   The system must ensure that the vehicle avoids any collision with detected obstacles while moving towards the goal location.

.. req:: Environment Mapping
   :id: REQ_1719060958
   :status: open

   The system must be able to create a map of an unknown environment and actively update it in real-time while moving towards the goal location.

.. req:: Vehicle Trajectory
   :id: REQ_1719056980
   :status: open

   The system must ensure that the vehicle moves along an optimal trajectory that minimizes the travelled distance while avoiding obstacles in a 
   known environment.

.. req:: User Feedback and Monitoring
   :id: REQ_1719070003
   :status: open

   The system must provide real-time user feedback on the vehicle's status, including velocity, position, and potential system issues.
   This feedback should be accessible through a user interface or API.
