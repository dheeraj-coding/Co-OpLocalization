# Co-OpLocalization

A co-operative QR landmark based localization for Swarm of FLSs. 

Our system will involve FLSs with a cube of QR codes attached to their bottom. 
Each FLS will have a unique ID and their ID will be encoded in their QR codes. 
For any 360 camera, we will process the stream of images and identify the list of QR codes that are valid and in the frame. 
We will extract their ID and cross reference it against our database to identify the FLS. 
Once we have identified the FLS we will query that FLS using MAVLink to obtain their status. 
If an FLS is in its designated position then we can use their coordinates as support points for estimating distance of our FLS, else we will simply discard it. 
Once we have at least 3 FLS and their distances we can compute the position of our FLS by triangulation.

For computing the distance of an FLS using camera images we will have to first identify the corner points of QR code using computer vision techniques such as Harris Corner detection etc. 
Once we have the four corner points of the QR code and the actual size of the QR code (known to us when we generate and print the QR code) we can solve for distance between the camera & FLS in view.

In reality when we compute distances using techniques such as Dead Reckoning or Visual Odometry it is bound to produce errors. 
Since errors get accumulated over time and can lead to skewed position results we have to constantly correct those errors from sensor computations. 
For this process we will be utilizing a Kalman Filter to improve our estimates.

The novelty of this project would be to utilize QR landmarks on neighboring drones to identify the relative camera pose with respect to the neighboring drones and fuse them using the Extended Kalman Filter (EKF) to improve the accuracy of localization and prevent drifts. 
Using computer vision techniques we can obtain the relative measurement between two FLSs.
