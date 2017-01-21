# README for the ueye package

Ueye ROS package: Driver package for the IDS camera UI-1221LE-M-GL. <br />

This package publishes synchronized left and right images pairs from a stereo-vision camera system and publish as a single combined image.<br />

1. Exposure is handcrafted and the current setting is 0.2 for outdoor environment and 10 for indoor environment. <br />

2. Gain value of the left camera is set to "AUTO", which means the camera driver will automatically tune the gain value to adapt to the environment. <br />
<br />
To ensure the left and right images have the same brightness. The gain value of the right camera is set to follow the gain value of the left camera. <br />

3. The details of the settings can be found in the Algorithm Design Documentation (ADD). <br />
  
