# OtoMatic
Automated Autoscope

Make sure to use:
source ~/.profile && workon cv

To run kmeans script:
python color_kmeans.py --image images/jp.png --clusters 3

* TODO:
  * Create static identifier for camera: 3rd answer
    * https://unix.stackexchange.com/questions/66901/how-to-bind-usb-device-under-a-static-name
  * Write data to File with timestamps
  * Camera button: One click start video second click save video and plot histogram or kmeans
  * Change code so sampling continues during video
  * Calculate percentages/actual values of sensors
  * Optimize code: faster plotting (current FPS is like 1)
