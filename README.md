# How to run
open posTracker.m and run

# Functions

## Record and control fear conditioning (FC)

Requirements:
* Web-camera connected to PC via USB
* MATLAB packages for webcam (winvideo adaptor)
* FC hardware: Arduino, foot shocker, mouse chamber with metal grid floor
* upload fear.ino to Arduino to control shocker via TTL

To-run:
* set duration
* click Directory to set the folder for saving data
* click Record to start

## Analyze video file of FC

To-run:
* click Load to open a video file 
* change duration to adjust processing time window if needed
* click Analyze
* check replay to see tracking accuracy
* if needed, change Tracking threshold to re-generate background image without mouse, then Analyze again
* check detect freezing
* click Save result

