# URC 2023 - Controls Stack
All of the python, cpp and ino files for URC 2023

****make sure to build the urc2023 package on a new workspace****

to start videofeeds:
1. on kratos terminal run videofeed#.cpp;  
    rosrun urc2023 videofeed# @
    
    #-1,2,3
    ; @ - 0,1,2
2. on the basestation 
    roslaunch urc2023 cameras_feed.launch

for panorama:  
*camera movement should be as small as possible for panorama.py to stitch it*  
*captured images and stitched pano can be found in jetson's /home/Captures *
1. on kratos terminal run panorama_captures.py  
     rosrun urc2023 panorama_captures.py
2. Press 1 and enter to capture images
3. to stitch the images:
     rosrun urc2023 panorama.py

## stm32 and Embassy
Embassy is the library used for asynchronous programming on the STM32, and needs to be in the same folder as the code
The stm32 folder contains the various programs required to run the microcontrollers on the rover
To build these programs, install `cargo`, `cd` into the directory of the program and run
```sh
cargo run --release
```
