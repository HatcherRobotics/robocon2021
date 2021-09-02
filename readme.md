# use one single scan LiDAR for detection in robocon2021
## theme
The theme of robocon2021 is about two robots, one for defence and the other try to throw arrows into pots.
## analysis
Our throw robot have three arrows,and my task is to use sensors to locate the pots in the frames of arrow1,2,3.<br/>
The key element to success is how to measure the position of pots in  the frame of arrows.</br>One method is to use RGB-D camera, but in the experiment even though it can distinguish red and blue pots, the accuracy of the distance can only be kept in five meters.<br/>While for the single scan LiDAR, the accuracy is much higher but the data is sparse, and it cannot distinguish whether the pot is red or blue.
## sensor message
With LiDAR we can have the distance of 1081 ranges in 270 degrees,which means that the angle resolution is 0.25degrees.The serial number of laser is named from 0 to 1081 counterclockwise.<br/>
## algorithm
### data pre-processing
The laser emiited may not be back, so for the ranges much small or large should be idealized.

### find the numbers of lasers on the center of the pot 
The most significant thing is how to find the ranges right on the center of pots.Since the distance will sharply change between the laser onto the edge of the pot and the last laser,the numbers of two edges of each pot is easy to find by the diffenence equation.the middle laser between the two lasers can be considered as the one onto the center of the pot.
### find the distance and angle in the frame of arrow
The laser point on the cneter of pot, the arrow tail point and the laser emitter point form a triangular, then use cosine law to solve the distances between the pots and arrows,then use sine law to solve the angles that between the pot-arrow line and the vertical line on arrow.<br/>
Move the robot to make the angle be zero,then the arrow is aimed at the pot and distance is  the arrow to the pot.
### serial communication
Use serial communication from Jetson NX to STM32, pass the array[angle,distance] of arrow1,2,3  