# FRDM-TFC
_____
Cup Car Project for Microcomputer Systems II class

### What is it?
The Cup Car is a small autonomous vehicle that traverses a race track. The car consists of the following hardware:

+ 1 12-bit 1x128 pixel Linescan Camera @25fps
+ 2 Motors
+ 1 H-Bridge
+ 1 Servo
+ 1 NXP FRDM-KL25Z Microcontroller
+ 1 NXP made interface board with 2 POTS

### How does it work?
We wrote a simple image processing algorithm that takes the data from the Linescan Camera and transforms it by taking the `central finite difference` to get the 1st derivative of the signal. Then, we use each peak to determine where the edges of the track are. The track is white, with black borders, so the input signal from the camera appears as a trapezoid. With this processed data, we developed a PID controller that used the difference between the current and the last frame to determine the drive and steering.

### What are it's limitations?
The final result was a car that could drive around a track built to specification, under the conditions that:
+ there was sufficient lighting
+ the car did not travel faster than the algorithm could process the camera data
+ there were no steep gradients on the track

### What could be done to improve it?
There are several ways to improve the image processing algorithm and the PID controller.
+ The camera lens is `wide-angle`, meaning the raw signal is dimmer on the edges. Additional processing is needed to account for this 'cosine' effect.
+ The edge detection algorithm should work as a state machine with 4 states: no edges, left edge only, right edge only, both edges detected.
+ The car should be able to detect when it is on a gradient using the accelerometer built into the KL25Z, this should be used to provide additional drive when needed

### Final Performance
Link to video coming...
