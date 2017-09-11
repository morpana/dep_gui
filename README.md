to do:
roboy_dep:

experiments:
1. transitions
	objective: transition from behavior to another maintaining as much of original behavior characteristics as possible
	experiments:
	i) obtain 3 behavior pairs
	ii) 3 transfer funtions:
		a) step
			Variables:
			i) when transition is performed
		b) ramp
			Variables:
			i) when 
			ii) how quickly (period)
		c) portion of sine
			Variables:
			i) when
			ii) how quickly (period)
	data:
	- tendon space: position, velocity and force
	- video recording (regular camera) -- for qualitative evaluation
	- joint space: elbow angle
	(- lighthouse tracking?)

	evaluation:
	- how smooth is the transition?
		- transient duration
		- transient "error" i.e. jitter
			- RMS of signal to w0*A+w1*B where w0, w1 are the transfer function?
			- peak-to-peak?
			- Frequency domain analysis of transient and signals:
		mitigation:
			- filtering: depends on transient and transfer function frequency differences

dep_gui:
fix gui list labels and moving around

2. new motions...

- analyse data in python
	- read data from bags
	- visualize data
	- frequency domain representation

write data to bags: http://wiki.ros.org/rosbag/Code API
- just use:
rosbag record -O <filename> <topic>

timing aspects of the main update loop
why does it not respond to forces?
create seperate node for linear combination (rather than thread)?
	- are the messages slowing down the network in current architecture?
		not really... hz = 49.63 +- 0.02 -> good enough
		-> could move thread down to roboy_dep

add button for transition with specifying transition function

setup camera for recording experiments

https://www.ncbi.nlm.nih.gov/pubmed/19945398

add transition message for publishing transition info for rosbag