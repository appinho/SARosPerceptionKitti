#!/bin/bash
echo "Run benchmark"
source /home/ANT.AMAZON.COM/sappel/catkin_ws/devel/setup.bash
for i in 0000 0001 0002 0003 0004 0006 0010 0011 0012 0013
do
  	roslaunch evaluation evaluation.launch scenario:=$i
	PID=$!
	# Wait for 2 seconds
	sleep 2
	# Kill it
	kill $PID
done
echo "Done benchmark"
cd ./python
python evaluate_tracking.py