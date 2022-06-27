# PiSLAM
This is intended to be a lightweight 2D slam framework for educational purposes.  Some experiments can be found in visualization, but the actual SLAM sim is in PySLAM/world_sim.py

to run call `python3 world_sim.py`

click on the map to place a new robot waypoint.  True position in green, red is ded reckoned, and blue is graphslam output.

requires opencv, pyglet, and numpy

This is a very simple framework for fun than anything else.  It uses a graphslam approach without weights.  Features are based on sensor spikes.
