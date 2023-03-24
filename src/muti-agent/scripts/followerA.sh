# In Qbot
# roslaunch koboki_node minimal.launch __ns:=/followerA

# In master
source ../../../devel/setup.bash
rosrun muti-agent leader-followerA.py
