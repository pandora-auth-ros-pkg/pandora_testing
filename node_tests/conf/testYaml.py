#~ import yaml
#~ 
#~ file = open('datafusion_functional.yaml')
#~ document = file.read();
#~ yadoc = yaml.load(document);
#~ 
#~ print yadoc


#!/usr/bin/env python
import roslib; roslib.load_manifest('node_tests')
import rospy

if __name__ == '__main__':
			
	rospy.init_node('yaml_tester')

	


