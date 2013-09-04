# Author: Carrick Detweiler 2011
# Updated by: Jinfu Leng 2013
#!/usr/bin/env python

import roslib; roslib.load_manifest('rosbag')
import rosbag
import sys
import os
 
if len(sys.argv) < 3:
    print 'Please specify the bag file to parse followed by topic fields'
    print 'For example:'
    print '    ', sys.argv[0], 'test.bag', '/wirelessPowerRx/amps.data','/wirelessPowerRx/volts5.header.stamp'
    print
 
    print """
    Note that these fields are output together on a single line each
    time the first field is received.
    """
    exit(1)
 
#First arg is the bag to look at
bagfile = sys.argv[1]
 
#The topic fields
topicFields = []
topics = []
fields = []
for i in range(2,len(sys.argv)):
    topicFields.append(sys.argv[i])
    topics.append(sys.argv[i].split('.',1)[0])
    fields.append(sys.argv[i].split('.',1)[1])

#Record the last value of each topic field
lastValue = {}
 
#Print the values found in latValue in a comma separated format
def printValuesCSV():
    print lastValue[topicFields[0]],
    for i in range(1,len(topicFields)):
        print ",",
        print lastValue[topicFields[i]],
    print
  
#Print the first line (variable names) of the csv file
print topicFields[0],
for i in range(1, len(topicFields)):
    print ',' + topicFields[i],
print
 

#Init the lastValue dictionary to zeros
for topicField in topicFields:
    lastValue[topicField] = 0.0

#Go through the bag file
bag = rosbag.Bag(bagfile)
for topic, msg, t in bag.read_messages(topics=topics):
    for i in range(0,len(topicFields)):
        if(topics[i] == topic):
            lastValue[topicFields[i]] = eval('msg.' + fields[i])
    
    #Every time we see an instance of the first field, print out everything
    if topic == topics[0]:
        printValuesCSV()
 
bag.close();

