#!/usr/bin/env python
import roslib; roslib.load_manifest('topic_rate_adjuster')
import rospy
import wx
from geometry_msgs.msg import TransformStamped
from threading import Timer

defaultRate = 200

class configGUIFrame(wx.Frame):
    
    def __init__(self, parent, title):
        # Initialize variables
        self.rate = defaultRate;
        self.transformStamped = TransformStamped()
        
        # Initialize frame and panel
        wx.Frame.__init__(self, parent, title=title, size=(300,100))
        panel = wx.Panel(self, 1)

        # Create the rate slider
        self.rateSlider = wx.Slider(panel, -1,
                                    defaultRate,
                                    0, 200, wx.DefaultPosition, (300, -1),
                                    wx.SL_AUTOTICKS | wx.SL_HORIZONTAL | wx.SL_LABELS)
        
        #Add slider to a box layout
        rateBox = wx.BoxSizer(wx.VERTICAL)
        rateBox.Add(wx.StaticText(panel,-1,'Rate'),0,wx.ALIGN_CENTER)
        rateBox.Add(self.rateSlider,1)

        # Add the final box
        panel.SetSizer(rateBox)

        # Create the callback on adjustment
        self.Bind(wx.EVT_SLIDER,self.callbackSliderMoved)

        # Create the ROS subscriber
        self.input_topic = rospy.get_param('/topic_rate_adjuster/input_topic')
        self.output_topic = rospy.get_param('/topic_rate_adjuster/output_topic')
        if self.output_topic[0] == '/':
            self.output_topic = self.output_topic[1:]
        rospy.Subscriber(self.input_topic, TransformStamped, self.callbackTransformStamped)
        # Create the ROS publisher
        self.Pub = rospy.Publisher(self.output_topic, TransformStamped)
        
        # Make us visible
        self.Show(True)
        
        # Start publishing
        timer = Timer(1.0/self.rate,self.callbackPublish)
        timer.start()
        
    def callbackPublish(self):        
        # Set a timer for next publishing
        timer = Timer(1.0/self.rate,self.callbackPublish)
        timer.start()
        # Publish one message
        self.Pub.publish(self.transformStamped)

    def callbackTransformStamped(self, data):
        # Update the message
        self.transformStamped = data
        self.transformStamped.child_frame_id = self.output_topic
        
    def callbackSliderMoved(self,event):
        # Get the slider values
        self.rate = self.rateSlider.GetValue()       
        
               
def startGUI():
    rospy.init_node('rate_configGUI')
    app = wx.App(False)
    frame = configGUIFrame(None, "Configuration GUI")  
    app.MainLoop()
    
if __name__ == '__main__':
    try:
        startGUI()
    except rospy.ROSInterruptException: pass
