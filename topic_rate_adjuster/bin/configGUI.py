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
        rospy.Subscriber("vicon/FISHTANK/FISHTANK", TransformStamped, self.callbackTransformStamped)
        # Create the ROS publisher
        self.Pub = rospy.Publisher('vicon/FISHTANK_A/FISHTANK_A', TransformStamped)
        
        # Make us visible
        self.Show(True)
        
        timer = Timer(1.0/self.rate,self.callbackPublish)
        timer.start()
        
    def callbackPublish(self):        
        # Set a timer
        timer = Timer(1.0/self.rate,self.callbackPublish)
        timer.start()
        self.Pub.publish(self.transformStamped)

    def callbackTransformStamped(self, data):
        self.transformStamped = data
        
    def callbackSliderMoved(self,event):
        # Get the slider values
        self.rate = self.rateSlider.GetValue()       
        #rospy.loginfo('New setting: rate: %d', self.rate)
        
        
        
def startGUI():
    rospy.init_node('rate_configGUI')
    app = wx.App(False)
    frame = configGUIFrame(None, "Configuration GUI")  
    app.MainLoop()
    
if __name__ == '__main__':
    try:
        startGUI()
    except rospy.ROSInterruptException: pass
