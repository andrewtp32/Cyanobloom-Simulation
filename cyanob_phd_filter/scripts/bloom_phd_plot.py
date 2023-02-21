#!/usr/bin/env python
from matplotlib import pyplot as plt
import numpy as np
import rospy
from sensor_msgs.msg import PointCloud

#include "sensor_msgs/PointCloud.h"

#interpret the sizes from header string
def get_sizes(headstr):
    #sample header -> "meas_ends=6093,prior=6182,post=6182"
    sizelist=[]
    t1=headstr.find(",")

    sizelist.append(int(headstr[10:t1])) #index of measurements end
    t2=headstr.find("=",t1)
    t3=headstr.find(",",t2)
    sizelist.append(int(headstr[t2+1:t3]))#index of prior ends
    t4=headstr.find("=",t3)
    sizelist.append(int(headstr[t4+1:len(headstr)]))#index of final estimates end
    return sizelist


def  callback_filter_results(msg):

    x=[]
    y=[]
    w=[]

    #my_array=np.array(msg.points)

    estimatesizes=[]#stores the indexes of each point types
    estimatesizes=get_sizes(msg.header.frame_id)

    #print("header count {}" .format(msg.header.seq))

    for pt in msg.points:
        x.append(pt.x)
        y.append(pt.y)
        w.append(pt.z)


    boder=plt.scatter(x[0:4],y[0:4],color="k",marker="s", label='corners')#four corners
    
    measurement=plt.scatter(x[5:estimatesizes[0]],y[5:estimatesizes[0]],color="g",marker="x", label='meas',alpha=0.5)
    
    priorestimates=plt.scatter(x[estimatesizes[0]:estimatesizes[1]],y[estimatesizes[0]:estimatesizes[1]],s=w[estimatesizes[0]:estimatesizes[1]],color="b",marker="d", label='prior',alpha=0.5)
    
    finalestimates=plt.scatter(x[estimatesizes[1]:estimatesizes[2]],y[estimatesizes[1]:estimatesizes[2]],s=w[estimatesizes[1]:estimatesizes[2]],color="r",marker="h", label='final',alpha=0.5)
    plt.legend(handles=[boder,measurement,priorestimates,finalestimates],loc='upper left')

    plt.pause(0.0001)
    plt.clf()#clear the canvas




    
def plotblooms():

    rospy.init_node('plotting_node',anonymous=True)

    fTopic=rospy.get_param('parallelfilter/results_topicName',default="/filter_results")
    
    rospy.Subscriber(fTopic,PointCloud,callback_filter_results)
    print("plotter initiated. Topic name is ", fTopic)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down plotter node")


if __name__ == '__main__':
   
    global x,y, fig

    plotblooms()

    rospy.on_shutdown()


print("node exited.......")

