#!/usr/bin/env python

import rospy
import matplotlib.pyplot as plt
from novatel_gps.msg import LogAll

computer_time = [0]
gps_time = [0]

def callback_logall(data):
    global computer_time
    global gps_time
    computer_time.append( data.header.stamp.secs - 1537820224 )
    gps_time.append( (int)(data.msg_header.gps_ms // 1e3)  )

def main():
    rospy.init_node('gps_computer_time', anonymous=False)
    rospy.Subscriber("/gps/all", LogAll, callback_logall)
    rate = rospy.Rate(1) # 1hz

    # fig = plt.figure()
    # ax = fig.add_subplot(1, 1, 1)
    plt.ion()
    while not rospy.is_shutdown():
        # plot the data
        # print xdata
        # plt.xlim(xdata.min(), xdata.max())
        # plt.ylim(computer_time.min(), computer_time.max())
        xdata = range(0, len(computer_time))
        plt.plot(xdata, computer_time, 'r', xdata, gps_time, 'b')
        # display the plot
        # print computer_time
        # print gps_time
        plt.title("GPS Time X Computer Time")
        plt.show()
        plt.draw()
        plt.pause(0.01)
        rate.sleep()

    plt.show(block=True)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
