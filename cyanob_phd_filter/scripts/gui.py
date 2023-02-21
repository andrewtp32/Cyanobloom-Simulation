#!/usr/bin/env python
from tkinter import *
from tkinter import ttk
from warnings import catch_warnings
import rospy
import os
import signal
import subprocess
from cyanob_phd_filter.msg import diagnostics
import rosgraph

class gui:

    def __init__(self):

        #if rosgraph.is_master_online()==False:
        #    self.rcore=subprocess.Popen('x-terminal-emulator -e roscore',shell=True)
        

        sdTopic=rospy.get_param('diaganostic_topicName',default="/system_Diagnostics")
        self._pub=rospy.Publisher(sdTopic, diagnostics, queue_size=10, latch=True )

        rospy.Subscriber(sdTopic, diagnostics, self.callback_diagnostics)

        self.create_window()


        try:
            rospy.spin()
        except KeyboardInterrupt:
            print("Shutting down Time update node")


    def close(self):

        #print(self.rcore.pid)
        #os.kill(self.rcore.pid,signal.SIGKILL)
        #self.lgod.communicate(signal.SIGKILL)
        # if self.lgod.poll()==False:

        #self.rcore.terminate()
        #     print('lgod is alive and terminated')
        root.destroy()
        exit()

    def start_sim(self):
        if self.isstarted==False:
            msg=diagnostics()
            msg.bloom_pub='request_status'
            self._pub.publish(msg)
            isstarted=True
            self.status.set('System is started')
        else:
            self.status.set('System is already started')
            print('system is already started')

    def launch_god(self):
        self.lgod=subprocess.Popen('x-terminal-emulator -e roslaunch cyanob_phd_filter parallel_filters_only.launch',shell=True)
        self.status.set('filter launched')
        print(self.lgod.pid)

    def launch_sim(self):
        self.lsim=subprocess.Popen('x-terminal-emulator ',shell=True)
        self.status.set('gazebo simulation launched')

    def hold_sim(self):
        msg=self.diagnostics_msg_copy
        msg.time_update="hold"
        self._pub.publish(msg)



    # def calculate(*args):
    #     try:
    #         value = float(feet.get())
    #         meters.set(int(0.3048 * value * 10000.0 + 0.5)/10000.0)
    #     except ValueError:
    #         pass

    def create_window(self):
        global root
        root = Tk()
        root.resizable(True, True)
        root.title("Algae bloom system manager ")

        mainframe = ttk.Frame(root, padding="3 3 12 0")
        mainframe.grid(column=0, row=0, sticky=(N, W, E, S))
        root.columnconfigure(0, weight=2)
        root.rowconfigure(0, weight=2)

        # feet = StringVar()
        # feet_entry = ttk.Entry(mainframe, width=7, textvariable=feet)
        # feet_entry.grid(column=2, row=1, sticky=(W, E))

        # meters = StringVar()
        
        # ttk.Label(mainframe, textvariable=meters).grid(column=2, row=2, sticky=(W, E))


        ttk.Button(mainframe, text="Start", command=self.start_sim).grid(column=1, row=3, sticky=W)
        ttk.Button(mainframe, text="Hold simulation", command=self.hold_sim).grid(column=3, row=1, sticky=W)
        ttk.Button(mainframe, text="Quit", command=self.close).grid(column=1, row=6, sticky=W)
        ttk.Button(mainframe, text="launch filters", command=self.launch_god).grid(column=1, row=1, sticky=W)
        ttk.Button(mainframe, text="launch Terminal", command=self.launch_sim).grid(column=1, row=2, sticky=W)

        #self.tu, self.mu,self.sim, self.fname, self.bfl, self.isstarted, self.status
        self.isstarted=False

        col_labels=2 #colmn for the msg labels

        ttk.Label(mainframe, text="Time update").grid(column=col_labels, row=2, sticky=W)
        self.tu=StringVar()
        ttk.Label(mainframe, textvariable=self.tu, background='white', relief=SUNKEN).grid(column=col_labels+1, row=2, sticky=(W, E))

        ttk.Label(mainframe, text="Measurement update").grid(column=col_labels, row=3, sticky=W)
        self.mu=StringVar()
        ttk.Label(mainframe, textvariable=self.mu, background='white', relief=SUNKEN).grid(column=col_labels+1, row=3, sticky=(W, E))

        ttk.Label(mainframe, text="Simulation").grid(column=col_labels, row=4, sticky=W)
        self.sim=StringVar()
        ttk.Label(mainframe, textvariable=self.sim, background='white', relief=SUNKEN).grid(column=col_labels+1, row=4, sticky=(W, E))

        ttk.Label(mainframe, text="File name").grid(column=col_labels, row=5, sticky=W)
        self.fname=StringVar()
        ttk.Label(mainframe, textvariable=self.fname, background='white', relief=SUNKEN).grid(column=col_labels+1, row=5, sticky=(W, E))

        ttk.Label(mainframe, text="Blooms file location").grid(column=col_labels, row=6, sticky=W)
        self.bfl=StringVar()
        ttk.Label(mainframe, textvariable=self.bfl, background='white', relief=SUNKEN).grid(column=col_labels+1, row=6, sticky=(W, E))
        

        for child in mainframe.winfo_children(): 
            child.grid_configure(padx=5, pady=5)
        self.status=StringVar()
        ttk.Label(mainframe, textvariable=self.status, relief=FLAT).grid(column=1,columnspan=2, row=8,padx=(0,0),pady=(5,0), sticky=(E))
        #feet_entry.focus()
        #root.bind("<Return>", calculate)
        self.status.set('System is ready to initiate.Press Start..')
        root.mainloop()

    def callback_diagnostics(self,msg):
        global diagnostic_msg_copy
        self.diagnostics_msg_copy=msg
        self.tu.set(msg.time_update)
        self.mu.set(msg.measurement_update)
        self.sim.set(msg.simulation)
        self.fname.set(msg.filename)
        self.bfl.set(msg.bloom_pub)



def main():
    rospy.init_node('gui_bloom_manager',anonymous=True)
    user_interface=gui()
    rospy.on_shutdown()


if __name__ == '__main__':
    main()
    #create_window()