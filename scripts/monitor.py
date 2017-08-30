#!/usr/bin/env python
import numpy as np
from traits.api import *
from traitsui.api import *
from chaco.chaco_plot_editor import ChacoPlotItem
from chaco.api import Plot, ArrayPlotData
from enable.api import ComponentEditor
from threading import Thread
from time import sleep
from pyface.timer.api import Timer
import logging
from collections import deque
import pdb
import os
import time


import rospy
from uav_geometric_controller.msg import states, trajectory

pub = rospy.Publisher('Jetson/xc', trajectory, queue_size= 10)

logging.basicConfig(level=logging.DEBUG,
        format='[%(levelname)s] (%(threadName)-10s) %(message)s',
    )

class DaqThread(Thread):
    wants_abort = False
    def __init__(self,args):
        Thread.__init__(self)
        self.args = args

    def run(self):
        print(self.args)
        while not self.wants_abort:
            print('Starting... ' + self.args)
            os.system(self.args)
            self.wants_abort = True
            print('Process:' + self.args+ ' killed')
            sleep(0.1)
            pass

class CmdThread(Thread):
    wants_abort = False
    cmd = trajectory()

    def __init__(self,args):
        Thread.__init__(self)
        self.args = args

    def run(self):
        print(self.args)
        while not self.wants_abort:
            self.cmd.header.frame_id = '/Jetson/uav'
            pub.publish(self.cmd)
            sleep(0.01)
            pass


class Viewer(HasTraits):
    index = Array(dtype=np.float64, shape=(None))
    eW_data_x = Array(dtype=np.float64, shape=(None))
    eW_data_y = Array(dtype=np.float64, shape=(None))
    eW_data_z = Array(dtype=np.float64, shape=(None))

    eR_data_x = Array(dtype=np.float64, shape=(None))
    eR_data_y = Array(dtype=np.float64, shape=(None))
    eR_data_z = Array(dtype=np.float64, shape=(None))

    M_data_x = Array(dtype=np.float64, shape=(None))
    M_data_y = Array(dtype=np.float64, shape=(None))
    M_data_z = Array(dtype=np.float64, shape=(None))

    plot_eW = Instance(Plot)
    plot_eR = Instance(Plot)
    plot_M = Instance(Plot)
    #
    traits_view = View(VGroup(
        Item('plot_eW',editor=ComponentEditor(), label='error W', show_label=False),
        Item('plot_eR',editor=ComponentEditor(), label='error R',show_label=False),
        Item('plot_M',editor=ComponentEditor(), label='Moment',show_label=False)
        ),
        width=800, height=700, resizable=True, title="Error Plot")

    def __init__(self, *args, **kw):
        super(Viewer, self).__init__(*args, **kw)
        plotdata = ArrayPlotData(time=self.index, x=self.eW_data_x)
        plot = Plot(plotdata)
        plot.plot(('time', 'x'), type="scatter", color="blue")
        self.plot_eW = plot
        self.plot_eR = plot
        self.plot_M = plot

    def _eW_data_x_changed(self):
        self.plot_eW = self.plot_default(self.index, self.eW_data_x, self.eW_data_y, self.eW_data_z)
        self.plot_eW.title = 'eW'
        spec_range = list(self.plot_eW.plots.values())[0][0].value_mapper.range
        spec_range.low = -0.5
        spec_range.high = 0.5

    def _eR_data_x_changed(self):
        self.plot_eR = self.plot_default(self.index, self.eR_data_x, self.eR_data_y, self.eR_data_z)
        self.plot_eR.title = 'eR'
        spec_range = list(self.plot_eR.plots.values())[0][0].value_mapper.range
        spec_range.low = -1.5
        spec_range.high = 1.5

    def _M_data_x_changed(self):
        self.plot_M = self.plot_default(self.index, self.M_data_x, self.M_data_y, self.M_data_z)
        self.plot_M.title = 'Moment'
        spec_range = list(self.plot_M.plots.values())[0][0].value_mapper.range
        spec_range.low = -0.5
        spec_range.high = 0.5

    def plot_default(self, time, x, y, z):
        plotdata = ArrayPlotData(time=time, x=x, y=y, z=z)
        plot = Plot(plotdata)
        plot.plot(('time', 'x'), type="line", color="red")
        plot.plot(('time', 'y'), type="line", color="green")
        plot.plot(('time', 'z'), type="line", color="blue")
        return plot

class ErrorView(HasTraits):
    name = Str('Jetson')
    host_IP = Str('161.253.73.237')
    host_IP_set = Button()
    error_val = Enum('eW', 'eR', 'M')
    ex0 = Float(0)
    ex1 = Float(0)
    ex2 = Float(0)
    xd = Array(shape=(3,))

    viewer = Instance(Viewer, ())
    N = Int(200)
    eW = np.zeros((200,3))
    eR = np.zeros((200,3))
    M = np.zeros((200,3))
    time = deque([0]*200, 200)
    start_stop_motor = Button()
    rqt_reconfig = Button(desc='starts ROS rqt_reconfig')
    # TODO: add ssh script exe on uav
    # linux terminal ssh and run script
    controller = Button()
    mapping = Button()
    exploration = Button()
    openGL = Button()
    mean = Float(0.0)
    stddev = Float(1.0)
    max_num_points = Int(100)
    num_ticks = Int(0)

    mission = Enum('halt', 'take off', 'land', 'spin', 'home')
    mission_exe = Button()

    capture_thread = Instance(DaqThread)
    rqt_thread = Instance(DaqThread)
    cmd_thread = Instance(CmdThread)

    _generator = Trait(np.random.normal, Callable)
    traits_view = View(
        Group(
            HGroup(
            Item('start_stop_motor', label='start stop motor', show_label=False),
            Item('rqt_reconfig', label='Gain tuning', show_label=False),
            Item('controller', label='Controller', show_label=False),
            Item('mapping', label='Mapping', show_label=False),
            Item('openGL', label='Visualization', show_label=False),
            Item('exploration', label='Exploration', show_label=False),
            label='Run',
            ),
            Item('ex0',label='error x',width=100),
            Item('ex1',label='error y',width=100),
            Item('ex2',label='error z',width=100),
            Item('host_IP',label='Basestation IP',width=100),
            Item('host_IP_set'),
            Item('mission', label='mission', show_label=False),
            Item('mission_exe', label='Run mission', show_label=False),
            Item('xd',label='desired position'),
        ),
        Group(
            HGroup(
            Item('name'),
            Item('error_val'),
            Item('N', label='Sample N'),
            ),
            Item('viewer', style='custom', show_label=False),
            orientation = 'vertical',
            label='UAV states',
        ),
        Group(
            Item('name'),
            label='Mapping',
        ),
        Group(
            Item('name'),
            label='Exporation',
        ),
            # width = 500,
            # height = 400,
            resizable = True,
        )

    def motor_set(self, motor, warmup):
        rospy.set_param('/'+self.name+'/uav/Motor', motor)
        rospy.set_param('/'+self.name+'/uav/MotorWarmup', warmup)

    def _host_IP_set_fired(self):
        print('export ROS_IP=' + self.host_IP)
        os.system('export ROS_IP=' + self.host_IP)

    # Eqample of decorator
    @on_trait_change('N')
    def sample_number_changed(self):
        """
        Set number of data sample to plot
        """
        self.eW = np.zeros((self.N,3))
        self.eR = np.zeros((self.N,3))
        self.M = np.zeros((self.N,3))

    def _error_val_changed(self):
        logging.debug('errer val changed to ' + self.error_val)

    def _mission_exe_fired(self):
        print('Mission starting: ' + self.mission)
        t_init = time.time()
        dt = 0.01
        z_min = 0.35
        v_up = 0.3
        t_total = 5
        x_v = [0,0,0]
        z_hover = 1.5
        cmd = self.cmd_thread.cmd

        if self.mission == 'take off':
            # self.motor_set(True,True)
            print('Motor warmup for 2 seconds')
            rospy.sleep(2)
            # motor_set(True,False)
            print('Taking off at {} sec'.format(time.time()-t_init))
            t_init = time.time()
            t_cur= 0
            cmd.b1 = [1,0,0]
            while t_cur <= t_total and self.mission == 'take off':
                t_cur = time.time() - t_init
                time.sleep(dt)
                cmd.header.stamp = rospy.get_rostime()
                height = z_min+v_up*t_cur
                cmd.xc = [x_v[0],x_v[1],height if height < z_hover else z_hover]
                cmd.xc_dot = [0,0,v_up]
                if z_hover == height:
                    continue
                self.xd = cmd.xc
                print(cmd.xc)
                # cmd_tf_pub(cmd.xc)
                # pub.publish(cmd)
            print('Take off complete')
            cmd.xc_dot = [0,0,0]
            self.mission = 'halt'

        elif self.mission == 'spin':
            # TODO
            t_cur = 0
            t_total = 15
            while t_cur <= t_total:
                t_cur = time.time() - t_init
                time.sleep(dt)
                cmd.header.stamp = rospy.get_rostime()
                theta = 2*np.pi/t_total*t_cur
                cmd.b1 = [np.cos(theta),np.sin(theta),0]
                cmd.xc = [(np.cos(theta)-1.)/2.0, 1./2.0*np.sin(theta),z_hover]
                cmd.xc_dot = [(np.sin(theta)-1.)/2.0, 1./2.0*np.sin(theta),0]
                if x_v[2] < z_min:
                    rospy.set_param('/Jetson/uav/Motor', False)
                print(cmd.xc)
            cmd.xc =[1,0,0]
            cmd.xc =[0,0,z_hover]
            cmd.xc_dot =[0,0,0]
            print(cmd.xc)
            self.mission = 'halt'

        elif self.mission == 'halt':
            # TODO
            if x_v[2] < z_min:
                rospy.set_param('/Jetson/uav/Motor', False)
            cmd.xc =[1,0,0]
            cmd.xc =[0,0,z_hover]
            cmd.xc_dot =[0,0,0]
            self.xd = cmd.xc
            self.mission = 'halt'

    def _xd_changed(self):
        print('Desired position')

    def _start_stop_motor_fired(self):
        """
        TODO
        """
        cmd = trajectory()
        cmd.b1 = [1,0,0]
        cmd.header.frame_id = '/Jetson/uav'
        self.motor_set(True,True)
        pub.publish(cmd)
        if self.cmd_thread and self.cmd_thread.isAlive():
            self.cmd_thread.wants_abort = True
            self.motor_set(False,False)
        else:
            self.cmd_thread = CmdThread(args='ls')
            self.cmd_thread.wants_abort = False
            self.cmd_thread.start()
        pass

    def _rqt_reconfig_fired(self):
        """
        Runs ros rqt_reconfigure
        """
        if self.rqt_thread and self.rqt_thread.isAlive():
            self.rqt_thread.wants_abort = True
        else:
            self.rqt_thread = DaqThread(
                args='rosrun rqt_reconfigure rqt_reconfigure'
                )
            self.rqt_thread.wants_abort = False
            self.rqt_thread.start()
        pass

    def timer_tick(self, *args):
        """
        Callback function
        """
        error_data = eval('self.' + self.error_val)
        self.time = np.linspace(0, 0.01*len(error_data), len(error_data))

        self.viewer.index = list(self.time)
        self.viewer.eW_data_x = list(error_data[:,0])
        self.viewer.eW_data_y = list(error_data[:,1])
        self.viewer.eW_data_z = list(error_data[:,2])

        self.time = np.linspace(0, 1, len(self.eR))

        self.viewer.eR_data_x = list(self.eR[:,0])
        self.viewer.eR_data_y = list(self.eR[:,1])
        self.viewer.eR_data_z = list(self.eR[:,2])

        self.viewer.M_data_x = list(self.M[:,0])
        self.viewer.M_data_y = list(self.M[:,1])
        self.viewer.M_data_z = list(self.M[:,2])

        # self.ex0 = self.ex.x
        # self.ex1 = self.ex.y
        # self.ex2 = self.ex.z
        return


class main(HasTraits):
    """
    main window and viewer handler
    """
    error_window = Instance(ErrorView)
    viewer = Instance(Viewer, ())
    error_window.viewer = viewer
    timer = Instance(Timer)
    view = View(Item('error_window', style='custom', show_label=False),
                # Item('viewer', style='custom', show_label=False),
                resizable=True)

    def edit_traits(self, *args, **kws):
        # Start up the timer! We should do this only when the main actually
        # starts and not when the main object is created.
        self.timer=Timer(10, self.error_window.timer_tick)
        return super(main, self).edit_traits(*args, **kws)

    def configure_traits(self, *args, **kws):
        self.timer=Timer(10, self.error_window.timer_tick)
        return super(main, self).configure_traits(*args, **kws)

    def _error_window_default(self):
        return ErrorView(viewer=self.viewer)

    def ros_callback(self, data):
        """
        Callback for getting all the UAV state variables
        eR
        eW
        M
        """
        self.error_window.eW = np.vstack((self.error_window.eW[1:,:],
            np.array([data.eW.x, data.eW.y, data.eW.z])))
        self.error_window.eR = np.vstack((self.error_window.eR[1:,:],
            np.array([data.eR.x, data.eR.y, data.eR.z])))
        self.error_window.M = np.vstack((self.error_window.M[1:,:],
            np.array([data.moment.x, data.moment.y, data.moment.z])))
        self.error_window.ex0, self.error_window.ex1, self.error_window.ex2  = [data.eX.x, data.eX.y, data.eX.z]
        pass


if __name__ == '__main__':
    rospy.init_node('error_plot', anonymous=True)
    view = main()
    rospy.Subscriber("Jetson/uav_states", states, view.ros_callback)
    view.configure_traits()
    view.error_window.cmd_thread.wants_abort = True
    print('Completed')
