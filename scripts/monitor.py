#!/usr/bin/env python
#from __future__ import print_function, division
import numpy as np
from traits.api import Array, Instance, Int, HasTraits, Str, Button, Enum
from traits.api import Trait, Callable, on_trait_change, Float, Bool
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

# publisher for the desired trajectory
pub = rospy.Publisher('Jetson/xc', trajectory, queue_size= 1)

logging.basicConfig(level=logging.DEBUG,
        format='[%(levelname)s] (%(threadName)-10s) %(message)s',
    )

class Run_thread(Thread):
    """Use this to run as terminal command
    """
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

class CmdThread(Thread, HasTraits):
    """Trajectory command thread
    """
    wants_abort = False
    cmd = trajectory()
    cmd.b1  = [1,0,0]
    cmd.xc = [0,0,0]
    mission = 'halt'
    t_init = time.time()
    t_cur = 0
    name = 'Jetson'
    x_v = Array(shape=(3,))

    def __init__(self,args):
        Thread.__init__(self)
        self.args = args

    def motor_set(self, motor, warmup):
        rospy.set_param('/'+self.name+'/uav/Motor', motor)
        rospy.set_param('/'+self.name+'/uav/MotorWarmup', warmup)

    def run(self):
        print(self.args)
        print('Process: cmd thread started')
        self.cmd.header.frame_id = '/Jetson/uav'

        dt = 0.1
        z_min = 0.2
        v_up = 0.3
        t_total = 5
        z_hover = 1.5
        cmd = self.cmd
        cmd.xc = [0,0,0]
        cmd.xc_dot = [0,0,0]
        cmd.xc_2dot = [0,0,0]
        cmd.b1 = [1,0,0]
        self.motor_set(True,False)
        motor_flag = False
        pub.publish(self.cmd)

        while not self.wants_abort:
            #print('Mission: ' + self.mission, end='')
            self.t_cur = time.time() - self.t_init
            cmd.header.stamp = rospy.get_rostime()

            print(', time: {:2.4f} sec'.format(self.t_cur))

            if self.mission == 'take off':
                motor_flag = True
                cmd.b1 = [1,0,0]
                if self.t_cur <= t_total and self.mission == 'take off':
                    height = z_min+v_up*self.t_cur
                    cmd.xc = [self.x_v[0],self.x_v[1],height if height < z_hover else z_hover]
                    cmd.xc_dot = [0,0,v_up*dt]
                    if z_hover == height:
                        continue
                    self.xd = cmd.xc
                else:
                    self.mission = 'halt'
                # print('Take off complete')

            elif self.mission == 'spin':
                # TODO
                t_total = 30
                if self.t_cur <= t_total:
                    theta = 2*np.pi/t_total*self.t_cur
                    cmd.b1 = [np.cos(theta),np.sin(theta),0]
                    cmd.xc = [(np.cos(theta)-1.)/2.0, 1./2.0*np.sin(theta),z_hover]
                    cmd.xc_dot = [dt*np.sin(theta)/2.0, dt*1./2.0*np.sin(theta),0]
                    # if x_v[2] < z_min:
                        # rospy.set_param('/Jetson/uav/Motor', False)
                    # print(cmd.xc)
                else:
                    self.mission = 'halt'
                # cmd.xc =[1,0,0]
                # cmd.xc =[0,0,z_hover]
                # cmd.xc_dot =[0,0,0]
            elif self.mission == 'land':
                t_total = 5
                cmd.b1 = [1,0,0]
                if self.t_cur <= t_total and self.mission == 'land':
                    height = z_hover - (v_up*self.t_cur)
                    cmd.xc[2] = height if height > z_min else z_min
                    # cmd.xc = [x_v[0],x_v[1],height if height > z_min else z_min]
                    cmd.xc_dot = [0,0,0]
                    if z_min == height:
                        continue
                    self.xd = cmd.xc
                else:
                    self.motor_set(False,False)
                    self.wants_abort = True
                    # print(cmd.xc)
                    # cmd_tf_pub(cmd.xc)
                    # pub.publish(cmd)

            elif self.mission == 'halt':
                # TODO
                # if x_v[2] < z_min:
                    # rospy.set_param('/Jetson/uav/Motor', False)
                cmd.b1 =[1,0,0]
                cmd.xc[2] = z_hover
                cmd.xc_dot =[0,0,0]
                self.t_cur= 0
                self.t_init = time.time()

    		if not motor_flag:
    		    cmd.xc =self.x_v
                cmd.xc_dot =[0,0,0]

            self.xd = cmd.xc
            pub.publish(self.cmd)
            time.sleep(dt)
            pass

        self.motor_set(False,False)
        print('Process: cmd thread killed')


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
        spec_range.low = -1
        spec_range.high = 1

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
    mapping_name = Str('uav_demo ...')
    exploration_name = Str('uav_demo ...')
    host_IP = Str('161.253.73.237')
    host_IP_set = Button()
    error_val = Enum('eW', 'eR', 'M')
    ex0 = Float(0)
    ex1 = Float(0)
    ex2 = Float(0)
    xd = Array(shape=(3,))
    x_v = Array(shape=(3,))

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
    rviz = Button()
    openGL = Button()
    abort = Button()
    mean = Float(0.0)
    stddev = Float(1.0)
    max_num_points = Int(100)
    num_ticks = Int(0)

    mission = Enum('take off', 'land', 'spin', 'home', 'halt')
    mission_exe = Button()
    landing_exe = Button()
    reset = Button()

    motor_bool = Bool

    capture_thread = Instance(Run_thread)
    rqt_thread = Instance(Run_thread)
    mapping_thread = Instance(Run_thread)
    cmd_thread = Instance(CmdThread)

    _generator = Trait(np.random.normal, Callable)
    traits_view = View(
        Group(
            HGroup(
            Item('start_stop_motor', label='start stop motor', show_label=False),
            Item('rqt_reconfig', label='Gain tuning', show_label=False),
            Item('controller', label='Controller', show_label=False),
            Item('openGL', label='Visualization', show_label=False),
            Item('rviz', label='Rviz', show_label=False),
            label='Run',
            ),
            Item('ex0',label='error x',width=100),
            Item('ex1',label='error y',width=100),
            Item('ex2',label='error z',width=100),
            Item('host_IP',label='Basestation IP',width=100),
            Item('host_IP_set'),
            HGroup(
                Item('mission', label='Mission', show_label=True),
                Item('mission_exe', label='Run mission', show_label=False),
                Item('landing_exe', label='Land', show_label=False),
            ),
            HGroup(
            Item('xd',label='desired position'),
            Item('x_v',label='vicon position'),
            Item('reset'),
            ),
            HGroup(
                Item('mapping_name', label='Mapping command', show_label=True),
                Item('mapping', label='Mapping', show_label=False),
            ),
            HGroup(
                Item('exploration_name', label='Exploration command', show_label=True),
                Item('exploration', label='Exploration', show_label=False),
            ),
            Item('abort', label='Stop motor', show_label=False),
            Item('motor_bool', label='Motor', show_label=True),
            label='Execution',
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
        if (not motor) and (not warmup):
            self.motor_bool = False

    def _abort_fired(self):
        self.motor_set(False, False)
        self.motor_bool = False

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

    def _reset_fired(self):
        cmd = trajectory()
        cmd.xc_2dot = [0,0,0]
        cmd.xc_dot = [0,0,0]
        cmd.xc = [0,0,0]
        self.xd = cmd.xc
        cmd.b1 = [1,0,0]
        pub.publish(cmd)

    def _xd_changed(self):
        cmd = trajectory()
        cmd.xc_2dot = [0,0,0]
        cmd.xc_dot = [0,0,0]
        cmd.xc = self.xd
        cmd.b1 = [1,0,0]
        # print('Desired position, x: ' + cmd.xc[0]+' y: '+cmd.xc[1]+' z: '+ cmd.xc[2])
        # pub.publish(cmd)

    def _mission_exe_fired(self):
        print('Mission fired: ' + self.mission)
        self.cmd_thread.t_cur = 0
        self.cmd_thread.t_init = time.time()
        self.cmd_thread.mission = self.mission

    def _landing_exe_fired(self):
        self.mission = 'land'
        print('Mission: ' + self.mission)
        self.cmd_thread.t_cur = 0
        self.cmd_thread.t_init = time.time()
        self.cmd_thread.mission = self.mission

    def _start_stop_motor_fired(self):
        """
        TODO
        """
        self.motor_set(True,True)
        if self.cmd_thread and self.cmd_thread.isAlive():
            self.motor_set(False,False)
            self.cmd_thread.wants_abort = True
        else:
            self.cmd_thread = CmdThread(args='')
            self.motor_set(True, True)
            self.motor_bool = True
            self.cmd_thread.wants_abort = False
            self.cmd_thread.start()
        pass

    def _x_v_changed(self):
        if self.cmd_thread and self.cmd_thread.isAlive():
            self.cmd_thread.x_v = self.x_v
        else:
            pass
        pass

    def _rqt_reconfig_fired(self):
        """
        Runs ros rqt_reconfigure
        """
        if self.rqt_thread and self.rqt_thread.isAlive():
            self.rqt_thread.wants_abort = True
        else:
            self.rqt_thread = Run_thread(
                args='rosrun rqt_reconfigure rqt_reconfigure'
                )
            self.rqt_thread.wants_abort = False
            self.rqt_thread.start()
        pass

    def _mapping_fired(self):
        """
        Runs ros mapping
        """
        if self.mapping_thread and self.mapping_thread.isAlive():
            self.mapping_thread.wants_abort = True
        else:
            self.mapping_thread = Run_thread(
                args=self.mapping_name
                )
            self.mapping_thread.wants_abort = False
            self.mapping_thread.start()
        pass

    def timer_tick(self, *args):
        """Callback function
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

        return


class main(HasTraits):
    """Main window and viewer handler
    """

    error_window = Instance(ErrorView)
    viewer = Instance(Viewer, ())
    error_window.viewer = viewer
    timer = Instance(Timer)
    view = View(Item('error_window', style='custom', show_label=False),
                # Item('viewer', style='custom', show_label=False),
                resizable=True)

    def __init__(self):
        """initialization of node
        """
        rospy.init_node('error_plot', anonymous=True)
        self.sub = rospy.Subscriber("Jetson/uav_states", states, self.ros_callback)

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
        self.error_window.ex0, self.error_window.ex1, self.error_window.ex2  = [data.ex.x, data.ex.y, data.ex.z]
        self.error_window.x_v = [data.x_v.x, data.x_v.y, data.x_v.z]
        pass


if __name__ == '__main__':
    view = main()
    view.configure_traits()
    view.error_window.motor_set(False,False)
    view.error_window.cmd_thread.wants_abort = True
    print('Completed')
