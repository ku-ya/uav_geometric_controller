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

import rospy
from uav_geometric_controller.msg import states

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
            print('Starting...')
            os.system(self.args)
            self.wants_abort = True
            print('Process...killed')
            sleep(0.1)
            pass


class Viewer(HasTraits):
    index = Array(dtype=np.float64, shape=(None))
    data = Array(dtype=np.float64, shape=(None))
    data_y = Array(dtype=np.float64, shape=(None))
    data_z = Array(dtype=np.float64, shape=(None))

    eR_data = Array(dtype=np.float64, shape=(None))
    eR_data_y = Array(dtype=np.float64, shape=(None))
    eR_data_z = Array(dtype=np.float64, shape=(None))

    M_data = Array(dtype=np.float64, shape=(None))
    M_data_y = Array(dtype=np.float64, shape=(None))
    M_data_z = Array(dtype=np.float64, shape=(None))

    plot_eW = Instance(Plot)
    plot_eR = Instance(Plot)
    plot_M = Instance(Plot)
    #
    traits_view = View(VGroup(
        Item('plot_eW',editor=ComponentEditor(), label='error W', show_label=True),
        Item('plot_eR',editor=ComponentEditor(), label='error R',show_label=True),
        Item('plot_M',editor=ComponentEditor(), label='Moment',show_label=True)
        ),
        width=1000, height=900, resizable=True, title="Error Plot")

    def __init__(self, *args, **kw):
        super(Viewer, self).__init__(*args, **kw)
        plotdata = ArrayPlotData(time=self.index, x=self.data)
        plot = Plot(plotdata)
        plot.plot(('time', 'x'), type="scatter", color="blue")
        self.plot_eW = plot
        self.plot_eR = plot
        self.plot_M = plot

    def _data_changed(self):
        self.plot_eW = self.plot_default(self.index, self.data, self.data_y, self.data_z)

    def _eR_data_changed(self):
        self.plot_eR = self.plot_default(self.index, self.eR_data, self.eR_data_y, self.eR_data_z)

    def _M_data_changed(self):
        self.plot_M = self.plot_default(self.index, self.M_data, self.M_data_y, self.M_data_z)

    def plot_default(self, time, x, y, z):
        plotdata = ArrayPlotData(time=time, x=x, y=y, z=z)
        plot = Plot(plotdata)
        plot.plot(('time', 'x'), type="line", color="red")
        plot.plot(('time', 'y'), type="line", color="green")
        plot.plot(('time', 'z'), type="line", color="blue")
        return plot

class ErrorView(HasTraits):
    name = Str
    error_val = Enum('eW', 'eR', 'M')
    N = Int(200)
    eW = np.zeros((200,3))#deque([0]*100, 100)
    eR = np.zeros((200,3))
    M = np.zeros((200,3))
    time = deque([0]*200, 200)
    start_stop_motor = Button()
    rqt_reconfig = Button()
    mean = Float(0.0)
    stddev = Float(1.0)
    max_num_points = Int(100)
    num_ticks = Int(0)

    mission = Enum('takeoff','land')

    capture_thread = Instance(DaqThread)
    rqt_thread = Instance(DaqThread)

    _generator = Trait(np.random.normal, Callable)
    traits_view = View(
        HGroup(
        Item('start_stop_motor', label='start stop motor', show_label=False),
        Item('rqt_reconfig', label='Gain tuning', show_label=False),
        Item('rqt_reconfig', label='Controller', show_label=False),
        Item('rqt_reconfig', label='Mapping', show_label=False),
        Item('rqt_reconfig', label='OpenGL', show_label=False),
        Item('rqt_reconfig', label='Exporation', show_label=False),
        Item('mission', label='mission', show_label=False),
        label='Run',
        ),
        Group(
            Item('name'),
            Item('error_val'),
            Item('N', label='Sample N'),
            orientation = 'horizontal',
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

    def _N_changed(self):
        self.eW = np.zeros((self.N,3))
        self.eR = np.zeros((self.N,3))
        self.M = np.zeros((self.N,3))

    def _error_val_changed(self):
        logging.debug('errer val changed to ' + self.error_val)

    def _start_stop_motor_fired(self):
        logging.debug('errer val changed')
        if self.capture_thread and self.capture_thread.isAlive():
            self.capture_thread.wants_abort = True
        else:
            self.capture_thread = DaqThread(args='ls')
            self.capture_thread.wants_abort = False
            self.capture_thread.start()
        pass

    def _rqt_reconfig_fired(self):
        logging.debug('errer val changed')
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
        self.viewer.data = list(error_data[:,0])
        self.viewer.data_y = list(error_data[:,1])
        self.viewer.data_z = list(error_data[:,2])

        self.time = np.linspace(0, 1, len(self.eR))

        self.viewer.eR_data = list(self.eR[:,0])
        self.viewer.eR_data_y = list(self.eR[:,1])
        self.viewer.eR_data_z = list(self.eR[:,2])

        self.viewer.M_data = list(self.M[:,0])
        self.viewer.M_data_y = list(self.M[:,1])
        self.viewer.M_data_z = list(self.M[:,2])
        return

class main(HasTraits):
    error_window = Instance(ErrorView)
    viewer = Instance(Viewer, ())
    timer = Instance(Timer)
    view = View(Item('error_window', style='custom', show_label=False),
                Item('viewer', style='custom', show_label=False),
                resizable=True)
    def edit_traits(self, *args, **kws):
        # Start up the timer! We should do this only when the demo actually
        # starts and not when the demo object is created.
        self.timer=Timer(10, self.error_window.timer_tick)
        return super(main, self).edit_traits(*args, **kws)
    def configure_traits(self, *args, **kws):
        # Start up the timer! We should do this only when the demo actually
        # starts and not when the demo object is created.
        self.timer=Timer(10, self.error_window.timer_tick)
        return super(main, self).configure_traits(*args, **kws)
    def _error_window_default(self):
        return ErrorView(viewer=self.viewer)

    def ros_callback(self, data):
        # pass eR eW M
        self.error_window.eW = np.vstack((self.error_window.eW[1:,:],
            np.array([data.eW.x, data.eW.y, data.eW.z])))
        self.error_window.eR = np.vstack((self.error_window.eR[1:,:],
            np.array([data.eR.x, data.eR.y, data.eR.z])))
        self.error_window.M = np.vstack((self.error_window.M[1:,:],
            np.array([data.moment.x, data.moment.y, data.moment.z])))
        pass


if __name__ == '__main__':
    rospy.init_node('error_plot', anonymous=True)
    view = main()
    rospy.Subscriber("uav_state", states, view.ros_callback)
    view.configure_traits()
    print('Completed')
