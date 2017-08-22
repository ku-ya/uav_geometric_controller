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
import random
from collections import deque
import pdb

import rospy
from uav_geometric_controller.msg import states

logging.basicConfig(level=logging.DEBUG,
        format='[%(levelname)s] (%(threadName)-10s) %(message)s',
    )

class DaqThread(Thread):
    wants_abort = False
    def run(self):
        while not self.wants_abort:
            print('thead alive')
            sleep(0.1)
            pass

class Viewer(HasTraits):
    index = Array(dtype=np.float64, shape=(100,))
    data = Array(dtype=np.float64, shape=(100,))

    plot_type = Enum("line", "scatter")

    # plot = Instance(Plot)
    #
    # traits_view = View(
    #     Item('plot',editor=ComponentEditor(), show_label=False),
    #     width=500, height=500, resizable=True, title="Error Plot")
    #
    # def _plot_default(self):
    #     x = np.linspace(-14, 14, 100)
    #     # y = x/2 * np.sin(x)
    #     # y2 = np.cos(x)
    #     plotdata = ArrayPlotData(x=self.index, y=self.data)
    #     plot = Plot(plotdata)
    #     plot.plot(('x', 'y'), type="scatter", color="blue")
    #     # plot.plot(("x", "y2"), type="line", color="red")
    #     return plot
    view = View(ChacoPlotItem("index", "data",
                          type_trait="plot_type",
                          resizable=True,
                          x_label="Time",
                          y_label="Signal",
                          color="blue",
                          bgcolor="white",
                          border_visible=True,
                          border_width=1,
                          padding_bg_color="lightgray",
                          width=800,
                          height=380,
                          marker_size=2,
                          show_label=False),
            HGroup(spring, Item("plot_type", style='custom'), spring),
            resizable = True,
            buttons = ["OK"],
            width=800, height=500)

class ErrorView(HasTraits):
    name = Str
    error_val = Enum('eW', 'eR', 'M')
    eW = deque([0]*100, 100)
    eR = deque([0]*100, 100)
    M = deque([0]*100, 100)
    time = deque([0]*100, 100)
    start_stop_motor = Button()
    mean = Float(0.0)
    stddev = Float(1.0)
    max_num_points = Int(100)
    num_ticks = Int(0)

    capture_thread = Instance(DaqThread)
    _generator = Trait(np.random.normal, Callable)
    traits_view = View(
        Group(
            Item('name'),
            Item('error_val'),
            orientation = 'horizontal',
        ),
        # Group(
        #     ChacoPlotItem('time','eW', show_label=False),
        #     ),
        Item('start_stop_motor'),
            # width = 500,
            # height = 400,
            resizable = True,
        )
    def _error_val_changed(self):
        logging.debug('errer val changed to ' + self.error_val)

    def _start_stop_motor_fired(self):
        # self.eW = np.linspace(0,10,12)
        # self.time = np.linspace(0,10,12)
        logging.debug('errer val changed')
        if self.capture_thread and self.capture_thread.isAlive():
            self.capture_thread.wants_abort = True
        else:
            self.capture_thread = DaqThread()
            self.capture_thread.wants_abort = False
            self.capture_thread.start()

        pass

    def timer_tick(self, *args):
        """
        Callback function that should get called based on a timer tick.  This
        will generate a new random data point and set it on the `.data` array
        of our viewer object.
        """
        # # Generate a new number and increment the tick count
        # new_val = self._generator(self.mean, self.stddev)
        # self.num_ticks += 1
        #
        # # grab the existing data, truncate it, and append the new point.
        # # This isn't the most efficient thing in the world but it works.
        # cur_data = self.viewer.data
        # new_data = np.hstack((cur_data[-self.max_num_points+1:], [new_val]))
        # new_index = np.arange(self.num_ticks - len(new_data) + 1,
        #                       self.num_ticks + 0.01)
        # self.eW.append(new_val)
        # self.time.append(self.num_ticks)
        error_data = eval('self.' + self.error_val)
        self.time = np.linspace(0, 1, len(error_data))

        self.viewer.index = list(self.time)
        self.viewer.data = list(error_data)
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
        self.error_window.eW.append(data.eW.x)
        pass


if __name__ == '__main__':
    rospy.init_node('error_plot', anonymous=True)
    view = main()
    rospy.Subscriber("uav_state", states, view.ros_callback)
    view.configure_traits()
    print('Completed')
