#!/usr/bin/env python
import numpy as np

from threading import Thread
from time import sleep
from traits.api import *
from traitsui.api import View, Item, ButtonEditor, Group
from chaco.api import ArrayPlotData, marker_trait, Plot
from chaco.chaco_plot_editor import ChacoPlotItem

class TextDisplay(HasTraits):
    string =  String()
    view= View( Item('string',show_label=False, springy=True, style='custom' ))


class CaptureThread(Thread):
    def run(self):
        self.display.string = 'Controller started\n' + self.display.string
        n_img = 0
        while not self.wants_abort:
            sleep(.5)
            n_img += 1
            self.display.string = '%d image captured\n' % n_img \
                                                    + self.display.string
        self.display.string = 'Controller stopped\n' + self.display.string

class PlotError(HasTraits):
    plot = Instance(Plot)
    eW = Array(dtype=np.float64, shape=(None))
    eR = Array(dtype=np.float64, shape=(None))
    eW = np.linspace(0,10,12)
    eR = eW
    traits_view = View(
                Group(ChacoPlotItem('eW', 'eR', show_label=False),
                    ),
                    width=400,height=300,resizable=True)
    def __init__(self):
        super(PlotError).__init__()
        x = np.linspace(-10, 10)
        y = np.sin(x)
        plotdata = ArrayPlotData(x=x,y=y)
        plot = Plot(plotdata)
        plot.plot(('x','y'),type='line',color='blue')
        self.plot = plot

class Controller(HasTraits):
    mission = Enum('takeoff', 'land', 'p2p')

    start_stop_motor = Button()
    start_stop_flight = Button()
    display = Instance(TextDisplay)
    capture_thread = Instance(CaptureThread)

    view = View(
        Item('start_stop_flight', show_label=False),
        Item('start_stop_motor', show_label=False ),
        Item('mission')
        )

    def _mission_changed(self):
        print(self.mission)

    def _start_stop_motor_fired(self):
        if self.capture_thread and self.capture_thread.isAlive():
            self.capture_thread.wants_abort = True
        else:
            self.capture_thread = CaptureThread()
            self.capture_thread.wants_abort = False
            self.capture_thread.display = self.display
            self.capture_thread.start()

class MainWindow(HasTraits):
    display = Instance(TextDisplay, ())
    controller = Instance(Controller)
    plotter = Instance(PlotError)

    def _controller_default(self):
        return Controller(display=self.display)

    view = View('display', 'controller', 'plotter', style="custom", resizable=True)



if __name__ == '__main__':
    MainWindow().configure_traits()
