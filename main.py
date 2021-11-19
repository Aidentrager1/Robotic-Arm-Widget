# ////////////////////////////////////////////////////////////////
# //                     IMPORT STATEMENTS                      //
# ////////////////////////////////////////////////////////////////

import math
import sys
import time
from threading import Thread

from kivy.app import App
from kivy.lang import Builder
from kivy.core.window import Window
from kivy.uix.screenmanager import ScreenManager, Screen
from kivy.uix.button import Button
from kivy.uix.floatlayout import FloatLayout
from kivy.graphics import *
from kivy.uix.popup import Popup
from kivy.uix.label import Label
from kivy.uix.widget import Widget
from kivy.uix.slider import Slider
from kivy.uix.image import Image
from kivy.uix.behaviors import ButtonBehavior
from kivy.clock import Clock
from kivy.animation import Animation
from functools import partial
from kivy.config import Config
from kivy.core.window import Window
from pidev.kivy import DPEAButton
from pidev.kivy import PauseScreen
from time import sleep
import RPi.GPIO as GPIO 
from pidev.stepper import stepper
from pidev.Cyprus_Commands import Cyprus_Commands_RPi as cyprus

import spidev
import os
from time import sleep
import RPi.GPIO as GPIO
from pidev.stepper import stepper
from Slush.Devices import L6470Registers
from pidev.Cyprus_Commands import Cyprus_Commands_RPi as cyprus
spi = spidev.SpiDev()

cyprus.initialize()
cyprus.setup_servo(2)
cyprus.set_servo_position(2, 0.5)

s0 = stepper(port=0, micro_steps=32, hold_current=20, run_current=20, accel_current=20, deaccel_current=20,
             steps_per_unit=200, speed=2)
# ////////////////////////////////////////////////////////////////
# //                      GLOBAL VARIABLES                      //
# //                         CONSTANTS                          //
# ////////////////////////////////////////////////////////////////
START = True
STOP = False
UP = False
DOWN = True
ON = True
OFF = False
YELLOW = .180, 0.188, 0.980, 1
BLUE = 0.917, 0.796, 0.380, 1
CLOCKWISE = 0
COUNTERCLOCKWISE = 1
ARM_SLEEP = 2.5
DEBOUNCE = 0.10

lowerTowerPosition = 60
upperTowerPosition = 76


# ////////////////////////////////////////////////////////////////
# //            DECLARE APP CLASS AND SCREENMANAGER             //
# //                     LOAD KIVY FILE                         //
# ////////////////////////////////////////////////////////////////
class MyApp(App):

    def build(self):
        self.title = "Robotic Arm"
        return sm

Builder.load_file('main.kv')
Window.clearcolor = (.1, .1,.1, 1) # (WHITE)

cyprus.open_spi()

# ////////////////////////////////////////////////////////////////
# //                    SLUSH/HARDWARE SETUP                    //
# ////////////////////////////////////////////////////////////////

sm = ScreenManager()
arm = stepper(port = 0, speed = 10)

# ////////////////////////////////////////////////////////////////
# //                       MAIN FUNCTIONS                       //
# //             SHOULD INTERACT DIRECTLY WITH HARDWARE         //
# ////////////////////////////////////////////////////////////////
	
class MainScreen(Screen):
    version = cyprus.read_firmware_version()
    armPosition = 0
    lastClick = time.clock()
    ON = True
    Position = "up"
    run = True
    def __init__(self, **kwargs):
        super(MainScreen, self).__init__(**kwargs)
        self.initialize()

    def debounce(self):
        processInput = False
        currentTime = time.clock()
        if ((currentTime - self.lastClick) > DEBOUNCE):
            processInput = True
        self.lastClick = currentTime
        return processInput

    def toggleArm(self):
        print("Process arm movement here")
        if self.Position == "up":
            cyprus.set_pwm_values(1, period_value=100000, compare_value=100000, compare_mode=cyprus.LESS_THAN_OR_EQUAL)
            self.Position = "down"
        else:
            cyprus.set_pwm_values(1, period_value=100000, compare_value=0, compare_mode=cyprus.LESS_THAN_OR_EQUAL)
            self.Position = "up"
    def toggleMagnet(self):
        print("Process magnet here")
        if self.ON:
            cyprus.set_servo_position(2, 0)
            self.ON = False
        else:
            cyprus.set_servo_position(2, 0.5)
            self.ON = True
    def start_movement_thread(self):  # This should be inside the MainScreen Class
        Thread(target=self.auto).start()
    def auto(self):11
        print("Run the arm automatically here")
        while True:
            if self.run:
                self.ON = False
                self.toggleMagnet()
                self.Position = "down"
                self.toggleArm()
                sleep(1)
                s0.go_until_press(0, 4000)
                while s0.isBusy():
                    sleep(0.1)
                s0.set_as_home()
                s0.start_relative_move(2.3)
                while s0.isBusy():
                    sleep(0.1)
                self.toggleArm()
                self.toggleMagnet()
                sleep(4)
                self.toggleArm()
                sleep(1)
                s0.start_relative_move(.68)
                sleep(1)
                self.toggleArm()
                sleep(2)
                self.toggleMagnet()
                self.toggleArm()
                self.run = False
                sleep(3)
            else:
                self.ON = True
                self.toggleMagnet()
                self.Position = "down"
                self.toggleArm()
                sleep(1)
                self.ON = False
                self.toggleMagnet()
                sleep(1)
                s0.go_until_press(0, 4000)
                while s0.isBusy():
                    sleep(0.1)
                s0.set_as_home()
                s0.start_relative_move(2.98)
                while s0.isBusy():
                    sleep(0.1)
                self.toggleArm()
                self.toggleMagnet()
                sleep(1)
                self.toggleArm()
                sleep(1)
                s0.start_relative_move(-.68)
                self.run = True
                while s0.isBusy():
                    sleep(0.1)
                self.toggleArm()
                sleep(4)
                self.toggleMagnet()
                self.toggleArm()
                sleep(3)
    def setArmPosition(self, position):
        print("Move arm here")
        s0.go_to_position(position/25)
        self.armControlLabel.text = "Arm Position: " + str(position)
        sleep(.01)

    def homeArm(self):
        arm.home(self.homeDirection)
        
    def isBallOnTallTower(self):
        print("Determine if ball is on the top tower")

    def isBallOnShortTower(self):
        print("Determine if ball is on the bottom tower")
        
    def initialize(self):
        print("Home arm and turn off magnet")

    def resetColors(self):
        self.ids.armControl.color = YELLOW
        self.ids.magnetControl.color = YELLOW
        self.ids.auto.color = BLUE

    def quit(self):
        MyApp().stop()
    
sm.add_widget(MainScreen(name = 'main'))


# ////////////////////////////////////////////////////////////////
# //                          RUN APP                           //
# ////////////////////////////////////////////////////////////////

MyApp().run()
cyprus.close_spi()
