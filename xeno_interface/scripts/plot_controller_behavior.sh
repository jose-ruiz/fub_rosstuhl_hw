#!/bin/bash

###
### @author Jose Alvarez-Ruiz <jose.alvarez-ruiz@fu-berlin.de>
### @date August 2016
###
### Creates two rqt_plot instances of desired vs. current values for the linear
### and for the angular velocities. Very useful for gain tunning.


#rqt_plot /xeno/base_controller/cmd_vel/linear/x,/xeno/base_controller/odom/twist/twist/linear/x &

#rqt_plot /xeno/base_controller/cmd_vel/angular/z,/xeno/base_controller/odom/twist/twist/angular/z &

rqt_plot /xeno/base_controller/cmd_vel/linear/x,/xeno/base_controller/cmd_vel/angular/z

