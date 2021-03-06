## *********************************************************
## 
## File autogenerated for the flyer_interface package 
## by the dynamic_reconfigure package.
## Please do not edit.
## 
## ********************************************************/

##**********************************************************
## Software License Agreement (BSD License)
##
##  Copyright (c) 2008, Willow Garage, Inc.
##  All rights reserved.
##
##  Redistribution and use in source and binary forms, with or without
##  modification, are permitted provided that the following conditions
##  are met:
##
##   * Redistributions of source code must retain the above copyright
##     notice, this list of conditions and the following disclaimer.
##   * Redistributions in binary form must reproduce the above
##     copyright notice, this list of conditions and the following
##     disclaimer in the documentation and/or other materials provided
##     with the distribution.
##   * Neither the name of the Willow Garage nor the names of its
##     contributors may be used to endorse or promote products derived
##     from this software without specific prior written permission.
##
##  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
##  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
##  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
##  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
##  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
##  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
##  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
##  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
##  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
##  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
##  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
##  POSSIBILITY OF SUCH DAMAGE.
##**********************************************************/

config_description = [{'srcline': 12, 'description': 'Roll command limit [rad]', 'max': 0.89000000000000001, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '../cfg/ctrl.cfg', 'name': 'cmd_roll_limit', 'edit_method': '', 'default': 0.14999999999999999, 'level': 0, 'min': 0.0, 'type': 'double'}, {'srcline': 13, 'description': 'Roll command max delta [rad/ms]', 'max': 0.89000000000000001, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '../cfg/ctrl.cfg', 'name': 'cmd_roll_delta_limit', 'edit_method': '', 'default': 0.01, 'level': 0, 'min': 0.0, 'type': 'double'}, {'srcline': 15, 'description': 'Pitch command limit [rad]', 'max': 0.89000000000000001, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '../cfg/ctrl.cfg', 'name': 'cmd_pitch_limit', 'edit_method': '', 'default': 0.14999999999999999, 'level': 0, 'min': 0.0, 'type': 'double'}, {'srcline': 16, 'description': 'Pitch command max delta [rad/ms]', 'max': 0.89000000000000001, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '../cfg/ctrl.cfg', 'name': 'cmd_pitch_delta_limit', 'edit_method': '', 'default': 0.10000000000000001, 'level': 0, 'min': 0.0, 'type': 'double'}, {'srcline': 18, 'description': 'Yaw rate command limit [rad/s]', 'max': 4.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '../cfg/ctrl.cfg', 'name': 'cmd_yaw_rate_limit', 'edit_method': '', 'default': 1.0, 'level': 0, 'min': 0.0, 'type': 'double'}, {'srcline': 19, 'description': 'Yaw rate command max delta [rad/s/ms]', 'max': 1.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '../cfg/ctrl.cfg', 'name': 'cmd_yaw_rate_delta_limit', 'edit_method': '', 'default': 0.10000000000000001, 'level': 0, 'min': 0.0, 'type': 'double'}, {'srcline': 21, 'description': 'Thrust command limit [%]', 'max': 100.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '../cfg/ctrl.cfg', 'name': 'cmd_thrust_limit', 'edit_method': '', 'default': 70.0, 'level': 0, 'min': 0.0, 'type': 'double'}, {'srcline': 22, 'description': 'Thrust command max delta [%/ms]', 'max': 100.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '../cfg/ctrl.cfg', 'name': 'cmd_thrust_delta_limit', 'edit_method': '', 'default': 0.5, 'level': 0, 'min': 0.0, 'type': 'double'}]

min = {}
max = {}
defaults = {}
level = {}
type = {}
all_level = 0

for param in config_description:
    min[param['name']] = param['min']
    max[param['name']] = param['max']
    defaults[param['name']] = param['default']
    level[param['name']] = param['level']
    type[param['name']] = param['type']
    all_level = all_level | param['level']

