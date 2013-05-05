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

config_description = [{'srcline': 12, 'description': 'Kp ', 'max': 10.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '../cfg/pid_yaw.cfg', 'name': 'k_p', 'edit_method': '', 'default': 0.0, 'level': 0, 'min': 0.0, 'type': 'double'}, {'srcline': 13, 'description': 'Ki ', 'max': 10.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '../cfg/pid_yaw.cfg', 'name': 'k_i', 'edit_method': '', 'default': 0.0, 'level': 0, 'min': 0.0, 'type': 'double'}, {'srcline': 14, 'description': 'Kd ', 'max': 10.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '../cfg/pid_yaw.cfg', 'name': 'k_d', 'edit_method': '', 'default': 0.0, 'level': 0, 'min': 0.0, 'type': 'double'}, {'srcline': 15, 'description': 'Bias [rad/s]', 'max': 5.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '../cfg/pid_yaw.cfg', 'name': 'bias', 'edit_method': '', 'default': 0.0, 'level': 0, 'min': -5.0, 'type': 'double'}, {'srcline': 16, 'description': 'Max integral [rad/s]', 'max': 10.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '../cfg/pid_yaw.cfg', 'name': 'max_i', 'edit_method': '', 'default': 1.0, 'level': 0, 'min': 0.0, 'type': 'double'}, {'srcline': 17, 'description': 'Max error [rad]', 'max': 2.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '../cfg/pid_yaw.cfg', 'name': 'max_err', 'edit_method': '', 'default': 1.0, 'level': 0, 'min': 0.0, 'type': 'double'}]

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

