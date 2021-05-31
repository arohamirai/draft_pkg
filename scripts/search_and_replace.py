#!/usr/bin/env python
# coding: utf-8

import os

g = os.walk(r'/home/lile/ibvs_ws/src/visual_servo_iplus/visual_servo_action/src')

for fileroot, dirnames, filenames in g:
    for filename in filenames:
        absolute_file_path = os.path.join(fileroot, filename)
        with open(absolute_file_path, 'r+') as f:
            lines = f.readlines()
            f.seek(0)  
            for s in lines:
                f.write( s.replace('as_.','as_->'))
