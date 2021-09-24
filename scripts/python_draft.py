'''
Author: liufeng(AT iplusbot.com)
Date: 2021-06-24 15:25:13
LastEditors: liufeng(AT iplusbot.com)
LastEditTime: 2021-09-02 22:30:08
Description: 
'''
"""
@file hough_lines.py
@brief This program demonstrates line finding with the Hough transform
"""
import sys
import math
import cv2 as cv
import numpy as np
import json

def main(argv):
    a = json.loads(json.dumps({'4': 5, '6': 7}))
    a["c"] = 9
    print( a)

if __name__ == "__main__":
    main(sys.argv[1:])
