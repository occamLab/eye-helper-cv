#!/bin/bash

rosbag record -O $1 -a -ex '(.*)image_raw'
