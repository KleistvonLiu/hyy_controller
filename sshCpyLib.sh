#!/bin/sh
var="all"
scp ./lib/* root@192.168.0.99:/usr/lib
scp ./build/HYYRobotMain root@192.168.0.99:/robot

exit 0

