cd ~/Work/HYYRobotX64GRIP/build

make -j$(nproc)

sudo cp ./libJEServer.so /usr/lib/

scp robot@192.168.0.99://home/robot/Work/HYYRobotX64GRIP/build/JETestV2 /home/kleist/Documents/Code/temp
scp robot@192.168.0.99://home/robot/Work/HYYRobotX64GRIP/build/JETestV2 /home/wangyunhao/code/
