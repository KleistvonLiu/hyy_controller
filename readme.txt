推荐默认（性能好且仍可调试符号）：
cmake -S . -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build -j
开启 LTO（通常值得试）：
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DHYY_ENABLE_LTO=ON
cmake --build build -j

sudo cp ./build/libJEServer.so /usr/lib/

scp robot@192.168.0.99://home/robot/Work/HYYRobotX64GRIP/build/JETestV2 /home/kleist/Documents/Code/temp
scp robot@192.168.0.99://home/robot/Work/HYYRobotX64GRIP/build/JETestV2 /home/wangyunhao/code/

robot@robot:~/Work/system/robot_config$ git push je master
fatal: unable to access 'https://github.com/KleistvonLiu/hyy_controller_config.git/': Error in the HTTP2 framing layer
robot@robot:~/Work/system/robot_config$ git config --global http.version HTTP/1.1
robot@robot:~/Work/system/robot_config$ git push je master
fatal: unable to access 'https://github.com/KleistvonLiu/hyy_controller_config.git/': Failed to connect to github.com port 443 after 129015 ms: Connection timed out
robot@robot:~/Work/system/robot_config$ sudo bash -c 'printf "\n# Prefer IPv4 over IPv6\nprecedence ::ffff:0:0/96  100\n" >> /etc/gai.conf'
[sudo] password for robot: 
robot@robot:~/Work/system/robot_config$ git push je master
Enumerating objects: 12, done.
Counting objects: 100% (12/12), done.
Delta compression using up to 4 threads
Compressing objects: 100% (12/12), done.
Writing objects: 100% (12/12), 8.66 KiB | 886.00 KiB/s, done.
Total 12 (delta 5), reused 0 (delta 0), pack-reused 0
remote: Resolving deltas: 100% (5/5), done.
To https://github.com/KleistvonLiu/hyy_controller_config.git
 * [new branch]      master -> master