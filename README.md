you need to clone these repositories and place in the same folder as it's repo:
 
git clone https://github.com/dtecta/solid3

Use "build" subfolder for building. Cmake will find binaries here.

for example, you can create catkin workspace at place this project and solid3 in src folder

for visualisation you need to install OpengGL ang GLUT cpp libraries

also you need jsoncpp, threads, Eigen3, 

TinyXML:
sudo apt-get install libtinyxml-dev 

connect to server
 ssh -v aklyunin@ellis.tra.ai


joint - сочленение
actuator - подвижное сочленение
link - звено

отладка 
gdb binary_name
gdb --args binary (сюда писать ключи как при нормальном зауске)


run - запустить
bt - показать back_trace


