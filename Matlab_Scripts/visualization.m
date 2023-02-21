%visualization for Cyno Bloom Filter
clear all

Ip ='192.168.1.29'%'192.168.1.29'
rostopic='/filter_results'%/particle_list_for_gazebo

rosshutdown;
rosinit(Ip,11311)% ip address of the other master

rostopic list
sub=rossubscriber(rostopic,"sensor_msgs/PointCloud",@callback_pointmsg)

pause(2)

%data=receive(point,10)
% data=point.LatestMessage
% DET = showdetails(data)
%rosshutdown


