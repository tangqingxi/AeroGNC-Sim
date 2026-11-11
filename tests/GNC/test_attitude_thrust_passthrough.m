function info = test_attitude_thrust_passthrough()
%TEST_ATTITUDE_THRUST_PASSTHROUGH 姿态变化时推力原样传递；无输入，输出测试结果。
f = thrust_control_fixture();
bus = f.bus;
for k=1:4
    t=(k-1)*0.005;
    thrust=17+k;
    ref=struct('quaternion',euler_to_quat_bn(0.05*k,-0.02*k,0.1*k), ...
        'thrust_n',thrust,'yaw',0.1*k,'yaw_rate',0);
    bus.topics.publish('navigation/state',f.nav,t,t,t,'test');
    bus.topics.publish('control/attitude_setpoint',ref,t,t,t,'test');
    bus=attitude_p(t,bus,f.attitude);
    out=bus.topics.read('control/rate_setpoint',t);
    check_that(isequal(out.data.thrust_n,thrust),'Attitude P 改变推力');
end
% 同一固定推力配合不同姿态也必须保持不变。
ref.thrust_n=19;
for k=1:3
    t=0.02+k*0.005;
    ref.quaternion=euler_to_quat_bn(0.1*k,0,0);
    bus.topics.publish('navigation/state',f.nav,t,t,t,'test');
    bus.topics.publish('control/attitude_setpoint',ref,t,t,t,'test');
    bus=attitude_p(t,bus,f.attitude);
    check_that(bus.topics.read('control/rate_setpoint',t).data.thrust_n==19, ...
        '姿态改变导致推力被修改');
end
info=struct('passed',true,'summary','不同姿态/变化推力与固定推力均原样通过 Attitude P');
end
