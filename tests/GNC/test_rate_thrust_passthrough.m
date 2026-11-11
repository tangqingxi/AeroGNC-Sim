function info = test_rate_thrust_passthrough()
%TEST_RATE_THRUST_PASSTHROUGH 推力不影响 PID 力矩和积分；无输入，输出测试结果。
f=thrust_control_fixture();
a=f.bus;
f2=thrust_control_fixture();
b=f2.bus;
for k=1:4
    t=(k-1)/400;
    ref=struct('omega',[0.3;-0.1;0.2],'alpha',zeros(3,1),'thrust_n',10+k);
    a.topics.publish('navigation/state',f.nav,t,t,t,'test');
    b.topics.publish('navigation/state',f.nav,t,t,t,'test');
    a.topics.publish('control/rate_setpoint',ref,t,t,t,'test');
    ref.thrust_n=20+k;
    b.topics.publish('control/rate_setpoint',ref,t,t,t,'test');
    integral=zeros(3,1);
    if k>1
        integral=a.control_state.rate.integral;
    end
    expected=f.rate.rate.Kp.*(ref.omega-f.nav.omega)+integral ...
        +f.rate.rate.Kff.*ref.omega-f.rate.rate.Kd.*f.nav.alpha;
    a=rate_pid(t,a,f.rate);
    b=rate_pid(t,b,f.rate);
    tau=a.topics.read('control/torque_setpoint',t);
    thrust=a.topics.read('control/thrust_setpoint',t);
    check_that(isequal(thrust.data.thrust_n,10+k),'Rate 改变推力');
    check_that(isequal(tau.data.torque,expected),'原 PID 力矩公式改变');
    check_that(isequal(tau.data.torque,b.topics.read('control/torque_setpoint',t).data.torque), ...
        '推力影响力矩输出');
    check_that(isequal(a.control_state.rate,b.control_state.rate),'推力影响积分状态');
    check_that(tau.sample_time_s==thrust.sample_time_s ...
        && tau.available_time_s==thrust.available_time_s,'两个输出的时间不一致');
end
% 缺少字段、负值、过期参考都不能静默生成默认推力。
a.topics.register('test/missing_thrust',aero_topic_spec({'omega','alpha'}, ...
    {[3 1],[3 1]},{'rad/s','rad/s^2'}));
a.topics.publish('test/missing_thrust',struct('omega',zeros(3,1),'alpha',zeros(3,1)),0.01,0.01,0.01,'test');
p=f.rate;p.rate_topic='test/missing_thrust';
check_error(@() rate_pid(0.01,a,p),'gnc:rate:thrust','缺少推力未拒绝');
ref.thrust_n=-1;
a.topics.publish('control/rate_setpoint',ref,0.02,0.02,0.02,'test');
check_error(@() rate_pid(0.02,a,f.rate),'gnc:rate:thrust','负推力未拒绝');
sequence=a.topics.sequence();
a=rate_pid(0.3,a,f.rate);
check_that(a.topics.sequence()==sequence,'过期输入仍生成控制输出');
info=struct('passed',true,'summary','Rate 推力原样传递、双输出同时间、PID/积分不变及非法参考拒绝通过');
end
