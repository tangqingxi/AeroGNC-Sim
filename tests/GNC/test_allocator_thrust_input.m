function info = test_allocator_thrust_input()
%TEST_ALLOCATOR_THRUST_INPUT 仅 torque/thrust 输入可完成分配；无输入，输出测试结果。
f=thrust_control_fixture();
bus=struct('topics',aero_topic_bus(),'command',f.trim.command);
specs=topic_specs();
for k=1:numel(specs)
    if ismember(specs{k}.name,{'control/torque_setpoint','control/thrust_setpoint','control/allocation_status'})
        bus.topics.register(specs{k}.name,specs{k}.spec);
    end
end
bus.topics.publish('control/torque_setpoint',struct('torque',zeros(3,1)),0,0,0,'test');
sequence=bus.topics.sequence();
original=bus.command;
bus=pinv_allocator(0,bus,f.allocation);
check_that(bus.topics.sequence()==sequence && isequal(bus.command,original), ...
    '缺少推力仍生成分配结果或采用 hover fallback');
for k=1:3
    t=k*0.01;thrust=10*k;
    bus.topics.publish('control/torque_setpoint',struct('torque',[0.02;0;0]),t,t,t,'test');
    bus.topics.publish('control/thrust_setpoint',struct('thrust_n',thrust),t,t,t,'test');
    bus=pinv_allocator(t,bus,f.allocation);
    out=bus.topics.read('control/allocation_status',t);
    expected=sqrt(thrust/f.allocation.k_T0);
    ve=sqrt(thrust/(f.allocation.diffuser_ratio*f.allocation.air_density*f.allocation.disc_area));
    check_that(abs(out.data.motor_rad_s-expected)<1e-12,'电机标量求解错误');
    check_that(abs(out.data.ve_nominal_m_s-ve)<1e-12,'Ve 未按输入推力计算');
end
% 推力有效而力矩过期时，也不得生成新结果。
bus.topics.publish('control/thrust_setpoint',struct('thrust_n',20),0.3,0.3,0.3,'test');
sequence=bus.topics.sequence();
bus=pinv_allocator(0.3,bus,f.allocation);
check_that(bus.topics.sequence()==sequence,'缺少有效力矩仍生成分配结果');
% 执行器物理限幅仍保留。
bus.topics.publish('control/torque_setpoint',struct('torque',zeros(3,1)),0.31,0.31,0.31,'test');
bus.topics.publish('control/thrust_setpoint',struct('thrust_n',1e4),0.31,0.31,0.31,'test');
bus=pinv_allocator(0.31,bus,f.allocation);
check_that(bus.command.motor==f.allocation.motor_max_rad_s,'电机物理上限未保持');
info=struct('passed',true,'summary','仅双控制输入、无 fallback、缺失/过期不更新、motor/Ve 与限幅通过');
end
