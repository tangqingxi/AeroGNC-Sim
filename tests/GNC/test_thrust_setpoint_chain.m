function info = test_thrust_setpoint_chain()
%TEST_THRUST_SETPOINT_CHAIN 四类参考的推力路径及 WTR 原生时间轴；无输入，输出测试结果。
source=struct('type','fixture','tables',shc09_aero_fixture());
constructors={@hover_scenario,@velocity_step_scenario,@attitude_step_scenario,@rate_step_scenario};
for k=1:4
    options=struct('aero_source',source,'tspan',[0 0.3],'start_s',0.05,'end_s',0.2);
    if k==1
        options.position_offset=[0.15;0;0.05];
    elseif k>=3
        options.thrust_reference_n=22;
    end
    s=constructors{k}(options);
    r=aero_sim(s.model,s.cfg);
    records=r.communication.log.records;
    names=cellfun(@(x)x.name,records,'UniformOutput',false);
    thrust=records(strcmp(names,'control/thrust_setpoint'));
    torque=records(strcmp(names,'control/torque_setpoint'));
    check_that(~isempty(thrust) && numel(thrust)==numel(torque),'双输出不完整');
    if k<=2
        sourceTopic='control/attitude_setpoint';
    elseif k==3
        sourceTopic='guidance/attitude_reference';
    else
        sourceTopic='guidance/rate_reference';
    end
    upstream=records(strcmp(names,sourceTopic));
    rateTopic='control/rate_setpoint';
    if k==4
        rateTopic=sourceTopic;
        check_that(s.cfg.experiment.definition.guidance.thrust_reference_n==22,'BodyRate 推力未进入定义');
    end
    rate=records(strcmp(names,rateTopic));
    for j=1:numel(thrust)
        actual=thrust{j};
        previous=rate{find(cellfun(@(x)x.sequence<actual.sequence,rate),1,'last')};
        if k==4
            origin=previous;
        else
            origin=upstream{find(cellfun(@(x)x.sequence<previous.sequence,upstream),1,'last')};
        end
        check_that(actual.data.thrust_n==previous.data.thrust_n ...
            && actual.data.thrust_n==origin.data.thrust_n,'推力在传递中改变或丢失');
        check_that(actual.sample_time_s==torque{j}.sample_time_s,'力矩推力时间不一致');
        check_that(previous.available_time_s<=actual.sample_time_s,'读取未来参考');
    end
    % 每个场景都检查 Allocation 使用 Rate 的推力，而非绕过链路读取 Guidance。
    allocation=records(strcmp(names,'control/allocation_status'));
    p=s.model.parameters;
    for j=1:numel(allocation)
        item=allocation{j};
        input=thrust{find(cellfun(@(x)x.sequence<item.sequence,thrust),1,'last')};
        limited=min(input.data.thrust_n,p.duct.k_T0*p.motor_device.omega_range_rad_s(2)^2);
        expected=sqrt(limited/p.duct.k_T0);
        expected=min(max(expected,p.motor_device.omega_range_rad_s(1)),p.motor_device.omega_range_rad_s(2));
        check_that(abs(item.data.motor_rad_s-expected)<1e-12,'链路推力与电机转速不符');
        check_that(input.available_time_s<=item.sample_time_s,'分配器读取未来推力');
    end
    for j=1:numel(records)
        check_that(records{j}.sample_time_s<=records{j}.available_time_s,'消息提前可用');
    end
    check_that(r.communication.log.dropped==0,'日志发生丢弃');
    root=tempname();cleanup=onCleanup(@()remove_tree(root)); %#ok<NASGU>
    out=wtr_export(r,s.model,struct('output_root',root));
    report=wtr_validate(out.wtr_path);
    check_that(report.passed,'WTR v1 独立校验失败');
    unzip(out.wtr_path,fullfile(root,'unpacked'));
    flight=load(fullfile(root,'unpacked','flight.mat'));
    curve=flight.topics.vehicle_thrust_setpoint;
    check_that(isequal(curve.time_s(:),cellfun(@(x)x.sample_time_s,thrust(:))), ...
        'WTR 未使用真实 Rate 发布时刻');
    check_that(isequal(curve.thrust_n(:),cellfun(@(x)x.data.thrust_n,thrust(:))), ...
        'WTR 推力数值发生改变');
end
% BodyRate 默认参考与历史悬停推力一致，非法推力在装配时拒绝。
s=rate_step_scenario(struct('aero_source',source));
check_that(s.cfg.experiment.definition.guidance.thrust_reference_n==s.model.trim.thrust_n, ...
    'BodyRate 默认推力改变');
for value={-1,NaN,Inf,1e5,[1 2],1+1i}
    check_error(@()rate_step_scenario(struct('aero_source',source,'thrust_reference_n',value{1})), ...
        'gnc:scenario:thrust','非法场景推力未拒绝');
end
info=struct('passed',true,'summary','四种 Reference 推力逐级一致、双输出因果、WTR 原生时刻、场景范围验证通过');
end

function remove_tree(root)
if isfolder(root)
    rmdir(root,'s');
end
end
