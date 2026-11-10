function value = ideal_sensor_sample(x, layout)
%IDEAL_SENSOR_SAMPLE 理想传感器采样（只经 Sensor 回调读取真值）
%
% 输出 13×1：[p_n(3); v_n(3); q_bn(4); omega_b(3)]。
% 该函数只由 scenario 的 sensor 组件声明使用，不是生产导航估计器。

value = [x(layout.position); x(layout.velocity); ...
    x(layout.quaternion); x(layout.omega)];

end
