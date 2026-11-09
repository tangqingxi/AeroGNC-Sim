function value = shc09_eval_table(pp, angle, domain, policy, name)
%SHC09_EVAL_TABLE 在有效域内评估气动样条，越界行为显式化
%
% 输入：
%   pp     - 样条（pp 形式）
%   angle  - 迎角（rad）
%   domain - 有效域 [min max]（rad）
%   policy - 'error'：越界立即报错；'clip'：显式钳位到边界后求值
%   name   - 表名，用于错误信息
%
% 说明：默认 'error'，禁止静默外推；只有用户显式选择 'clip' 时才钳位，
% 且该选择记录在参数中，便于在报告中说明。

if angle < domain(1) || angle > domain(2)
    switch lower(char(policy))
        case 'error'
            error('shc09:aero:outOfDomain', ...
                '%s 迎角 %.6f rad 超出有效域 [%.6f, %.6f]', ...
                name, angle, domain(1), domain(2));
        case 'clip'
            angle = min(max(angle, domain(1)), domain(2));
        otherwise
            error('shc09:aero:policy', '未知的气动越界策略：%s', char(policy));
    end
end
value = ppval(pp, angle);

end
