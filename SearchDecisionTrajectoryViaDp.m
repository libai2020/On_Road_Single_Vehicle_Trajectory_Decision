%  MATLAB Source Codes for the book "Cooperative Decision and Planning for
%  Connected and Automated Vehicles" published by Mechanical Industry Press
%  in 2020.
% 《智能网联汽车协同决策与规划技术》书籍配套代码
%  Copyright (C) 2020 Bai Li
%  2020.02.14
% ==============================================================================
%  第四章 4.2节 动态规划搜索最优决策轨迹
% ==============================================================================
function [x, y, theta] = SearchDecisionTrajectoryViaDp()
global state_space dp_
NT = dp_.num_t_grids;
NS = dp_.num_s_grids;
NL = dp_.num_l_grids;
state_space = cell(NT, NS, NL);
node_default_status = [Inf, -1, -1, -Inf]; % cost val (1), parent ind (2-3), parent's absolute s
% 状态空间初始化
for ii = 1 : NT
    for jj = 1 : NS
        for kk = 1 : NL
            state_space{ii, jj, kk} = node_default_status;
        end
    end
end
% 打表
for jj = 1 : NS
    for kk = 1 : NL
        [cost_val, s_achieved] = EstimateCostZero(jj,kk);
        state_space{1, jj, kk} = [cost_val, -999, -999, s_achieved];
    end
end
for ii = 1 : (NT-1)
    for jj = 1 : NS
        for kk = 1 : NL
            cur_state_ind = [ii, jj, kk];
            for mm = 1 : NS
                for nn = 1 : NL
                    next_layer_state_ind = [ii+1, mm, nn];
                    [delta_cost, s_achieved] = EstimateCost(cur_state_ind, next_layer_state_ind);
                    cost = state_space{ii, jj, kk}(1) + delta_cost;
                    if (cost < state_space{ii+1, mm, nn}(1))
                        state_space{ii+1, mm, nn} = [cost, jj, kk, s_achieved];
                    end
                end
            end
        end
    end
end
% 寻找最优的末端结点
cur_best = Inf;
for jj = 1 : NS
    for kk = 1 : NL
        if (state_space{NT, jj, kk}(1) < cur_best)
            cur_best = state_space{NT, jj, kk}(1);
            cur_best_s_ind = jj;
            cur_best_l_ind = kk;
        end
    end
end
ind_s = cur_best_s_ind; ind_l = cur_best_l_ind;
child_s = cur_best_s_ind; child_l = cur_best_l_ind;
for ii = (NT-1) : -1 : 1
    cur_ind = state_space{ii+1, child_s, child_l}(2:3);
    ind_s = [cur_ind(1), ind_s];
    ind_l = [cur_ind(2), ind_l];
    child_s = cur_ind(1); child_l = cur_ind(2);
end
% 结点回溯
global BV_ vehicle_geometrics_
s = BV_.s0; l = BV_.l0;
for ii = 1 : NT
    s = [s, dp_.ds(ind_s(ii)) + s(end)];
    [lb, rb] = ProvideRoadBound(s(end));
    lb = lb + vehicle_geometrics_.vehicle_width * 0.5;
    rb = rb - vehicle_geometrics_.vehicle_width * 0.5;
    l = [l, lb + (rb - lb) * dp_.dl(ind_l(ii))];
end
% 重采样并转化到笛卡尔坐标系
[s, l] = ResampleSL(s, l);
[x, y, theta] = ConvertSlToXYTheta(s, l);
end

function [s_full,l_full] = ResampleSL(s,l)
s_full = []; l_full = [];
global precise_timeline_index dp_
for ii = 1 : dp_.num_t_grids
    ind1 = precise_timeline_index{1,ii}.ind1;
    ind2 = precise_timeline_index{1,ii}.ind2;
    len = ind2 - ind1;
    if (ii == dp_.num_t_grids)
        len = len + 1;
    end
    s_full = [s_full, linspace(s(ii), s(ii+1), len)];
    l_full = [l_full, linspace(l(ii), l(ii+1), len)];
end
end

function [x, y, theta] = ConvertSlToXYTheta(s, l)
Nfe = length(s);
x = zeros(1,Nfe); y = zeros(1,Nfe); theta = zeros(1,Nfe);
for ii = 1 : Nfe
    [x(ii), y(ii), theta(ii)] = ConvertFrenetToCartesian(s(ii), l(ii));
end
end

function [val, s1] = EstimateCostZero(jj,kk)
global dp_ BV_ precise_timeline_index vehicle_geometrics_
s0 = BV_.s0;
l0 = BV_.l0;
s1 = s0 + dp_.ds(jj);
[lb, rb] = ProvideRoadBound(s1);
lb = lb + vehicle_geometrics_.vehicle_width * 0.5;
rb = rb - vehicle_geometrics_.vehicle_width * 0.5;
l1 = lb + (rb - lb) * dp_.dl(kk);

Nfe = precise_timeline_index{1,1}.ind2 - precise_timeline_index{1,1}.ind1 + 1;
s_list = linspace(s0, s1, Nfe);
l_list = linspace(l0, l1, Nfe);
val = dp_.w1 * EvaluateCollision(1, s_list, l_list) + ...
    dp_.w2 * (EvaluateLateralChange(s_list, l_list)) + ...
    dp_.w3 * EvaluateAchievement(s_list) + ...
    0.1 * mean(abs(l_list));
end

function [val, s1] = EstimateCost(cur_state_ind, next_layer_state_ind)
global state_space dp_ precise_timeline_index vehicle_geometrics_
s0 = state_space{cur_state_ind(1), cur_state_ind(2), cur_state_ind(3)}(4);
[lb, rb] = ProvideRoadBound(s0);
lb = lb + vehicle_geometrics_.vehicle_width * 0.5;
rb = rb - vehicle_geometrics_.vehicle_width * 0.5;
l0 = lb + (rb - lb) * dp_.dl(cur_state_ind(3));

s1 = s0 + dp_.ds(next_layer_state_ind(2));
[lb, rb] = ProvideRoadBound(s1);
lb = lb + vehicle_geometrics_.vehicle_width * 0.5;
rb = rb - vehicle_geometrics_.vehicle_width * 0.5;
l1 = lb + (rb - lb) * dp_.dl(next_layer_state_ind(3));

Nfe = precise_timeline_index{1,next_layer_state_ind(1)}.ind2 - precise_timeline_index{1,next_layer_state_ind(1)}.ind1 + 1;
s_list = linspace(s0, s1, Nfe);
l_list = linspace(l0, l1, Nfe);
val = dp_.w1 * EvaluateCollision(next_layer_state_ind(1), s_list, l_list) + ...
    dp_.w2 * (EvaluateLateralChange(s_list, l_list) + dp_.w_relative * EvaluateLongituteChange(cur_state_ind(2), next_layer_state_ind(2))) + ...
    dp_.w3 * EvaluateAchievement(s_list) + ...
    0.1 * mean(abs(l_list)); % 最后这一项也是书中没有的，用于反映决策轨迹尽量沿着指引线行驶
end

function val = EvaluateLongituteChange(ind1, ind2)
global dp_
val = abs(dp_.ds(ind1) - dp_.ds(ind2)) / dp_.ds(end);
end

function cost = EvaluateLateralChange(s_list, l_list)
ds = s_list(end) - s_list(1) + 0.00000001;
dl = abs(l_list(end) - l_list(1));
cost = dl / ds;
end

function val = EvaluateAchievement(s_list)
global dp_
val = 1 - (s_list(end) - s_list(1)) / dp_.max_unit_s;
end

function cost = EvaluateCollision(ind_time, s_list, l_list)
cost = 0;
global obstacles_ precise_timeline_index Nobs
ind1 = precise_timeline_index{1,ind_time}.ind1;
ind2 = precise_timeline_index{1,ind_time}.ind2;
for ii = 1 : Nobs
    s = obstacles_{1,ii}.s(ind1 : ind2);
    l = obstacles_{1,ii}.l(ind1 : ind2);
    cost = cost + MeasureCollision(s,l,s_list,l_list);
end
end

function val = MeasureCollision(s, l, s_ego, l_ego)
global vehicle_geometrics_ dp_
Nfe = length(s);
err_s = abs(s - s_ego);
ind = find(err_s <= vehicle_geometrics_.vehicle_length);
s = s(ind); l = l(ind); s_ego = s_ego(ind); l_ego = l_ego(ind);
err_l = abs(l - l_ego);
ind = find(err_l <= vehicle_geometrics_.vehicle_width);
if (length(ind) > 0)
    val = dp_.Ncollision + length(ind) / Nfe;
    % 注意，结尾加的这一项在书中没有体现，它的存在使得碰撞程度的刻画更具区分度
else
    val = 0;
end
end