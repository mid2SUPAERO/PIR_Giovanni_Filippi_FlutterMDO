% This is a simple script to run demo_wing_dt and demo_wing_sequential save
% the results and log
%% 
% 
% Author: Giovanni Filippi
% Date: 29/06/18

platform = computer;
diary_name = ['dt_results_' platform '.txt'];
if ~isempty(dir(diary_name))
    delete(diary_name);
end 
diary(diary_name);
demo_wing_dt;
save(['dt_results_' platform]);
diary('off');
% 
%% 
%
platform = computer;
diary_name = ['seq_results_' platform '.txt'];
if ~isempty(dir(diary_name))
    delete(diary_name);
end 
diary(diary_name);
demo_wing_sequential;
save(['seq_results_' platform]);
diary('off');
%
