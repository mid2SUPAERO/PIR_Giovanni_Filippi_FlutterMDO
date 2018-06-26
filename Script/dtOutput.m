function stop = dtOutput(obj, n, x, optimValues, state)
% dtOutput saves the intermediate result of direct transcription

% This function is used for running long optimizations. The data file
% can be examined in a different MATLAB session. 

stop = false;
filename = 'dt_intermediate_result';
platform = computer;

switch state
    case 'init'
        % nothing
    case 'iter'
        if rem(optimValues.iteration, n) == 0
            xd = x(1:obj.n_design);
            offset = 0;
            xu = {};
            for i=1: length(obj.t)
                nt = length(obj.t{i});
                total_length = (obj.n_state+obj.n_control )* nt;
                xu{i} = reshape(x(obj.n_design + offset +1: ...
                    obj.n_design+offset+total_length), ...
                    (obj.n_state+obj.n_control), nt);
                offset = offset + total_length;
            end
            
            save([filename '_' platform ...
                '_'  mat2str(floor(optimValues.iteration/n))]);
        end
    case 'done'
        % nothing
    otherwise
end


end