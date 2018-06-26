function [G_s, JG_s] = computeStateConstraints(obj, x, x_offset, xd)
% computeStateConstraints computes the state constraints and Jacobian
% matrix. 
%
% The additional x_offset is an offset into the given x vector indicates
% the starting point of state variables. For sequential solvers it is
% always 0, for DT solver it is the number of design variables. 
%
% Add state constraints

if ~isempty(obj.stateConstraint)
    nt = calculateTotalSize(obj);
    
    % Determine size of the state constraint
    x0 = getFirstStateFrame(x, obj, x_offset);
    nc = zeros(length(obj.t), 1);
    tlen = zeros(length(obj.t), 1);
    for i=1:length(obj.t)
        nc(i) = length(obj.stateConstraint(i).fun(xd, x0));
        tlen(i) = length(obj.t{i});
    end
    
    total_c = sum(nc .* tlen);
    G_s = zeros(total_c, 1);  % Constraint for all time points
    JG_s = sparse(total_c, x_offset + nt* (obj.n_state + obj.n_control)); 
    
    n_offset = 0;  % Offset of constraint into G_s
    state_offset = x_offset; % Offset of state variable into x
    if x_offset ==0
        noDesign = true;
    else
        noDesign = false;
    end
    for i=1:length(obj.t)
        t = obj.t{i};
        nt = length(t);
        total_length = nt * (obj.n_state + obj.n_control);
        
        x_state = x(state_offset+1: state_offset+total_length);
        x_state_frames = reshape(x_state, obj.n_state + obj.n_control, nt);
        
        [G_s, JG_s] = fillInStateConstraint(G_s, JG_s, n_offset, ...
            state_offset, t, xd, x_state_frames, obj.stateConstraint(i), ...
            nc(i), noDesign);
        
        % Move offsets
        n_offset = n_offset + nt * nc(i);
        state_offset = state_offset + total_length;
    end
else
    G_s = [];
    JG_s = [];
end
end

function nt = calculateTotalSize(obj)
% calculate total size
nt = sum(cellfun(@length, obj.t));
end

function x0 = getFirstStateFrame(x, obj, offset)
% Extract the first frame of state in a given state frame trajectory
    frame_length = (obj.n_state + obj.n_control);
    x0 = x(offset+1:offset+frame_length);
end
