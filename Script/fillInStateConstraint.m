function [G, JG] = fillInStateConstraint(G, JG, con_offset, ...
    state_offset, t, xd, state_frames, stateConstraint, nc, noDesign)
% fillInStateConstraint fill in the state constraint at a given offset   
% Fill in the state constraint for G(offset ...) and with time vector t
%
% Used in computeStateConstraints

nd = length(xd);
nxu = size(state_frames,1);
for i=1:length(t)
    xu = state_frames(:,i);
    g = stateConstraint.fun(xd, xu);
    g = reshape(g, numel(g),1);
    G(con_offset+(i-1)*nc+1:con_offset +i*nc) = g;
    
    %
    [Jd, Js] = stateConstraint.jacobian(xd,xu);
    if ~noDesign
        JG(con_offset+(i-1)*nc+1 : con_offset+i*nc, 1:nd) = Jd;
    end
    JG(con_offset+(i-1)*nc+1 : con_offset+i*nc, ...
        state_offset + (i-1)* nxu +1 : state_offset + i*nxu) = Js;
    

end

end
