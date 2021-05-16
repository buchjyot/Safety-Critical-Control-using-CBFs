function [value,isterminal,direction] = hOriginCaptureEvent(t,x) %#ok<INUSL>
%% hOriginCaptureEvent Function
% This function defines capture event to stop the simulation

% We want to bring the states to origin
FinalPosition = 0;
FinalVelocity = 0;
value(1)      = (x(1)-FinalPosition)^2 + (x(2)-FinalVelocity)^2 - sqrt(eps);
isterminal(1) = 1;   % we want to end integration when collision occurs
direction(1)  = 0;   % The zero should only ever be approached from above
end