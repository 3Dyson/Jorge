function [exectime, data] = actuator_code(seg, data)

switch seg
    case 1      
        uk = ttGetMsg;
        if ~isempty(uk)
            data.uk = uk;
        end

        ttAnalogOut(1, data.uk)
        exectime = 1e-6;
        
    case 2
        data.tk = data.tk + data.h
        ttSendMsg(2, data.tk, 80); % Send message (80 bits) to node 2 (sensor)
        exectime = 1e-6;
    case 3
        exectime = -1; % finished
end