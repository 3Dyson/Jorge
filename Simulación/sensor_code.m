function [exectime, data] = sensor_code(seg, data)

persistent tk timeStamp xk tauk

switch seg
    case 1
        tk = ttGetMsg;         % Obtain time            
        if isempty(tk)
            disp('Error in sensor: no message received!');
        end
        exectime = 1e-6;
        
    case 2
        xk(1) = ttAnalogIn(1);
        xk(2) = ttAnalogIn(2);
        timeStamp = ttCurrentTime;
        tauk = tk - timeStamp;
        exectime = 1e-6;
        
    case 3

        datt(1) = xk(1);
        datt(2) = xk(2);
        datt(3) = tauk;     
        
        ttSendMsg(3, datt, 80); % Send message (80 bits) to node 3 (controller)
        exectime = -1; % finished
end