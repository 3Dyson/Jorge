function [exectime, data] = controller_code(seg, data)

switch seg
    case 1
        rcvData = ttGetMsg;
        if isempty(rcvData)
            disp('Error in controller: no message received!');
        end
        
        x1 = rcvData(1)
        x2 = rcvData(2)
        tauk = rcvData(3)
        r = ttAnalogIn(1)    % Read reference value        sys = ss(data.A,data.B,data.C,data.D,tauk);
        
        sys = ss(data.A, data.B, data.C, data.D);
        sys = c2d(sys, tauk)
        
        x_hat = sys.a*[x1; x2] + sys.b*data.uk
        
        Q = [2 0; 0 0.5];
        R = 0.01;
        [K,S,e] = lqrd(data.A,data.B,Q,R,tauk) 
        
        data.uk = -K(1)*(x_hat(1)-r) - K(2)*x_hat(2)
        
        exectime = 1e-6;
        
    case 2
        ttSendMsg(1, data.uk, 80);    % Send 80 bits to node 4 (actuator)
        exectime = -1; % finished
end