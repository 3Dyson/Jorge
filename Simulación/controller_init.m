function controller_init(arg)

global A B C D 
% Initialize TrueTime kernel
ttInitKernel('prioDM') % deadline-monotonic scheduling

% data.K1 = K(1);
% data.K2 = K(2);
data.A = A;
data.B = B;
data.C = C;
data.D = D;

data.uk = 0;

% Sporadic controller task, activated by arriving network message
deadline = 1e-3;
ttCreateTask('controller_task', deadline, 'controller_code', data);
ttAttachNetworkHandler('controller_task')
