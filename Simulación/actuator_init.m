function actuator_init

% Initialize TrueTime kernel
ttInitKernel('prioDM');   % deadline-monotonic scheduling

data.uk = 0;
data.tk = 0;
data.h = 0.012;


% Periodic actuator task
starttime = 0.0;

ttCreatePeriodicTask('actuator_task', starttime, data.h, 'actuator_code', data);

% Sporadic actuator task, activated by arriving network message
%deadline = 1e-3;
%ttCreateTask('actuatorReceiveMessage_task', deadline, 'actuatorReceiveMessage_code', data);
%ttAttachNetworkHandler('actuatorReceiveMessage_task')

