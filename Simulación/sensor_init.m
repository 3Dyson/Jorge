function sensor_init
% Initialize TrueTime kernel

ttInitKernel('prioDM') % deadline-monotonic scheduling

deadline = 1e-3;
% Sporadic controller task, activated by arriving network message
ttCreateTask('sensor_task', deadline, 'sensor_code');%, data);
ttAttachNetworkHandler('sensor_task')