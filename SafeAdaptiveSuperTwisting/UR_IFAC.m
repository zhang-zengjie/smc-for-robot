disp('Program started');
% vrep=remApi('remoteApi','extApi.h'); % using the header (requires a compiler)
vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19997,true,true,5000,5);

load('ifac_data.mat');

if (clientID>-1)
    disp('Connected to remote API server');

    % Now try to retrieve data in a blocking fashion (i.e. a service call):
    %[res,objs]=vrep.simxGetObjects(clientID,vrep.sim_handle_joint_type,vrep.simx_opmode_blocking);
% 		if (res==vrep.simx_return_ok)
% 			fprintf('Number of objects in the scene: %d\n',length(objs));
% 		else
% 			fprintf('Remote API function call returned with error code: %d\n',res);
%         end

    Jnum = 7;
    objs = zeros(1,Jnum);

    [rec,objs(1)] = vrep.simxGetObjectHandle(clientID,'UR5_joint1',vrep.simx_opmode_blocking);
    [rec,objs(2)] = vrep.simxGetObjectHandle(clientID,'UR5_joint2',vrep.simx_opmode_blocking);
    [rec,objs(3)] = vrep.simxGetObjectHandle(clientID,'UR5_joint3',vrep.simx_opmode_blocking);
    [rec,objs(4)] = vrep.simxGetObjectHandle(clientID,'UR5_joint4',vrep.simx_opmode_blocking);
    [rec,objs(5)] = vrep.simxGetObjectHandle(clientID,'UR5_joint5',vrep.simx_opmode_blocking);
    [rec,objs(6)] = vrep.simxGetObjectHandle(clientID,'UR5_joint6',vrep.simx_opmode_blocking);
    [rec,objs(7)] = vrep.simxGetObjectHandle(clientID,'UR5_joint7',vrep.simx_opmode_blocking);
    
    Jnt = [2,3,5];
    
    for i = 1:length(qr)
        for k = 1:length(Jnt)
            [rtn] = vrep.simxSetJointTargetPosition(clientID, objs(Jnt(k)), qr(i,k)/2, vrep.simx_opmode_oneshot);
        end
    end
    
    
    
    
%         for k = 1:Jnum
%             vrep.simxSetJointTargetPosition(clientID,objs(k),pi/2,vrep.simx_opmode_oneshot);
%         end
%         
% 		pause(2);

    % Now retrieve streaming data (i.e. in a non-blocking fashion):
    t=clock;
    startTime=t(6);
    currentTime=t(6);
% 		vrep.simxGetIntegerParameter(clientID,vrep.sim_intparam_mouse_x,vrep.simx_opmode_streaming); % Initialize streaming
    while (currentTime-startTime < 15)	
% 			[returnCode,data]=vrep.simxGetIntegerParameter(clientID,vrep.sim_intparam_mouse_x,vrep.simx_opmode_buffer); % Try to retrieve the streamed data
% 			if (returnCode==vrep.simx_return_ok) % After initialization of streaming, it will take a few ms before the first value arrives, so check the return code
% 				fprintf('Mouse position x: %d\n',data); % Mouse position x is actualized when the cursor is over V-REP's window
% 			end
        for k = 1:Jnum
           % [rec,pos] = vrep.simxGetJointPosition(clientID,objs(k),vrep.simx_opmode_blocking);
           [rec,pos] = vrep.simxGetJointForce(clientID,objs(k),vrep.simx_opmode_blocking);
                if (rec==vrep.simx_return_ok)
                    fprintf('The position of Joint %d is %f\n',k,pos);
                else
                    fprintf('Position obtaining error\n');
                end
        end
        t=clock;
        currentTime=t(6);
    end

    % Now send some data to V-REP in a non-blocking fashion:
    vrep.simxAddStatusbarMessage(clientID,'Hello V-REP!',vrep.simx_opmode_oneshot);

    % Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
    vrep.simxGetPingTime(clientID);

    % Now close the connection to V-REP:	
    vrep.simxFinish(clientID);
else
    disp('Failed connecting to remote API server');
end
vrep.delete(); % call the destructor!

disp('Program ended');