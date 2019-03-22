--[[
control key
    from keyboard: up down left right
when vehicle velocity is as approximate to zero as requirement, vehicle brakes
]]


function sysCall_init() 
    steer_handle= sim.getObjectHandle('steer_joint')
    motor_handle= sim.getObjectHandle('motor_joint')
    fl_brake_handle= sim.getObjectHandle('fl_brake_joint')
    fr_brake_handle= sim.getObjectHandle('fr_brake_joint')
    bl_brake_handle= sim.getObjectHandle('bl_brake_joint')
    br_brake_handle= sim.getObjectHandle('br_brake_joint')
    
    --wheel radius:         0.09
    --wheel base:             0.6
    --wheel track:             0.35
    --maximum steering rate:     70 deg/sec
    
        --the maximum steer angle 30 degree
    max_steer_angle=0.5235987
        --the maximum torque of the motor
    motor_torque=60
    
    dVel=1
    dSteer=0.1
    
        --input steer
    steer_angle=0
        --input velocity
    motor_velocity=dVel*10
        --input brake
    brake_force=0
end

function sysCall_cleanup() 
 
end 

function sysCall_actuation() 
    --current steer pos
    steer_pos=sim.getJointPosition(steer_handle);
    --current angular velocity of back left wheel
    bl_wheel_velocity=sim.getObjectFloatParameter(bl_brake_handle,sim.jointfloatparam_velocity)
    --current angular velocity of back right wheel
    br_wheel_velocity=sim.getObjectFloatParameter(br_brake_handle,sim.jointfloatparam_velocity)
    --average angular velocity of the back wheels
    rear_wheel_velocity=(bl_wheel_velocity+br_wheel_velocity)/2
    --linear velocity
    linear_velocity=rear_wheel_velocity*0.09 
    
    
    -- Read the keyboard messages (make sure the focus is on the main window, scene view):
    message,auxiliaryData=sim.getSimulatorMessage()
    while message~=-1 do
        if (message==sim.message_keypress) then
            if (auxiliaryData[1]==2007) then
                -- up key
                if (motor_velocity<dVel*9.99) then
                    motor_velocity=motor_velocity+dVel
                end
            end
            if (auxiliaryData[1]==2008) then
                -- down key
                if (motor_velocity>-dVel*1.99) then
                    motor_velocity=motor_velocity-dVel
                else
                --    brake_force=100
                end
            end
            if (auxiliaryData[1]==2009) then
                -- left key
                if (steer_angle<dSteer*4.99) then
                    steer_angle=steer_angle+dSteer
                end
            end
            if (auxiliaryData[1]==2010) then
                -- right key
                if (steer_angle>-dSteer*4.99) then
                    steer_angle=steer_angle-dSteer
                end
            end
        end
        message,auxiliaryData=sim.getSimulatorMessage()
    end
    
    if (math.abs(motor_velocity)<dVel*0.1) then
        brake_force=100
    else
        brake_force=0
    end
    
        --set maximum steer angle
    if (steer_angle> max_steer_angle) then
    steer_angle=max_steer_angle
    end
    if (steer_angle< -max_steer_angle) then
    steer_angle= -max_steer_angle
    end
    sim.setJointTargetPosition(steer_handle, steer_angle)
    
        --brake and motor can not be applied at the same time
    if(brake_force>0) then
    sim.setJointForce(motor_handle, 0)
    else
    sim.setJointForce(motor_handle, motor_torque)
    sim.setJointTargetVelocity(motor_handle, motor_velocity)
    end
    
    sim.setJointForce(fr_brake_handle, brake_force)
    sim.setJointForce(fl_brake_handle, brake_force)
    sim.setJointForce(bl_brake_handle, brake_force)
    sim.setJointForce(br_brake_handle, brake_force)
end 
