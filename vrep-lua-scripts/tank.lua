motion_param={0,0,0,0,0}

function sysCall_init() 
    left_front_handle= sim.getObjectHandle('left_front')
    left_back_handle= sim.getObjectHandle('left_back')
    right_back_handle= sim.getObjectHandle('right_back')
    right_front_handle= sim.getObjectHandle('right_front')
    
    sj_l_1_handle= sim.getObjectHandle('sj_l_1')
    sj_l_2_handle= sim.getObjectHandle('sj_l_2')
    sj_l_3_handle= sim.getObjectHandle('sj_l_3')
    sj_l_4_handle= sim.getObjectHandle('sj_l_4')
    sj_l_5_handle= sim.getObjectHandle('sj_l_5')
    sj_l_6_handle= sim.getObjectHandle('sj_l_6')
    
    sj_r_1_handle= sim.getObjectHandle('sj_r_1')
    sj_r_2_handle= sim.getObjectHandle('sj_r_2')
    sj_r_3_handle= sim.getObjectHandle('sj_r_3')
    sj_r_4_handle= sim.getObjectHandle('sj_r_4')
    sj_r_5_handle= sim.getObjectHandle('sj_r_5')
    sj_r_6_handle= sim.getObjectHandle('sj_r_6')
    MaxVel=10
    leftvelocity=0
    rightvelocity=0
    dVel=0.5;
    --sim.setJointTargetVelocity(left_front_handle,leftvelocity)
    sim.setJointTargetVelocity(left_back_handle,leftvelocity)
    sim.setJointTargetVelocity(right_back_handle,rightvelocity)
    --sim.setJointTargetVelocity(right_front_handle,rightvelocity)

--added
                --sim.setJointForce(left_front_handle, 10000)
                sim.setJointForce(left_back_handle, 10000)
                sim.setJointForce(right_back_handle, 10000)
                --sim.setJointForce(right_front_handle, 10000)
    
                sim.setJointForce(sj_r_1_handle, 0)
                sim.setJointForce(sj_r_2_handle, 0)
                sim.setJointForce(sj_r_3_handle, 0)
                sim.setJointForce(sj_r_4_handle, 0)
                sim.setJointForce(sj_r_5_handle, 0)
                sim.setJointForce(sj_r_6_handle, 0)
    
                sim.setJointForce(sj_l_1_handle, 0)
                sim.setJointForce(sj_l_2_handle, 0)
                sim.setJointForce(sj_l_3_handle, 0)
                sim.setJointForce(sj_l_4_handle, 0)
                sim.setJointForce(sj_l_5_handle, 0)
                sim.setJointForce(sj_l_6_handle, 0)
end

function sysCall_actuation()
            if (motion_param[2]==1) then
                -- up key
                motion_param[2]=0
                leftvelocity=(leftvelocity+rightvelocity)/2
                rightvelocity=leftvelocity
                leftvelocity=leftvelocity+dVel
                rightvelocity=rightvelocity+dVel
            end
            if (motion_param[3]==1) then
                -- down key
                motion_param[3]=0
                leftvelocity=(leftvelocity+rightvelocity)/2
                rightvelocity=leftvelocity
                leftvelocity=leftvelocity-dVel
                rightvelocity=rightvelocity-dVel
            end
            if (motion_param[4]==1) then
                -- left key
                motion_param[4]=0
                leftvelocity=leftvelocity-dVel
                rightvelocity=rightvelocity+dVel
            end
            if (motion_param[5]==1) then
                -- right key
                motion_param[5]=0
                leftvelocity=leftvelocity+dVel
                rightvelocity=rightvelocity-dVel
            end   

    if leftvelocity>MaxVel then
        leftvelocity=MaxVel
    end
    if leftvelocity<-MaxVel then
        leftvelocity=-MaxVel
    end
    
    if rightvelocity>MaxVel then
                rightvelocity=MaxVel
    end
    if rightvelocity<-MaxVel then
                rightvelocity=-MaxVel
    end
    
    --sim.setJointTargetVelocity(left_front_handle,leftvelocity)
    sim.setJointTargetVelocity(left_back_handle,leftvelocity)
    sim.setJointTargetVelocity(right_back_handle,rightvelocity)
print('left',leftvelocity,'    right',rightvelocity)
    --sim.setJointTargetVelocity(right_front_handle,rightvelocity)
    
end 

function set_Tank_Param(inInts)
    motion_param=inInts
    return {},{},{},''
end
