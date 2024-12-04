sim=require'sim'
simROS=require'simROS'

function sysCall_init()
    youBot=sim.getObject('.')
    pub=simROS.advertise('/youbot_tf', 'std_msgs/Float32MultiArray')
    
    corout=coroutine.create(coroutineMain)
    youbotRef = sim.getObject('./youBot_ref')
    youbotTarget = sim.getObject('./bot_target')
    sim.setObjectPosition(youbotRef, sim.handle_parent, {0,0,0})
    sim.setObjectPosition(youbotTarget, sim.handle_parent, {0,0,0})
    sim.setObjectParent(youbotTarget, -1, true)
    
    --Prepare initial values and retrieve handles:
    wheelJoints={-1,-1,-1,-1} -- front left, rear left, rear right, front right
    wheelJoints[1]=sim.getObject('./rollingJoint_fl')
    wheelJoints[2]=sim.getObject('./rollingJoint_rl')
    wheelJoints[3]=sim.getObject('./rollingJoint_rr')
    wheelJoints[4]=sim.getObject('./rollingJoint_fr')
    
    prevForwBackVel =0
    prevLeftRightVel = 0
    prevRotVel = 0 
end

function sysCall_actuation()

    local pos=sim.getObjectPosition(youBot)
    local rot=sim.getObjectOrientation(youBot)
    local x, y, z, w=toQ(0, 0, rot[2])
    local message = {data={pos[1], pos[2], 0, x, y, z, w}}
    simROS.publish(pub,message)

    if coroutine.status(corout)~='dead' then
        local ok,errorMsg=coroutine.resume(corout)
        if errorMsg then
            error(debug.traceback(corout,errorMsg),2)
        end
    end
    
    relP = sim.getObjectPosition(youbotTarget, youbotRef)
    relE = sim.getObjectOrientation(youbotTarget, youbotRef)
    
    posParam = 20
    rotParam = 10
    accF = 0.035    
    maxVel = 2
    maxRotVel = 3
    
    ForwBackVel = relP[2] * posParam
    LeftRightVel = relP[1] * posParam
    
    rotVel = - relE[3] * rotParam
        
    vel = math.sqrt(ForwBackVel*ForwBackVel + LeftRightVel*LeftRightVel)
    
    if vel > maxVel then
        ForwBackVel = ForwBackVel * maxVel / vel
        LeftRightVel = LeftRightVel * maxVel / vel
    end  
        
    if math.abs(rotVel) > maxRotVel then
        rotVel = maxRotVel * rotVel / math.abs(rotVel)
    end
    
    df = ForwBackVel - prevForwBackVel
    ds = LeftRightVel - prevLeftRightVel
    dr = rotVel - prevRotVel
    
    if math.abs(df) > (maxVel * accF) then
        df = math.abs(df) * (maxVel * accF) / df
        ForwBackVel = prevForwBackVel + df
    end
    
    if math.abs(ds) > (maxVel * accF) then
        ds = math.abs(ds) * (maxVel * accF) / ds
        LeftRightVel = prevLeftRightVel + ds
    end
    
    if math.abs(dr) > (maxVel * accF) then
        df = math.abs(dr) * (maxVel * accF) / dr
        rotVel = prevRotVel + dr
    end
    
    
    setMovement(ForwBackVel, LeftRightVel, rotVel)
    
    prevForwBackVel = ForwBackVel
    prevLeftRightVel = LeftRightVel
    prevRotVel = rotVel
    
    --print(sim.getObjectOrientation(youbotTarget, youbotRef))
    
end

reachYoubotTargetPosition = function ()
    repeat
        sim.switchThread()
        p1 = sim.getObjectPosition(youbotTarget, -1)
        p2 = sim.getObjectPosition(youbotRef, -1)
        p = {p2[1] - p1[1], p2[2] - p1[2]}
        pErr = math.sqrt(p[1]*p[1] + p[2]*p[2])
        oErr = math.abs(sim.getObjectOrientation(youbotRef, youbotTarget)[3])
    until (pErr < 0.001) and (oErr < 0.1 * math.pi/180)
end


function setMovement(forwBackVel,leftRightVel,rotVel)
    -- Apply the desired wheel velocities:
    sim.setJointTargetVelocity(wheelJoints[1],-forwBackVel-leftRightVel-rotVel)
    sim.setJointTargetVelocity(wheelJoints[2],-forwBackVel+leftRightVel-rotVel)
    sim.setJointTargetVelocity(wheelJoints[3],-forwBackVel-leftRightVel+rotVel)
    sim.setJointTargetVelocity(wheelJoints[4],-forwBackVel+leftRightVel+rotVel)
end

function coroutineMain()
    
    reachYoubotTargetPosition()
    
    --print(sim.getObjectOrientation(youbotTarget, youbotRef))
    
    --sensor = sim.getObject('./Proximity_sensor')
    
    
    --proxSens = {}
    --temp = sim.handleProximitySensor(sensor)
    --proxSens = sim.readProximitySensor(sensor)
    
    
    --det_dist = proxSens.distance
    --det_res = proxSens.result
    
    --if det_res > 0 then
    --    print(det_dist)
    --end
    --for i=0,4,1 do
      --  armJoints[i+1]=sim.getObject('./youBotArmJoint'..i)
    --end

    
end

function sysCall_cleanup()
    simROS.shutdownPublisher(pub)
end


function toQ(roll, pitch, yaw)
    cr = math.cos(roll * 0.5);
    sr = math.sin(roll * 0.5);
    cp = math.cos(pitch * 0.5);
    sp = math.sin(pitch * 0.5);
    cy = math.cos(yaw * 0.5);
    sy = math.sin(yaw * 0.5);
    
    w = cr * cp * cy + sr * sp * sy;
    x = sr * cp * cy - cr * sp * sy;
    y = cr * sp * cy + sr * cp * sy;
    z = cr * cp * sy - sr * sp * cy;
    return x, y, z, w
end