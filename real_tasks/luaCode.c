-- Foraging implementation for CoppeliaSim
-- Jahir Argote
-- 10 Dec 2020
-- The University of Sheffield
--
-- Revised version for CoppeliaSim
-- Roderich Gross
-- 03 Nov 2020
-- The University of Sheffield
--
-- Revised version for CoppeliaSim
-- Roderich Gross
-- 12 Feb 2020
-- The University of Sheffield
--
-- Original version for V-REP
-- Anil Ozdemir
-- 07 Feb 2019
-- The University of Sheffield

-- This is the e-puck principal control script. It is threaded.
threadFunction=function()

    searchl = 2
    searchr = 3
    rot_spd = -1.87
    lmt = 0.04
    dir = 1
    mem = 1
    
    velhigh = 3
    vellow = 2
    velLeft = 0
    velRight = 0
    while sim.getSimulationState()~=sim.simulation_advancing_abouttostop do
            
        -- Part 3: Obtains proximity sensor reading values

        s=sim.getObjectSizeFactor(bodyElements) -- make sure that if we scale the robot during simulation, other values are scaled too!
        noDetectionDistance=0.05*s
        proxSensDist={noDetectionDistance,noDetectionDistance,noDetectionDistance,noDetectionDistance,noDetectionDistance,noDetectionDistance,noDetectionDistance,noDetectionDistance}
        --Loop through all the proximity sensors
        for i=1,8,1 do
            res,dist=sim.readProximitySensor(proxSens[i])
            if (res>0) and (dist<noDetectionDistance) then
                proxSensDist[i]=dist
            end
        end

        -- Part 4: Determines the velocity values for the two wheel motors.
        
    --Functions that controls the different behaviors of the system.
    
        --Move around to explore
        function search()
            velLeft = searchl
            velRight = searchr
            sim.addStatusbarMessage('vl: '..velLeft)
        end
        
        --Rotate to free front part from collision.
        function rotate()
            velLeft = rot_spd*dir
            velRight = -rot_spd*dir
            sim.addStatusbarMessage('th: '..dir)
            mem = 0
        end
        
        function rot_dir()
            if (proxSensDist[5]<lmt and mem == 1) then
                dir = 1
            end
            if(proxSensDist[2]<lmt and mem == 1) then
                dir = -1
            end
        end
        
        function controller()
            error_x = proxSensDist[6]-proxSensDist[1] + math.cos(45*math.pi/180)*(proxSensDist[5]-proxSensDist[2]) + math.cos(60*math.pi/180)*(proxSensDist[7]-proxSensDist[8])
            error_y = math.sin(45*math.pi/180)*(proxSensDist[5]-proxSensDist[2]) + math.sin(60*math.pi/180)*(proxSensDist[7]-proxSensDist[8])
            
            --thetha = math.atan2(error_y,error_x)
            --sim.addStatusbarMessage('th: '..theta)
            
            velLeft = vellow
            velRight = vellow
            mem = 1
            if(error_x < 0) then
                velLeft = vellow
                velRight = velhigh
            end
            
            if(error_x > 0) then
                velLeft = velhigh
                velRight = vellow
            end
            
        end
            
            
        -- Event handler for frontal collision and right.
        if (proxSensDist[3]<lmt or proxSensDist[4]<lmt) then
            rot_dir()
            rotate()
        else
            --more_explore()
            controller()
        end
        
        -- Part 5 - DO NOT CHANGE: Apply noise to motor speeds.
        velLeft  = velLeft  + (math.random()-0.5)*0.312 -- 5% uniform noise
        velRight = velRight + (math.random()-0.5)*0.312 -- 5% uniform noise

        -- Part 6 - DO NOT CHANGE: Probes whether motor speeds are within allowed range.
        maxVel = 6.24 -- maximum wheel speeds in rad/s. DO NOT CHANGE.
        if (velLeft < -maxVel) then
            velLeft = -maxVel
        elseif (velLeft > maxVel) then
            velLeft = maxVel
        end
        if (velRight < -maxVel) then
            velRight = -maxVel
        elseif (velRight > maxVel) then
            velRight = maxVel
        end
       
        -- Part 7 - DO NOT CHANGE: Applies the leftMotor and rightMotor variables to the wheels.
        sim.setJointTargetVelocity(leftMotor,velLeft)
        sim.setJointTargetVelocity(rightMotor,velRight)
        
        sim.switchThread() -- don't waste too much time in here (simulation time will anyway only change in next thread switch)
    end
end

-- these are handles, you do not need to change here. (If you need e.g. bluetooth, you can add it here)
sim.setThreadSwitchTiming(200) -- we will manually switch in the main loop
bodyElements=sim.getObjectHandle('ePuck_bodyElements')
leftMotor=sim.getObjectHandle('ePuck_leftJoint')
rightMotor=sim.getObjectHandle('ePuck_rightJoint')
ePuck=sim.getObjectHandle('ePuck')
ePuckCam=sim.getObjectHandle('ePuck_camera')
ePuckBase=sim.getObjectHandle('ePuck_base')
ledLight=sim.getObjectHandle('ePuck_ledLight')

proxSens={-1,-1,-1,-1,-1,-1,-1,-1}
for i=1,8,1 do
    proxSens[i]=sim.getObjectHandle('ePuck_proxSensor'..i)
end


res,err=xpcall(threadFunction,function(err) return debug.traceback(err) end)
if not res then
    sim.addStatusbarMessage('Lua runtime error: '..err)
end
