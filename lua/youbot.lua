function sysCall_init()
    -- Initialization of libraries
    sim = require 'sim'
    simROS2 = require 'simROS2'

    -- Set ROS2 topics by id
    id = '0001'
    robot_pose_topic = '/robot_' .. id .. '/robot_pose'
    arm_joint_states_topic = '/robot_' .. id .. '/arm_joint_states'
    arm_joint_commands_topic = '/robot_' .. id .. '/arm_joint_commands'
    wheel_velocities_topic = '/robot_' .. id .. '/wheel_velocities'

    -- Get handles of wheel joints
    wheelJoints = {
        sim.getObject('./../rollingJoint_fl'),
        sim.getObject('./../rollingJoint_rl'),
        sim.getObject('./../rollingJoint_rr'),
        sim.getObject('./../rollingJoint_fr')
    }

    -- Get handles of arm joints
    armJoints = {
        sim.getObject('./../arm_joint1'),
        sim.getObject('./../arm_joint2'),
        sim.getObject('./../arm_joint3'),
        sim.getObject('./../arm_joint4'),
        sim.getObject('./../arm_joint5')
    }

    -- Get handle of robot base
    robotBase = sim.getObject('./../youBot_ref')
    
    -- Get handle of robot base
    TCP = sim.getObject('./../arm_TCP')

    -- Create publishers and subscribers    
    wheelVelSub = simROS2.createSubscription(wheel_velocities_topic, 'sensor_msgs/msg/JointState', 'wheelVelCallback')
    jointCmdSub = simROS2.createSubscription(arm_joint_commands_topic, 'sensor_msgs/msg/JointState', 'jointCmdCallback')
    jointStatePub = simROS2.createPublisher(arm_joint_states_topic, 'sensor_msgs/msg/JointState')
    posePub = simROS2.createPublisher(robot_pose_topic, 'geometry_msgs/msg/Pose')
end

-- Handler for wheel velocities (Using JointState message)
function wheelVelCallback(msg)
    -- msg.position and msg.velocity contain arrays of joint data
    for i = 1, #wheelJoints do
        -- Use the position or velocity to control the wheels
        if msg.velocity[i] then
            sim.setJointTargetVelocity(wheelJoints[i], msg.velocity[i])
        end
    end
end

-- Handler for joint commands (Using JointState message)
function jointCmdCallback(msg)
    -- msg.position, msg.velocity, and msg.effort contain arrays of joint data
    for i = 1, #armJoints do
        -- Use position or velocity to control the manipulator
        if msg.position[i] then
            sim.setJointTargetPosition(armJoints[i], msg.position[i])
        end
        -- If you want to use velocities for control
        -- if msg.velocity[i] then
        --     sim.setJointTargetVelocity(armJoints[i], msg.velocity[i])
        -- end
    end
end

function sysCall_sensing()
    -- Publish robot pose
    local position = sim.getObjectPosition(robotBase, -1)
    local orientation = sim.getObjectQuaternion(robotBase, -1)

    local poseMsg = {
        position = {x = position[1], y = position[2], z = position[3]},
        orientation = {x = orientation[1], y = orientation[2], z = orientation[3], w = orientation[4]}
    }
    simROS2.publish(posePub, poseMsg)

    -- Publish joint states
    local jointStateMsg = {
        name = {},
        position = {},
        velocity = {}
    }

    for i = 1, #armJoints do
        table.insert(jointStateMsg.name, 'joint' .. i)

        local jointPosition = sim.getJointPosition(armJoints[i])
        table.insert(jointStateMsg.position, jointPosition)

        local jointVelocity = sim.getObjectFloatParameter(armJoints[i], sim.jointfloatparam_velocity)
        table.insert(jointStateMsg.velocity, jointVelocity)
    end

    simROS2.publish(jointStatePub, jointStateMsg)
end