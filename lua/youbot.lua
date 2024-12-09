function sysCall_init()
    -- Initialization of libraries
    sim = require 'sim'
    simROS2 = require 'simROS2'

    -- Set ROS2 topics by id
    id = 0001
    robot_pose_topic = '/robot_' .. id .. '/robot_pose'
    arm_joint_states_topic = '/robot_' .. id .. '/arm_joint_states'
    arm_joint_commands_topic = '/robot_' .. id .. '/arm_joint_commands'
    wheel_velocities_topic = '/robot_' .. id .. '/wheel_velocities'

    -- Get handles of wheel joints
    wheelJoints = {
        sim.getObject('./rollingJoint_fl'),
        sim.getObject('./rollingJoint_rl'),
        sim.getObject('./rollingJoint_rr'),
        sim.getObject('./rollingJoint_fr')
    }

    -- Get handles of arm joints
    armJoints = {
        sim.getObject('./ang_joint1'),
        sim.getObject('./ang_joint2'),
        sim.getObject('./ang_joint3'),
        sim.getObject('./ang_joint4'),
        sim.getObject('./ang_joint5')
    }

    -- Get handle of robot base
    robotBase = sim.getObject('.')

    -- Create publishers and subscribers    
    wheelVelSub = simROS2.createSubscription(wheel_velocities_topic, 'std_msgs/Float64MultiArray', 'wheelVelCallback')
    jointCmdSub = simROS2.createSubscription(joint_commands_topic, 'std_msgs/Float64MultiArray', 'jointCmdCallback')
    jointStatePub = simROS2.createPublisher(joint_states_topic, 'sensor_msgs/JointState')
    posePub = simROS2.createPublisher(robot_pose_topic, 'geometry_msgs/Pose')
end

-- Handler for wheel velocities
function wheelVelCallback(msg)
    for i = 1, #wheelJoints do
        sim.setJointTargetVelocity(wheelJoints[i], msg.data[i])
    end
end

-- Handler for joint commands
function jointCmdCallback(msg)
    for i = 1, #armJoints do
        sim.setJointTargetVelocity(armJoints[i], msg.data[i])
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