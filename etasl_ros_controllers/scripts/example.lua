-- Loading the KUKA KR6 R900 sixx (Agilus) robot with a standard script:
require("context")
require("geometric")

-- loading a model for the KUKA KR6 R900 Sixx (Agilus)
local u = UrdfExpr()
u:readFromFile(rospack_find("etasl_ros_controllers") .. "/scripts/kuka_kr6r900sixx.urdf")
u:addTransform("ee", "tool0", "base_link")

local r = u:getExpressions(ctx)

-- The transformation of the robot mounting plate frame with respect to the robot base frame
robot_ee = r.ee
-- The name of the robot joints
robot_joints = {
    "joint_a1",
    "joint_a2",
    "joint_a3",
    "joint_a4",
    "joint_a5",
    "joint_a6"
}

j1 = ctx:getScalarExpr(robot_joints[1])
j2 = ctx:getScalarExpr(robot_joints[2])
j3 = ctx:getScalarExpr(robot_joints[3])
j4 = ctx:getScalarExpr(robot_joints[4])
j5 = ctx:getScalarExpr(robot_joints[5])
j6 = ctx:getScalarExpr(robot_joints[6])

maxvel = 0.3
for i = 1, #robot_joints do
    BoxConstraint{
        context = ctx,
        var_name = robot_joints[i],
        lower = -maxvel,
        upper = maxvel
    }
end

deg2rad = math.pi / 180.0
zeroval = constant(0.0)
ninetydegrees = constant(90 * deg2rad)

Constraint{
    context = ctx,
    name = "joint_trajectory1",
    expr = j1,
    target = constant(0)
}

Constraint{
    context = ctx,
    name = "joint_trajectory2",
    expr = j2,
    target = constant(-150 * deg2rad)
}

Constraint{
    context = ctx,
    name = "joint_trajectory3",
    expr = j3,
    target = constant(150 * deg2rad)
}

Constraint{
    context = ctx,
    name = "joint_trajectory4",
    expr = j4,
    target = constant(0)
}

Constraint{
    context = ctx,
    name = "joint_trajectory5",
    expr = j5,
    target = constant(0)
}

Constraint{
    context = ctx,
    name = "joint_trajectory6",
    expr = j6,
    target = constant(0)
}

-- Monitor{
--     context = ctx,
--     name = 'finish_after_some_time',
--     upper = 10,
--     actionname = 'exit',
--     expr = time
-- }

-- ctx:setOutputExpression("j2", j2)
-- ctx:setOutputExpression("trajectory", trajectory)
