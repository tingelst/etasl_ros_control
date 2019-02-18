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

maxvel = 0.5
for i = 1, #robot_joints do
    BoxConstraint{
        context = ctx,
        var_name = robot_joints[i],
        lower = -maxvel,
        upper = maxvel
    }
end

tgt_x = ctx:createInputChannelScalar("tgt_x", 0.7)
tgt_y = ctx:createInputChannelScalar("tgt_y", 0.2)
tgt_z = ctx:createInputChannelScalar("tgt_z", 0.7)

Constraint{
    context = ctx,
    name = "x",
    expr = tgt_x - coord_x(origin(robot_ee)),
    priority = 2,
    K = 4
}

Constraint{
    context = ctx,
    name = "y",
    expr = tgt_y - coord_y(origin(robot_ee)),
    priority = 2,
    K = 4
}

Constraint{
    context = ctx,
    name = "z",
    expr = tgt_z - coord_z(origin(robot_ee)),
    priority = 2,
    K = 4
}

ctx:setOutputExpression("error_x", coord_x(origin(robot_ee)) - tgt_x)
ctx:setOutputExpression("error_y", coord_y(origin(robot_ee)) - tgt_y)
ctx:setOutputExpression("error_z", coord_z(origin(robot_ee)) - tgt_z)
