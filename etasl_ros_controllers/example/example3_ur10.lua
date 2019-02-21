require("context")
require("geometric")

local u = UrdfExpr()
u:readFromParam("/robot_description")
u:addTransform("ee", "ee_link", "base_link")

local r = u:getExpressions(ctx)

-- The transformation of the robot mounting plate frame with respect to the robot base frame
robot_ee = r.ee
-- The name of the robot joints
robot_joints = {
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint"
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

tgt_x = ctx:createInputChannelScalar("tgt_x", 0.5)
tgt_y = ctx:createInputChannelScalar("tgt_y", 0)
tgt_z = ctx:createInputChannelScalar("tgt_z", 0)

d = Variable{context = ctx, name = "d", vartype = "feature"}

Constraint{
    context = ctx,
    name = "laserdistance",
    expr = d,
    target_lower = 0.5,
    target_upper = 0.7,
    K = 4
}

laserspot = robot_ee * vector(0, 0, d)

Constraint{
    context = ctx,
    name = "x",
    expr = tgt_x - coord_x(laserspot),
    priority = 2,
    K = 4
}
Constraint{
    context = ctx,
    name = "y",
    expr = tgt_y - coord_y(laserspot),
    priority = 2,
    K = 4
}
Constraint{
    context = ctx,
    name = "z",
    expr = tgt_z - coord_z(laserspot),
    priority = 2,
    K = 4
}

ctx:setOutputExpression("error_x", coord_x(laserspot) - tgt_x)
ctx:setOutputExpression("error_y", coord_y(laserspot) - tgt_y)
ctx:setOutputExpression("error_z", coord_z(laserspot) - tgt_z)
