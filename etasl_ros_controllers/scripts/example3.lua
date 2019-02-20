require("context")
require("geometric")

local u = UrdfExpr()
u:readFromParam("/robot_description")
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

maxvel = 1.0
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

tgt_pose = ctx:createInputChannelFrame("tgt_pose")

d = Variable{context = ctx, name = "d", vartype = "feature"}

Constraint{
    context = ctx,
    name = "laserdistance",
    expr = d,
    target_lower = 0.5,
    target_upper = 0.9,
    K = 4
}

laserspot = robot_ee * tgt_pose * vector(0, 0, d)

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
ctx:setOutputExpression("laser_x", coord_x(laserspot))
ctx:setOutputExpression("laser_y", coord_y(laserspot))
ctx:setOutputExpression("laser_z", coord_z(laserspot))

ctx:setOutputExpression("out_pose", robot_ee * tgt_pose)
