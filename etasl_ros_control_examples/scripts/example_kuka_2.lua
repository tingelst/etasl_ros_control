-- Copyright (c) 2019 Norwegian University of Science and Technology
-- Use of this source code is governed by the LGPL-3.0 license, see LICENSE
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

maxvel = 0.6
for i = 1, #robot_joints do
    BoxConstraint{
        context = ctx,
        var_name = robot_joints[i],
        lower = -maxvel,
        upper = maxvel
    }
end

deg2rad = math.pi / 180.0

Constraint{
    context = ctx,
    name = "c1",
    expr = j1,
    target = 45 * deg2rad,
    priority = 2,
    K = 4
}
Constraint{
    context = ctx,
    name = "c2",
    expr = j2,
    target = -120 * deg2rad,
    priority = 2,
    K = 4
}
Constraint{
    context = ctx,
    name = "c3",
    expr = j3,
    target = 120 * deg2rad,
    priority = 2,
    K = 4
}
Constraint{
    context = ctx,
    name = "c4",
    expr = j4,
    target = 0 * deg2rad,
    priority = 2,
    K = 4
}
Constraint{
    context = ctx,
    name = "c5",
    expr = j5,
    target = 45 * deg2rad,
    priority = 2,
    K = 4
}
Constraint{
    context = ctx,
    name = "c6",
    expr = j6,
    target = 45 * deg2rad,
    priority = 2,
    K = 4
}
