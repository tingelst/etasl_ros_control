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

maxvel = 0.3
for i = 1, #robot_joints do
    BoxConstraint{
        context = ctx,
        var_name = robot_joints[i],
        lower = -maxvel,
        upper = maxvel
    }
end

tgt_x = ctx:createInputChannelScalar("tgt_x")
tgt_y = ctx:createInputChannelScalar("tgt_y")
tgt_z = ctx:createInputChannelScalar("tgt_z")

d = Variable{context = ctx, name = "d", vartype = "feature"}

Constraint{
    context = ctx,
    name = "laserdistance",
    expr = d,
    target_lower = 0.5,
    target_upper = 0.9,
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

Monitor{
        context=ctx,
        name="signal_time",
        upper=0.0,
        actionname="event",
	argument="ten_seconds",
        expr=time-constant(10)
}

Monitor{
        context=ctx,
        name="finish_after_motion",
        upper=0.0,
        actionname="exit",
        expr=time-constant(30)
}
ctx:setOutputExpression("error_x", coord_x(laserspot) - tgt_x)
ctx:setOutputExpression("error_y", coord_y(laserspot) - tgt_y)
ctx:setOutputExpression("error_z", coord_z(laserspot) - tgt_z)
ctx:setOutputExpression("laser", laserspot)
