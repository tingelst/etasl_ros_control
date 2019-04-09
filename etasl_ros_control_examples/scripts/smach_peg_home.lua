require("context")
require("geometric3")
require("output_utils")
require("collision")

local u = UrdfExpr()
u:readFromParam("/robot_description")
u:addTransform("endeff","ee","base_link")

local r = u:getExpressions(ctx)
robotiq_frame = r.endeff

speed = ctx:createInputChannelScalar("speed")

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

deg2rad = math.pi / 180.0

Constraint{
    context = ctx,
    name = "c1",
    expr = j1,
    target = 0 * deg2rad,
    priority = 2,
    K = 0.5*speed
}
Constraint{
    context = ctx,
    name = "c2",
    expr = j2,
    target = -90 * deg2rad,
    priority = 2,
    K = 0.5*speed
}
Constraint{
    context = ctx,
    name = "c3",
    expr = j3,
    target = 90 * deg2rad,
    priority = 2,
    K = 0.5*speed
}
Constraint{
    context = ctx,
    name = "c4",
    expr = j4,
    target = 0 * deg2rad,
    priority = 2,
    K = 0.5*speed
}
Constraint{
    context = ctx,
    name = "c5",
    expr = j5,
    target = 90 * deg2rad,
    priority = 2,
    K = 0.5*speed
}
Constraint{
    context = ctx,
    name = "c6",
    expr = j6,
    target = 0 * deg2rad,
    priority = 2,
    K = 0.5*speed
}