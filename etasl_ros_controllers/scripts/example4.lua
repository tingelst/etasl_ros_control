require("context")
require("geometric")

local u = UrdfExpr()
u:readFromParam("/robot_description")
u:addTransform("ee", "tool0", "base_link")

local r = u:getExpressions(ctx)

robot_ee = r.ee
robot_joints = {
    "joint_a1",
    "joint_a2",
    "joint_a3",
    "joint_a4",
    "joint_a5",
    "joint_a6"
}

maxvel = 0.25
for i = 1, #robot_joints do
    BoxConstraint{
        context = ctx,
        var_name = robot_joints[i],
        lower = -maxvel,
        upper = maxvel
    }
end

default_target = Frame(Rotation.RPY(0.0, math.pi, 0.0), Vector(0.7, 0.0, 0.4))
target = ctx:createInputChannelFrame("target", default_target)
maxvel = ctx:createInputChannelScalar("maxvel", 0.3)

Constraint{
    context = ctx,
    name = "endpoint",
    expr = inv(target) * robot_ee,
    K = 10,
    priority = 0
}
