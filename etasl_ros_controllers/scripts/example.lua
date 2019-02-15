-- Loading the KUKA KR6 R900 sixx (Agilus) robot with a standard script:
require("context")
require("geometric")

-- loading a model for the KUKA KR6 R900 Sixx (Agilus)
local u = UrdfExpr()
u:readFromFile(rospack_find("etasl_kuka_rsi") .. "/robots/kuka_kr6r900sixx.urdf")
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

deg2rad    = math.pi/180.0

trajectory = constant(-45*deg2rad) + constant(20*deg2rad)*sin( constant(1)*time )
zeroval = constant(0.0)

Constraint{ context=ctx, name="joint_trajectory1", expr= j1 - zeroval };
Constraint{ context=ctx, name="joint_trajectory2", expr= j2 - trajectory };
Constraint{ context=ctx, name="joint_trajectory3", expr= j3 - zeroval };
Constraint{ context=ctx, name="joint_trajectory4", expr= j4 - zeroval };
Constraint{ context=ctx, name="joint_trajectory5", expr= j5 - zeroval };
Constraint{ context=ctx, name="joint_trajectory6", expr= j6 - zeroval };

Monitor{
        context=ctx,
        name='finish_after_some_time',
        upper=10,
        actionname='exit',
        expr=time
}


ctx:setOutputExpression("j2",j2)
ctx:setOutputExpression("trajectory",trajectory)
