require("context")
require("geometric2")
require("output_utils")
require("collision")

local u = UrdfExpr()
u:readFromParam("/robot_description")
u:addTransform("endeff","ee","base_link")

local r = u:getExpressions(ctx)
robotiq_frame = r.endeff


pi = cached(acos(constant(-1.0)))
SWtrans = translate_z(0.02)*translate_y(0.5)*rotate_x(pi/2)


-- PICKUP PEG CONSTRAINTS
robotiq_orig = origin(robotiq_frame*translate_z(0.105))
robotiq_dir = -unit_z(rotation(robotiq_frame))
peg_orig = origin(SWtrans * frame(vector(0.5, 0.0, 0.0))) --Can this be an input from the Zivid camera??
peg_dir = vector(0.0, 0.0, 1.0) --Can this be an input from the Zivid camera??

ctx:pushGroup("pickup")
Constraint{
    context     = ctx,
    name        = "RobotiqPeg_originDist",
    expr        = distance_point_point(robotiq_orig,peg_orig),
    K           = 0.5, 
    weight      = 1.0,
    priority    = 2
}
Constraint{
    context     = ctx,
    name        = "RobotiqPeg_angle",
    expr        = angle_line_line(robotiq_dir,peg_dir),
    K           = 0.7,
    weight      = 1.5,
    priority    = 2
}
Constraint{
    context     = ctx,
    name        = "RobotiqPeg_LineDist",
    expr        = distance_line_line(robotiq_dir,robotiq_orig,peg_dir,peg_orig),
    K           = 0.7,
    weight      = 1.5,
    priority    = 2
}
ctx:popGroup()

-- LINEUP PEG/HOLE CONSTRAINTS
L2_orig = origin( SWtrans * frame( vector(0.025, 0.0, -0.025)) )
L2_dir = vector( 0.0, 0.0, 1.0 )
L1_dir = -unit_z( rotation(robotiq_frame) )
L1_orig = origin( robotiq_frame*translate_z(0.105) )

ctx:pushGroup("lineup")
Constraint{
    context     = ctx,
    name        = "LineLine_angle",
    expr        = angle_line_line(L1_dir, L2_dir),
    K           = 0.5,
    weight      = 1.5,
    priority    = 2
}
Constraint{
    context     = ctx,
    name        = "LineLine_dist",
    expr        = distance_line_line(L1_dir, L1_orig, L2_dir, L2_orig),
    K           = 0.5,
    weight      = 1.5,
    priority    = 2
}
ctx:popGroup()

-- INSERTION CONSTRAINTS
Coincident1plane1Orig = origin( SWtrans * frame( vector(0.025, 0.0, -0.025)) )
Coincident1plane1Normal = vector( 0.0, 0.0, 1.0 )
Coincident1plane2Normal = -unit_z( rotation(robotiq_frame) )
Coincident1plane2Orig = origin( robotiq_frame*translate_z(0.105) )

ctx:pushGroup("insertion")
-- Constraint{
--     context = ctx,
--     name = "Coincident_PlanePlane_Par",
--     expr = angle_plane_plane(Coincident1plane1Normal,Coincident1plane2Normal),
--     K = 0.5,
--     weight = 1.0,
--     priority = 2
-- }
-- Constraint{
--     context = ctx,
--     name = "Coincident_PlanePlane_Dist",
--     expr = distance_plane_plane(Coincident1plane1Normal,Coincident1plane2Normal,Coincident1plane1Orig,Coincident1plane2Orig),
--     K = 0.5,
--     weight = 1.0,
--     priority = 2
-- }
Constraint{
    context     = ctx,
    name        = "Coincident_PlanePlane_point",
    expr        = distance_point_point(Coincident1plane1Orig,Coincident1plane2Orig),
    K           = 0.4,
    weight      = 1.0,
    priority    = 2
}
ctx:popGroup()

-- COLLISION AVOIDANCE CONSTRAINT
holes = Box(0.25, 0.05 , 0.025)
peg = CylinderZ(0.02, 0.02, 0.05)

ctx:pushGroup("collision")
Constraint{
    context         = ctx,
    name            = "CollisionAvoidance",
    expr            = distance_between(robotiq_frame*translate_z(0.105),peg,0,0.01, SWtrans*frame(vector(0,0,0)),holes,0,0.01),
    target_lower    = 0.01,
    target_upper    = 0.1,
    K               = 1,
    weight          = 2.0,
    priority        = 2
}
ctx:popGroup()

ctx:activate_cmd("+global.lineup +global.insertion") -- +global.pickup +global.lineup +global.insertion +global.collision")


-- Monitor{
--     context     = ctx,
--     name        = "ready_for_pickup",
--     expr        = distance_point_point(robotiq_orig,peg_orig),
--     upper       = 1E-3,
--     actionname  = "exit",
--     action      = "active: -global.pickup +global.lineup"
-- }


print(ctx)