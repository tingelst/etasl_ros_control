require("context")
require("geometric3")
require("output_utils")
require("collision")

local u = UrdfExpr()
u:readFromParam("/robot_description")
u:addTransform("endeff","ee","base_link")

local r = u:getExpressions(ctx)
robotiq_frame = r.endeff

force_z = ctx:createInputChannelScalar("force_z")
tgt_x = ctx:createInputChannelScalar("tgt_x")
tgt_y = ctx:createInputChannelScalar("tgt_y")
tgt_z = ctx:createInputChannelScalar("tgt_z")
block_frame = ctx:createInputChannelFrame("block_frame")

d = Variable{context = ctx, name = "d", vartype = "feature"}

pi = cached(acos(constant(-1.0)))
SWtrans = translate_z(0.02)*translate_y(0.5)*rotate_x(pi/2)

-- PICKUP PEG CONSTRAINTS
robotiq_orig = origin(robotiq_frame*translate_z(0.105))
robotiq_dir = -unit_z(rotation(robotiq_frame))
peg_orig = origin(SWtrans * frame(vector(0.5, 0.0, 0.0))) --origin(block_frame)
peg_dir = vector(0.0, 0.0, 1.0) --unit_z(rotation(block_frame))
--peg_orig = origin(SWtrans * frame(vector(0.5, -0.1, 0.0))) --Can this be an input from the Zivid camera??

ctx:pushGroup("pickup_lineup")
coincident_line_line(robotiq_orig,robotiq_dir,peg_orig,peg_dir,
    ctx,
    "pickup",
    0.7,
    1.5,
    2
)
ctx:popGroup()

ctx:pushGroup("pickup_closein")
Constraint{
    context     = ctx,
    name        = "RobotiqPeg_originDist",
    expr        = distance_point_point(robotiq_orig,peg_orig),
    K           = 0.5, 
    weight      = 1.1,
    priority    = 2
}
ctx:popGroup()

-- LINEUP PEG/HOLE CONSTRAINTS
L2_orig = origin( SWtrans * frame( vector(0.025, 0.0, -0.025)) )
L2_dir = vector( 0.0, 0.0, 1.0 )
L1_dir = -unit_z( rotation(robotiq_frame) )
L1_orig = origin( robotiq_frame*translate_z(0.105) )

ctx:pushGroup("insertion_lineup")
concentric(L1_orig, L1_dir, L2_orig, L2_dir, 
    ctx, 
    "lineup", 
    0.5, 
    1.5, 
    2
)
ctx:popGroup()

-- INSERTION CONSTRAINTS
Coincident1plane1Orig = origin( SWtrans * frame( vector(0.025, 0.0, -0.025)) )
Coincident1plane1Normal = vector( 0.0, 0.0, 1.0 )
Coincident1plane2Normal = -unit_z( rotation(robotiq_frame) )
Coincident1plane2Orig = origin( robotiq_frame*translate_z(0.105) )

ctx:pushGroup("insertion_closein")
coincident_plane_plane(Coincident1plane1Orig,Coincident1plane1Normal,Coincident1plane2Orig,Coincident1plane2Normal,
    ctx,
    "insertion",
    0.5,
    1.0,
    2
)
ctx:popGroup()

-- NEVER COLLIDE WITH TABLE
table = Box(0.8, 1.53, 0.87) --ConvexObject("/home/daglof/table.obj") --
robotiq_coll = CapsuleZ(0.065,0.0)

ctx:pushGroup("safety")
Constraint{
    context         = ctx,
    name            = "NoCollisionWithTable",
    expr            = distance_between(frame(rot_z(pi/2),vector(0.675, 0.2, -0.85)),table,0.0,0.01, robotiq_frame*translate_z(-0.055),robotiq_coll,0.0,0.01),
    target_lower    = 0.01, --target_upper    = 2.0,
    K               = 1.0,
    weight          = 1.0,
    priority        = 2,
}
ctx:popGroup()

-- COLLISION AVOIDANCE CONSTRAINT
holes = Box(0.25, 0.025 , 0.05)
peg = CylinderZ(0.02, 0.02, 0.05)

ctx:pushGroup("collision")
Constraint{
    context         = ctx,
    name            = "CollisionAvoidance",
    expr            = distance_between(robotiq_frame*translate_z(0.055),peg,0.0,0.01, SWtrans*frame(vector(0,0,-0.05)),holes,0.0,0.01),
    target_lower    = 0.01, --target_upper    = 1.0,
    K               = 1.0,
    weight          = 2.0,
    priority        = 2
}
ctx:popGroup()

laserspot = robotiq_frame * vector(0, 0, d)

--LISSAJOUS CONSTRAINTS
ctx:pushGroup("lissajous")
Constraint{
    context = ctx,
    name = "x",
    expr = tgt_x - coord_x(laserspot) + coord_x(L2_orig),
    priority = 2,
    K = 10
}
Constraint{
    context = ctx,
    name = "y",
    expr = tgt_y - coord_y(laserspot) + coord_y(L2_orig),
    priority = 2,
    K = 10
}
Constraint{
    context = ctx,
    name = "z",
    expr = tgt_z - coord_z(laserspot) + coord_z(L2_orig),
    priority = 2,
    K = 10
}
ctx:popGroup()

ctx:activate_cmd("+global.pickup_lineup +global.safety") 
print(ctx)

ctx:activate_cmd("+global.pickup_closein") 
print(ctx)

-- close gripper

ctx:activate_cmd("-global.pickup_lineup -global.pickup_closein +global.insertion_lineup +global.collision") 
print(ctx)

ctx:activate_cmd("-global.collision +global.insertion_closein")  
print(ctx)

-- --If force too large
-- ctx:activate_cmd("+global.lissajous")
-- print(ctx)

-- Monitor{
--     context     = ctx,
--     name        = "ready_for_pickup",
--     expr        = distance_point_point(robotiq_orig,peg_orig),
--     upper       = 1E-3,
--     actionname  = "exit",
--     action      = "active: -global.pickup +global.lineup"
-- }

-- ctx:activate_cmd("+global.pickup_lineup +global.pickup_closein +global.safety") 
-- print(ctx)