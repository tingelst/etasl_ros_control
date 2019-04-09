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

peg_frame = {}
for i=1,5 do 
    peg_frame[i] = ctx:createInputChannelFrame("peg"..i.."_frame")
end

d = Variable{context = ctx, name = "d", vartype = "feature"}
laserspot = robotiq_frame * vector(0, 0, d)
pi = cached(acos(constant(-1.0)))


-- PEG IN GRIPPER
pegInGripper_orig = origin(robotiq_frame*translate_z(0.105))
pegInGripper_dir = -unit_z(rotation(robotiq_frame))
-- INSERTION PLANE
insertionplane_orig = origin( block_frame )
insertionplane_normal = unit_y(rotation( block_frame ))
-- HOLE LINES
holes_frame = {}
holes_frame[1] = block_frame*frame(vector(0.025,0.0,-0.025))
for i=2,5 do
    holes_frame[i] = holes_frame[i-1]*translate_x(0.05)
end 

-- NEVER COLLIDE WITH TABLE
table = Box(0.8, 1.53, 0.87)--ConvexObject(rospack_find("etasl_ros_control_examples").."/mesh/table.obj") --
robotiq_coll = CapsuleZ(0.065,0.0)--ConvexObject(rospack_find("etasl_ros_control_examples").."/mesh/robotiq_base_link.obj") --CapsuleZ(0.065,0.0)--

ctx:pushGroup("safety")
Constraint{
    context         = ctx,
    name            = "NoCollisionWithTable",
    expr            = distance_between(frame(rot_z(pi/2),vector(0.675, -1.0, -0.85)),table,0.0025,0.0, robotiq_frame*translate_z(0.5),robotiq_coll,0.0025,0.0),--
    target_lower    = 0.1, --target_upper    = 2.0,
    K               = 1.0,
    weight          = 1.0,
    priority        = 2
}
ctx:popGroup()

-- COLLISION AVOIDANCE CONSTRAINT
holes = ConvexObject(rospack_find("etasl_ros_control_examples").."/mesh/block.obj")--Box(0.25, 0.025 , 0.05)--
peg = ConvexObject(rospack_find("etasl_ros_control_examples").."/mesh/peg.obj")--CylinderZ(0.02, 0.02, 0.05)--

ctx:pushGroup("collision")
Constraint{
    context         = ctx,
    name            = "CollisionAvoidance",
    expr            = distance_between(robotiq_frame*translate_z(0.055),peg,0.0025,0.0, block_frame,holes,0.0025,0.0),--
    target_lower    = 0.01, --target_upper    = 1.0,
    K               = 1.0,
    weight          = 2.0,
    priority        = 2
}
ctx:popGroup()

-- INSERTION CONSTRAINTS
ctx:pushGroup("insertion_closein")
coincident_plane_plane(insertionplane_orig,insertionplane_normal,pegInGripper_orig,pegInGripper_dir,
    ctx,
    "insertion",
    0.5,
    1.0,
    2
)
ctx:popGroup()

for i=1,5 do
    -- PICKUP CONSTRAINTS
    local peg_orig = origin( peg_frame[i] )
    local peg_dir = unit_z(rotation( peg_frame[i] ))

    ctx:pushGroup("pickup_lineup_"..i)
    coincident_line_line(pegInGripper_orig,pegInGripper_dir,peg_orig,peg_dir,
        ctx,
        "pickup"..i,
        0.7,
        1.5,
        2
    )
    Monitor{
        context = ctx,
        name = "pickup_lineup_reached",
        expr = distance_line_line(pegInGripper_orig,pegInGripper_dir,origin( peg_frame[i] ),unit_z(rotation( peg_frame[i] ))),
        lower = 1E-3,
        actionname = "activate",
        argument = "+global.pickup_closein_"..i
    }
    ctx:popGroup()

    ctx:pushGroup("pickup_closein_"..i)
    Constraint{
        context     = ctx,
        name        = "RobotiqPeg_originDist"..i,
        expr        = distance_point_point(pegInGripper_orig,peg_orig),
        K           = 0.5, 
        weight      = 1.1,
        priority    = 2
    }
    ctx:popGroup()

    -- LINEUP PEG/HOLE CONSTRAINTS
    local hole_orig = origin( holes_frame[i] )
    local hole_dir = unit_y(rotation( holes_frame[i] ))

    ctx:pushGroup("insertion_lineup_"..i)
    concentric(pegInGripper_orig, pegInGripper_dir, hole_orig, hole_dir, 
        ctx, 
        "lineup"..i, 
        0.5, 
        1.5, 
        2
    )
    ctx:popGroup()

    --LISSAJOUS CONSTRAINTS
    ctx:pushGroup("lissajous_"..i)
    Constraint{
        context = ctx,
        name = "x"..i,
        expr = tgt_x - coord_x(laserspot) + coord_x(hole_orig),
        priority = 2,
        K = 10
    }
    Constraint{
        context = ctx,
        name = "y"..i,
        expr = tgt_y - coord_y(laserspot) + coord_y(hole_orig),
        priority = 2,
        K = 10
    }
    Constraint{
        context = ctx,
        name = "z"..i,
        expr = tgt_z - coord_z(laserspot) + coord_z(hole_orig),
        priority = 2,
        K = 10
    }
    ctx:popGroup()

end

ctx:activate_cmd("+global.pickup_lineup_1")
-- print(ctx)
-- ctx:activate_cmd("+global.pickup_lineup_2 +global.pickup_closein_2")
-- print(ctx)
-- ctx:activate_cmd("+global.pickup_lineup_3 +global.pickup_closein_3")
-- print(ctx)
-- ctx:activate_cmd("+global.pickup_lineup_4 +global.pickup_closein_4")
-- print(ctx)
-- ctx:activate_cmd("+global.pickup_lineup_5 +global.pickup_closein_5")
-- print(ctx)

--ctx:activate_cmd("-global.pickup_lineup_1 +global.insertion_lineup_3 +global.insertion_closein")


-- ctx:setOutputExpression("laser", laserspot)
ctx:setOutputExpression("e_event", constant(0.0))

--print(ctx)

-- ctx:activate_cmd("+global.insertion_closein")
-- print(ctx)


-- for i=1,5 do
-- ctx:activate_cmd("+global.pickup_lineup_i +global.safety")
-- "+global.pickup_closein_i"
-- "Close gripper"
-- "-global.pickup_lineup_i -global.pickup_closein_i +global.insertion_lineup_i +global.collision"
-- "-global.collision +global.insertion_closein"
-- if(force_z>tolerance) "+global.lissajous"
-- "-global.lissajous"
-- "Open gripper"
-- "-global.insertion_closein +global.collision"
-- end

