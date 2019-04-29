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
tgt = ctx:createInputChannelVector("tgt")
block_frame = ctx:createInputChannelFrame("block_frame")
netft_data = ctx:createInputChannelWrench("netft_data")

d = Variable{context = ctx, name = "d", vartype = "feature"}
liss_spot = robotiq_frame*vector(0, 0, d)

-- PEG IN GRIPPER
pegInGripper_orig = origin(robotiq_frame*translate_z(0.105))
pegInGripper_dir = -unit_z(rotation(robotiq_frame))
-- INSERTION PLANE
insertionplane_orig = origin( block_frame )
insertionplane_normal = unit_y(rotation( block_frame ))

-- LINEUP PEG/HOLE CONSTRAINTS
hole_orig = origin( block_frame )
hole_dir = unit_y(rotation( block_frame ))

-- INSERTION CONSTRAINTS
ctx:pushGroup("insertion_lineup")
concentric(pegInGripper_orig, pegInGripper_dir, hole_orig, hole_dir, 
    ctx, 
    "lineup", 
    1.0*speed, 
    1.5, 
    2
)
ctx:popGroup()

ctx:pushGroup("insertion_closein")
coincident_plane_plane(insertionplane_orig,insertionplane_normal,pegInGripper_orig,pegInGripper_dir,
    ctx,
    "insertion",
    0.5*speed,
    1.0,
    2
)
Monitor{
    context = ctx,
    name = "insertion_complete",
    expr = distance_plane_plane(insertionplane_orig,insertionplane_normal,pegInGripper_orig,pegInGripper_dir),
    lower = 1E-3,
    actionname = "exit"
}
ctx:popGroup()

ctx:pushGroup("force")
Constraint{
    context = ctx,
    name = "force_x",
    expr = coord_x(force(netft_data)),
    K = 4,
    weight = 10,
    priority = 2
}
Constraint{
    context = ctx,
    name = "force_y",
    expr = coord_y(force(netft_data)),
    K = 4,
    weight = 10,
    priority = 2
}
Constraint{
    context = ctx,
    name = "force_z",
    expr = coord_z(force(netft_data)),
    target_lower = -50,
    K = 4,
    weight = 10,
    priority = 2
}
Constraint{
    context = ctx,
    name = "torque_x",
    expr = coord_x(torque(netft_data)),
    K = 4,
    weight = 10,
    priority = 2
}
Constraint{
    context = ctx,
    name = "torque_y",
    expr = coord_y(torque(netft_data)),
    K = 4,
    weight = 10,
    priority = 2
}
Constraint{
    context = ctx,
    name = "torque_z",
    expr = coord_z(torque(netft_data)),
    K = 4,
    weight = 10,
    priority = 2
}
ctx:popGroup()

ctx:pushGroup("lissajous")
Constraint{
    context = ctx,
    name = "x",
    expr = coord_x(tgt) - coord_x(liss_spot) + coord_x(hole_orig),
    K = 10*speed,
    weight = 1.1,
    priority = 2
}
Constraint{
    context = ctx,
    name = "y",
    expr = coord_y(tgt) - coord_y(liss_spot) + coord_y(hole_orig),
    K = 10*speed,
    weight = 1.1,
    priority = 2
}
Constraint{
    context = ctx,
    name = "z",
    expr = coord_z(tgt) - coord_z(liss_spot) + coord_z(hole_orig),
    K = 10*speed,
    weight = 1.1,
    priority = 2
}
ctx:popGroup()

ctx:activate_cmd("+global.insertion_lineup +global.insertion_closein +global.force")

ctx:setOutputExpression("e_event", constant(0.0))