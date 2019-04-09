require("context")
require("geometric3")
require("output_utils")
require("collision")

local u = UrdfExpr()
u:readFromParam("/robot_description")
u:addTransform("endeff","ee","base_link")

local r = u:getExpressions(ctx)
robotiq_frame = r.endeff

block_frame = ctx:createInputChannelFrame("block_frame")

ndx = 1
speed = ctx:createInputChannelScalar("speed")

-- PEG IN GRIPPER
pegInGripper_orig = origin(robotiq_frame*translate_z(0.105))
pegInGripper_dir = -unit_z(rotation(robotiq_frame))
-- HOLE LINES
holes_frame = {}
holes_frame[1] = block_frame*frame(vector(0.025,0.0,-0.025))
for i=2,5 do
    holes_frame[i] = holes_frame[i-1]*translate_x(0.05)
end 

-- LINEUP PEG/HOLE CONSTRAINTS
hole_orig = origin( holes_frame[ndx] )
hole_dir = unit_y(rotation( holes_frame[ndx] ))

ctx:pushGroup("insertion_lineup_1")
concentric(pegInGripper_orig, pegInGripper_dir, hole_orig, hole_dir, 
    ctx, 
    "lineup1", 
    0.3*speed, 
    1.5, 
    2
)
Constraint{
    context     = ctx,
    name        = "lineup1_dist",
    expr        = distance_plane_plane(pegInGripper_orig, pegInGripper_dir, hole_orig, hole_dir),
    target      = 0.1,
    K           = 0.3*speed, 
    weight      = 1.1,
    priority    = 2
}
Monitor{
    context = ctx,
    name = "insertion_lineup_reached",
    expr = distance_line_line(pegInGripper_orig, pegInGripper_dir, hole_orig, hole_dir),
    lower = 1E-3,
    actionname = "exit"
}
ctx:popGroup()

ctx:activate_cmd("+global.insertion_lineup_1")

ctx:setOutputExpression("e_event", constant(0.0))