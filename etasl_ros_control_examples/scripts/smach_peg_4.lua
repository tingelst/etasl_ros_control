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
-- INSERTION PLANE
insertionplane_orig = origin( block_frame )
insertionplane_normal = unit_y(rotation( block_frame ))
-- HOLE LINES
holes_frame = {}
holes_frame[1] = block_frame*frame(vector(0.025,0.0,-0.025))
for i=2,5 do
    holes_frame[i] = holes_frame[i-1]*translate_x(0.05)
end 

-- LINEUP PEG/HOLE CONSTRAINTS
hole_orig = origin( holes_frame[ndx] )
hole_dir = unit_y(rotation( holes_frame[ndx] ))

-- INSERTION CONSTRAINTS
ctx:pushGroup("insertion_lineup_1")
concentric(pegInGripper_orig, pegInGripper_dir, hole_orig, hole_dir, 
    ctx, 
    "lineup1", 
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
    name = "insertion_lineup_reached",
    expr = distance_plane_plane(insertionplane_orig,insertionplane_normal,pegInGripper_orig,pegInGripper_dir),
    lower = 1E-3,
    actionname = "exit"
}
ctx:popGroup()

ctx:activate_cmd("+global.insertion_lineup_1 +global.insertion_closein")

ctx:setOutputExpression("e_event", constant(0.0))