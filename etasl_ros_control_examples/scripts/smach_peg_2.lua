require("context")
require("geometric3")
require("output_utils")
require("collision")

local u = UrdfExpr()
u:readFromParam("/robot_description")
u:addTransform("endeff","ee","base_link")

local r = u:getExpressions(ctx)
robotiq_frame = r.endeff

ndx = 1
speed = ctx:createInputChannelScalar("speed")

-- PEG IN GRIPPER
pegInGripper_orig = origin(robotiq_frame*translate_z(0.105))
pegInGripper_dir = -unit_z(rotation(robotiq_frame))

peg_frame = {}
for i=1,5 do 
    peg_frame[i] = ctx:createInputChannelFrame("peg"..i.."_frame")
end

-- PICKUP CONSTRAINTS
peg_orig = origin( peg_frame[ndx] )
peg_dir = unit_z(rotation( peg_frame[ndx] ))
ctx:pushGroup("pickup_lineup_1")
coincident_line_line(pegInGripper_orig,pegInGripper_dir,peg_orig,peg_dir,
    ctx,
    "pickup1",
    1.5*speed,
    1.5,
    2
)
ctx:popGroup()

ctx:pushGroup("pickup_closein_1")
Constraint{
    context     = ctx,
    name        = "RobotiqPeg_originDist",
    expr        = distance_point_point(pegInGripper_orig,peg_orig),
    K           = 0.3*speed, 
    weight      = 1.1,
    priority    = 2
}
Monitor{
    context = ctx,
    name = "pickup_reached",
    expr = distance_point_point(pegInGripper_orig,peg_orig),
    lower = 1E-3,
    actionname = "exit"
}
ctx:popGroup()

ctx:setOutputExpression("e_event", constant(0.0))

ctx:activate_cmd("+global.pickup_lineup_1 +global.pickup_closein_1")
