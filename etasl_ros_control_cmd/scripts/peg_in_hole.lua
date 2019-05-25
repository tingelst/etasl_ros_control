require("context")
require("geometric3")
require("output_utils")
require("collision")
require("velocities")

local u = UrdfExpr()
u:readFromParam("/robot_description")
u:addTransform("endeff","ee","base_link")
u:addTransform("a5","link_5","base_link")

local r = u:getExpressions(ctx)
robotiq_frame = r.endeff
a5_frame = r.a5

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

peg_frame = {}
for i=1,5 do 
    peg_frame[i] = make_constant(ctx:createInputChannelFrame("peg"..i.."_frame"))
end
speed = make_constant(ctx:createInputChannelScalar("speed"))
block_frame = make_constant(ctx:createInputChannelFrame("block_frame"))
netft_data = ctx:createInputChannelWrench("netft_data")
netft_transform = ref_point(netft_data,vector(0.0,0.0,0.195))

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

for i=1,5 do
    -- PICKUP CONSTRAINTS
    local peg_orig = origin( peg_frame[i] )
    local peg_dir = unit_z(rotation( peg_frame[i] ))

    local hole_orig = origin( holes_frame[i] )
    local hole_dir = unit_y(rotation( holes_frame[i] ))

    ctx:pushGroup("pickup_lineup_"..i)
    Coincident_line_line{
        context = ctx,
        name = "pickup"..i,
        K = 0.3*speed,
        weight = 1.5,
        priority = 2,
        point_a = pegInGripper_orig,
        dir_a = pegInGripper_dir,
        point_b = peg_orig,
        dir_b = peg_dir
    }
    Constraint{
        context     = ctx,
        name        = "pickup_dist"..i,
        expr        = distance_plane_plane(pegInGripper_orig, pegInGripper_dir, peg_orig,peg_dir),
        target      = 0.1,
        K           = 0.2*speed, 
        weight      = 1.1,
        priority    = 2
    }
    Monitor{
        context = ctx,
        name = "exit",
        expr = distance_line_line(pegInGripper_orig,pegInGripper_dir,peg_orig,peg_dir),
        lower = 1E-2,
        actionname = "exit",
        argument = "-global.pickup_lineup_"..i.." +global.pickup_closein_"..i
    }
    ctx:popGroup()

    ctx:pushGroup("pickup_closein_"..i)
    Coincident_line_line{
        context = ctx,
        name = "pickup"..i,
        K = 1.5*speed,
        weight = 1.5,
        priority = 2,
        point_a = pegInGripper_orig,
        dir_a = pegInGripper_dir,
        point_b = peg_orig,
        dir_b = peg_dir
    }
    Constraint{
        context     = ctx,
        name        = "RobotiqPeg_originDist"..i,
        expr        = distance_point_point(pegInGripper_orig,peg_orig),
        K           = 0.3*speed, 
        weight      = 1.1,
        priority    = 2
    }
    Monitor{
        context = ctx,
        name = "exit",
        expr = distance_point_point(pegInGripper_orig,peg_orig),
        lower = 1E-3,
        actionname = "exit",
        argument = "-global.pickup_closein_"..i
    }
    ctx:popGroup()

    ctx:pushGroup("insertion_lineup_"..i)
    Concentric{
        context     = ctx, 
        name        = "lineup"..i, 
        K           = 0.3*speed, 
        weight      = 1.5, 
        priority    = 2,
        point_a     = pegInGripper_orig,
        dir_a       = pegInGripper_dir,
        point_b     = hole_orig,
        dir_b       = hole_dir
    }
    Constraint{
        context     = ctx,
        name        = "lineup_dist"..i,
        expr        = distance_plane_plane(pegInGripper_orig, pegInGripper_dir, hole_orig, hole_dir),
        target      = 0.1,
        K           = 0.5*speed, 
        weight      = 1.1,
        priority    = 2
    }
    Monitor{
        context = ctx,
        name = "exit",
        expr = distance_line_line(pegInGripper_orig, pegInGripper_dir, hole_orig, hole_dir),
        lower = 1E-3,
        actionname = "exit",
        argument = "-global.insertion_lineup_"..i.." +global.insert_"..i--"-global.insertion_lineup_"..i.." +global.compliance"--
    }
    ctx:popGroup()

    ctx:pushGroup("insert_"..i)
    Concentric{
        context     = ctx, 
        name        = "lineup"..i, 
        K           = 1.0*speed, 
        weight      = 1.5, 
        priority    = 2,
        point_a     = pegInGripper_orig,
        dir_a       = pegInGripper_dir,
        point_b     = hole_orig,
        dir_b       = hole_dir
    }
    Coincident_plane_plane{
        context     = ctx,
        name        = "insertion"..i,
        K           = 0.5*speed,
        weight      = 1.0,
        priority    = 2,
        point_a     = insertionplane_orig,
        dir_a       = insertionplane_normal,
        point_b     = pegInGripper_orig,
        dir_b       = pegInGripper_dir
    }
    Monitor{
        context = ctx,
        name = "exit",
        expr = distance_plane_plane(insertionplane_orig,insertionplane_normal,pegInGripper_orig,pegInGripper_dir),
        lower = 1E-3,
        actionname = "exit",
        argument = "-global.insert_"..i.." -global.compliance"
    }
    ctx:popGroup()
    
    ctx:pushGroup("retract_"..i)
    Concentric{
        context     = ctx, 
        name        = "lineup"..i, 
        K           = 0.3*speed, 
        weight      = 1.5, 
        priority    = 2,
        point_a     = pegInGripper_orig,
        dir_a       = pegInGripper_dir,
        point_b     = hole_orig,
        dir_b       = hole_dir
    }
    Constraint{
        context = ctx,
        name = "retract",
        expr = distance_plane_plane(insertionplane_orig,insertionplane_normal,pegInGripper_orig,pegInGripper_dir),
        target = 0.1,
        K = 0.5*speed, 
        weight = 1.0,
        priority = 2
    }
    Monitor{
        context = ctx,
        name = "exit",
        expr = distance_plane_plane(pegInGripper_orig, pegInGripper_dir, hole_orig, hole_dir),
        upper = 0.1 - 1E-2,
        actionname = "exit",
        argument = "-global.retract_"..i.." +global.pickup_lineup_"..(i+1)
    }
    ctx:popGroup()

end

ctx:pushGroup("compliance")
Constraint{
    context = ctx,
    name = "compliance_x",
    model = -(1/1)*coord_x(origin(robotiq_frame)),
    meas = coord_x(force(netft_transform)),
    target = 0.0,
    K = 1,
    weight = 1.0,
    priority = 2
}
Constraint{
    context = ctx,
    name = "compliance_y",
    model = -(1/1)*coord_y(origin(robotiq_frame)),
    meas = coord_y(force(netft_transform)),
    target = 0.0,
    K = 1,
    weight = 1.0,
    priority = 2
}
Constraint{
    context = ctx,
    name = "compliance_z",
    model = -(1/1)*coord_z(origin(robotiq_frame)),
    meas = coord_z(force(netft_transform)),
    target = -50.0,
    K = 1,
    weight = 1.0,
    priority = 2
}
ctx:popGroup()

-- HOME POSITION
ctx:pushGroup("home")
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
ctx:popGroup()

VelocityLimits {
    context = ctx,
    name    = "max_speed_cartesian_a5",
    expr    = origin(a5_frame),
    limit   = 0.25
}
VelocityLimits{
    context = ctx,
    name    = "max_speed_cartesian_gripper",
    expr    = origin(robotiq_frame),
    limit   = 0.25
}

ctx:activate_cmd("+global.pickup_lineup_1")

ctx:setOutputExpression("ee_frame", frame(rotation(robotiq_frame)*rot_y(constant(180* deg2rad)),pegInGripper_orig))
ctx:setOutputExpression("peg_hole_1", frame(rotation(holes_frame[1])*rot_x(constant(-90* deg2rad)),origin(holes_frame[1])))
ctx:setOutputExpression("peg_hole_2", frame(rotation(holes_frame[2])*rot_x(constant(-90* deg2rad)),origin(holes_frame[2])))
ctx:setOutputExpression("peg_hole_3", frame(rotation(holes_frame[3])*rot_x(constant(-90* deg2rad)),origin(holes_frame[3])))
ctx:setOutputExpression("peg_hole_4", frame(rotation(holes_frame[4])*rot_x(constant(-90* deg2rad)),origin(holes_frame[4])))
ctx:setOutputExpression("peg_hole_5", frame(rotation(holes_frame[5])*rot_x(constant(-90* deg2rad)),origin(holes_frame[5])))
ctx:setOutputExpression("netft_transform", netft_transform)