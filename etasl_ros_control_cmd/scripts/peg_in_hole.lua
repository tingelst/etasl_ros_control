require("context")
require("geometric3")
require("output_utils")
require("collision")

local u = UrdfExpr()
u:readFromParam("/robot_description")
u:addTransform("endeff","ee","base_link")

local r = u:getExpressions(ctx)
robotiq_frame = r.endeff

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
    peg_frame[i] = ctx:createInputChannelFrame("peg"..i.."_frame")
end
speed = ctx:createInputChannelScalar("speed")
block_frame = ctx:createInputChannelFrame("block_frame")
netft_data = ctx:createInputChannelWrench("netft_data")

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
    coincident_line_line(pegInGripper_orig,pegInGripper_dir,peg_orig,peg_dir,
        ctx,
        "pickup"..i,
        0.3*speed,
        1.5,
        2
    )
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
    coincident_line_line(pegInGripper_orig,pegInGripper_dir,peg_orig,peg_dir,
        ctx,
        "pickup"..i,
        1.5*speed,
        1.5,
        2
    )
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
    concentric(pegInGripper_orig, pegInGripper_dir, hole_orig, hole_dir, 
        ctx, 
        "lineup"..i, 
        0.3*speed, 
        1.5, 
        2
    )
    Constraint{
        context     = ctx,
        name        = "lineup_dist"..i,
        expr        = distance_plane_plane(pegInGripper_orig, pegInGripper_dir, hole_orig, hole_dir),
        target      = 0.1,
        K           = 0.3*speed, 
        weight      = 1.1,
        priority    = 2
    }
    Monitor{
        context = ctx,
        name = "exit",
        expr = distance_line_line(pegInGripper_orig, pegInGripper_dir, hole_orig, hole_dir),
        lower = 1E-3,
        actionname = "exit",
        argument = "-global.insertion_lineup_"..i.." +global.insert_"..i
    }
    Monitor{
        context = ctx,
        name = "high_force",
        expr = coord_z(force(netft_data)),
        lower = -50.0,
        actionname = "exit",
        argument = "+global.force"
    }
    ctx:popGroup()

    ctx:pushGroup("insert_"..i)
    concentric(pegInGripper_orig, pegInGripper_dir, hole_orig, hole_dir, 
        ctx, 
        "lineup"..i, 
        1.0*speed, 
        1.5, 
        2
    )
    coincident_plane_plane(insertionplane_orig,insertionplane_normal,pegInGripper_orig,pegInGripper_dir,
        ctx,
        "insertion"..i,
        0.5*speed,
        1.0,
        2
    )
    Monitor{
        context = ctx,
        name = "exit",
        expr = distance_plane_plane(insertionplane_orig,insertionplane_normal,pegInGripper_orig,pegInGripper_dir),
        lower = 1E-3,
        actionname = "exit",
        argument = "-global.insert_"..i.." -global.force"
    }
    ctx:popGroup()
    
    ctx:pushGroup("retract_"..i)
    concentric(pegInGripper_orig, pegInGripper_dir, hole_orig, hole_dir, 
        ctx, 
        "lineup"..i, 
        0.3*speed, 
        1.5, 
        2
    )
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

ctx:activate_cmd("+global.pickup_lineup_1")
