--[[
This is a copy of the script embedded in the V-REP model file.
This copy is done so that it is possible to examine the code even
without installing V-REP. Please refer to the V-REP api
<http://www.coppeliarobotics.com/helpFiles/en/apiOverview.htm>
as well as the model screenshot for reference.

Copyright (C) 2019 Lauri Peltonen

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
]]

-- Return length of 3-dimensional vector
local function vectorLength(a)
    return math.sqrt(a[1]*a[1] + a[2]*a[2] + a[3]*a[3])
end

function sysCall_init()
    -- Leg base node
    base = sim.getObjectHandle('leg_base')

    -- Get motor nodes
    -- Leg
    rot_motor = sim.getObjectHandle('leg_rot')
    tilt_motor = sim.getObjectHandle('leg_tilt')
    joint1_motor = sim.getObjectHandle('leg_joint0')
    joint2_motor = sim.getObjectHandle('leg_joint1')

    -- Foot
    foot_rot_motor = sim.getObjectHandle('foot_rot')
    foot_tilt_motor = sim.getObjectHandle('foot_tilt')
    foot_joint_motor = sim.getObjectHandle('foot_joint')


    -- Target and foot marker
    tip = sim.getObjectHandle('leg_tip_point')
    target = sim.getObjectHandle('leg_tip_target')

    -- Calculate joint lengths
    local pos1 = sim.getObjectPosition(joint2_motor, joint1_motor) -- Position of leg part 1
    local pos2 = sim.getObjectPosition(tip, joint2_motor) -- Position of leg part 2
    length1 = vectorLength(pos1) -- Length of first leg segment
    length2 = vectorLength(pos2) -- Length of second leg segment

end


-- Calculate quadrant-correct arctan
local function atan2(x, y)
    local val = 3.14159 / 2.0
    if x == 0 then
        if y > 0 then
            return val
        else
            return -val
        end
    end
    val = math.atan(y/x)
    if x > 0 then
        return val
    end
    if y < 0 then
        return val - 3.14159
    end
    return val + 3.14159
end

-- Calculate cross product of two 3-dimensional vectors
local function cross(a, b)
    c = {0, 0, 0}
    c[1] = a[2]*b[3] - a[3]*b[2]
    c[2] = a[3]*b[1] - a[1]*b[3]
    c[3] = a[1]*b[2] - a[2]*b[1]
    return c
end


function sysCall_actuation()
    -- Get target position for leg
    local target_pos_orig = sim.getObjectPosition(target, base)

    -- "De-rotate" the target
    local rot_angle = sim.getJointPosition(rot_motor)
    local target_pos = {target_pos_orig[1]*math.cos(-rot_angle) - target_pos_orig[2]*math.sin(-rot_angle),
                        target_pos_orig[1]*math.sin(-rot_angle) + target_pos_orig[2]*math.cos(-rot_angle),
                        target_pos_orig[3]}

    -- Calculate target distances
    local target_dist2 = target_pos[1]*target_pos[1] + target_pos[2]*target_pos[2] + target_pos[3]*target_pos[3]
    local target_dist = math.sqrt(target_dist2)

    -- If target point is not reachable, go to the nearest point
    if target_dist > (length1+length2) then
        -- Target is too far --> move it closer
        local scale = (length2+length1) / target_dist
        target_pos = {target_pos[1]*scale, target_pos[2]*scale, target_pos[3]*scale}
        target_dist = (length2+length1)
        target_dist2 = target_dist*target_dist
    end

    -- Only calculate IK if the point is reachable
    if target_dist >= (length2-length1) and target_dist <= (length1+length2) then
        -- Calculate rotation
        target_tilt = atan2(target_pos[2], target_pos[3]) + 1.5708    -- Tilting (z / y) + pi/2

        -- Vector that points "down" in direction of the tilt
        local down_vector = {0.0, math.sin(target_tilt), -math.cos(target_tilt)}

        -- Calculate distance that is normal to the tilt angle
        local target_vdist = -(down_vector[1]*target_pos[1] + down_vector[2]*target_pos[2] + down_vector[3]*target_pos[3])

        -- Calculate distance that is perpendicular to the tilt angle
        local target_hdist = math.sqrt(target_dist2 - target_vdist*target_vdist)
        local perp_vector = cross(down_vector, target_pos) -- Use cross product to check which side we are on
        if perp_vector[2] > 0 then -- target is "behind" the joint
            target_hdist = -target_hdist
        end
        
        -- Calculate angles of the two joints
        local alpha = math.acos((length1*length1 - length2*length2 + target_dist2)/(2.0*length1*target_dist))
        local beta = math.acos((length1*length1 + length2*length2 - target_dist2)/(2.0*length1*length2))

        -- Default angle (zero) is 90 degrees for joint 2
        local target_ang2 = beta - 1.5708 -- pi/2

        -- Angle between vertical and horizontal triangle
        local Adot = atan2(target_hdist, target_vdist)
        local target_ang1 = Adot + alpha

        -- Set the target positions for joints
        sim.setJointTargetPosition(tilt_motor, target_tilt)
        sim.setJointTargetPosition(joint1_motor, target_ang1)
        sim.setJointTargetPosition(joint2_motor, target_ang2)

        -- Set the foot orientation (try to keep it horizontal)
        sim.setJointTargetPosition(foot_tilt_motor, -target_tilt)
        sim.setJointTargetPosition(foot_joint_motor, -(target_ang1+target_ang2))
    end
end