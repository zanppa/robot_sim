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

-- Return length of a 3-dimensional vector
function vectorLength(a)
    return math.sqrt(a[1]*a[1] + a[2]*a[2] + a[3]*a[3])
end

function sysCall_init()
    -- Leg base dummy
    base = sim.getObjectHandle('leg_base')

    -- Get motor nodes
    rot_motor = sim.getObjectHandle('leg_rot')
    joint1_motor = sim.getObjectHandle('leg_joint0')
    joint2_motor = sim.getObjectHandle('leg_joint1')
    tip = sim.getObjectHandle('leg_tip_point')
    target = sim.getObjectHandle('leg_tip_target')

    -- Calculate joint lengths
    local pos1 = sim.getObjectPosition(joint2_motor, joint1_motor) -- Position of leg part 1
    local pos2 = sim.getObjectPosition(tip, joint2_motor) -- Position of leg part 2
    length1 = vectorLength(pos1) -- Length of leg segment 1
    length2 = vectorLength(pos2) -- Length of leg segment 2

end


-- Calculate quadrant-correct atan
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


function sysCall_actuation()
    -- Get target position for leg
    target_pos = sim.getObjectPosition(target, base)
    target_dist2 = target_pos[1]*target_pos[1] + target_pos[2]*target_pos[2] + target_pos[3]*target_pos[3]
    target_dist = math.sqrt(target_dist2)
    target_hdist = math.sqrt(target_pos[1]*target_pos[1] + target_pos[2]*target_pos[2])

    -- Only calculate IK if the point is reachable
    if target_dist >= (length2-length1) and target_dist <= (length1+length2) then
        -- Calculate rotation
        target_rot = atan2(target_pos[1], target_pos[2])
        
        -- Calculate angles of the two joints
        alpha = math.acos((length1*length1 - length2*length2 + target_dist2)/(2.0*length1*target_dist))
        beta = math.acos((length1*length1 + length2*length2 - target_dist2)/(2.0*length1*length2))

        -- Default angle (zero) is 90 degrees for joint 2
        target_ang2 = beta - 1.5708 -- pi/2

        -- Angle between vertical and horizontal triangle
        Adot = atan2(target_hdist, target_pos[3])
        target_ang1 = Adot + alpha

        -- Set the target positions
        sim.setJointTargetPosition(rot_motor, target_rot)
        sim.setJointTargetPosition(joint1_motor, target_ang1)
        sim.setJointTargetPosition(joint2_motor, target_ang2)
    end
end