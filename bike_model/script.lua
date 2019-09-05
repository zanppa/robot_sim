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



function sysCall_init()
    wheel = sim.getObjectHandle("wheel_motor")
    handle = sim.getObjectHandle("handle_motor")
    front_wheel = sim.getObjectHandle("front_hub")

    -- Configuration
    wheel_diameter = 0.1 -- m
    local v = sim.getObjectPosition(front_wheel, wheel)
    wheel_distance = math.sqrt(v[1]*v[1] + v[2]*v[2] + v[3]*v[3])

    -- Targets
    velocity = 0.5 -- m/s
    angle_target = 0.0    -- degrees, handle position target
    accel_target = 0.0  -- X axis acceleration target value

    -- Integrators
    accel_int = 0.0
    angle_int = 0.0
end

function sysCall_actuation()
    -- Set rear wheel target velocity
    local target_speed = 2.0 * velocity / wheel_diameter
    sim.setJointTargetVelocity(wheel, target_speed)

    -- Read accelerometer
    local xAccel=sim.getFloatSignal('accelerometerX')
    local yAccel=sim.getFloatSignal('accelerometerY')
    local zAccel=sim.getFloatSignal('accelerometerZ')

    if (not xAccel) or (not yAccel) or (not zAccel) then
        xAccel = 0.0
        yAccel = 0.0
        zAccel = -9.81
    end

    local angle_error = angle_target - sim.getJointPosition(handle)
    local accel_target = 0.01 * angle_error + 0.001 * angle_int
    angle_int = angle_int + angle_error * sim.getSimulationTimeStep()
    print(accel_target)

    local accel_error = accel_target - xAccel
    local control_input = 2.0 * accel_error + 0.5 * accel_int

    accel_int = accel_int + accel_error*sim.getSimulationTimeStep()

    --sim.setJointTargetPosition(handle, -xAccel)
    sim.setJointTargetPosition(handle, control_input)


    --print(angle, control_input)
end

function sysCall_sensing()
    -- put your sensing code here
end

function sysCall_cleanup()
    -- do some clean-up here
end

function setTargetVelocity(vel)
    velocity = vel
end

function setTargetAngle(angle)
    angle_target = angle -- degrees
end
