using DrWatson
@quickactivate "Essaim"

using Agents
using DataStructures: CircularBuffer
using Random
using Colors

# we define how each robot behaves, at
function agent_step!(r1, model)
    neighbours = nearby_agents(r1, model, r1.vis_range)
    σ = 1.0

    dpos = r1.pos
    for r2 ∈ neighbours
        dpos = dpos .- σ .* (r1.pos .- r2.pos)
    end

    r1.dθ = 0
    r1.vel = dpos

    r1.pos = r1.pos .+ r1.vel .* model.δt
    r1.θ = atan(r1.vel[2], r1.vel[1])

    return
end

# a sim behaviour for the robots
# - they keep a low speed with 0 rotation speed
# - unless they reach further than X meters from the nearest neighbor,
# then they rotate
function agent_test_step!(robot, model)
    neighbor_ids = nearby_ids(robot, model, robot.vis_range)

    robot.pos = robot.pos .+ robot.vel .* model.δt
    robot.θ += robot.dθ * model.δt

    robot.vel = (
        robot.pos[1]*cos(robot.θ) - robot.pos[2]*sin(robot.θ),
        robot.pos[1]*sin(robot.θ) + robot.pos[2]*cos(robot.θ)
    )

    for id in neighbor_ids
        if euclidean_distance(robot.pos, model[id].pos, model) > robot.com_range
            robot.dθ += 0.01
            return
        end
    end

    robot.dθ -= 0.005

    return
end
