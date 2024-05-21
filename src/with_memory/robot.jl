using DrWatson
@quickactivate "Essaim"

# applies the laplacian from the task context
function agent_step!(r_i, model)
    N = nagents(model)
    D, R, O, E = model.dimensions  # coordinates, rotations, ontic, epistemic 

    # generate observation
    y_i = observation(r_i.pos, r_i.vel, r_i.x, r_i.dx)

    # perception step (i observes j)
    p_neighbours = nearby_agents(r_i, model, r_i.vis_range)

    p_i = Dict{Int,NTuple{  # Dictionary to store the id and their communication 
        SVector{D,Float64},
        SVector{D,Float64},
        MVector{O,Float64},
        MVector{O,Float64}}
     } 

    for r_j ∈ p_neighbours
        p_i[r_j.id] = perception(r_j.pos, r_j.vel, r_j.x, r_j.dx)
    end

    # communication step (i receives from j)
    c_neighbours = nearby_agents(r_i, model, r_i.com_range)

    c_i = Dict{Int,NTuple{
        SVector{E,Float64},
        SVector{E,Float64}}
    }

    for r_j ∈ c_neighbours
        c_i[r_j.id] = communication(r_j.μ, r_j.dμ)
    end

    # evolve epistemic state
    μ, dμ = epistemic_evolution(r_i.μ, r_i.dμ, y_i, p_i, c_i)

    r_i.μ_i = μ
    r_i.dμ_i = dμ

    # generate control
    u_i = control(r_i.μ_i, r_i.dμ_i)

    # evolve ontic state
    pos, vel, x, dx = ontic_evolution(r.pos, r.vel. r.x, r.dx, u_i)

    r_i.pos = pos
    r_i.vel = vel
    r_i.x = x
    r_i.dx = dx

    return
end

#TODO: Check if this works with variable dimensions on control
function ontic_evolution(pos_i, vel_i, x_i, dx_i, u_i)
    D = size(pos_i)
    O = size(x_i)

    @assert D == size(vel_i)
    @assert O == size(dx_i)
    @assert D + O == size(u_i)

    pos = pos_i
    vel = vel_i
    x = x_i
    dx = dx_i 

    vel = u_i[1:D] # was 'dpos' in the example before
    pos = pos + model.δt * vel

    dx = u_i[D+1:O] # was 'atan(vel[2], vel[1])' and changed directly θ
    x  = x + model.δt * dx

    pos, vel, x, dx
end

#TODO: Finish this code, doesn't make much sense what is being done.
# compare with previous example.
# difference of epistemic states only
function epistemic_evolution(μ_i, y_i, p_i, c_i)
    μ = μ_i 

    σ = 0.1

    for (μ_j, dμ_j) ∈ c_i
        diff = diff - σ * (μ_i - μ_j)
    end

    μ = diff

    μ, dμ
end

function observation(pos_i, vel_i, x_i, dx_i)
    pos_i, vel_i, x_i, dx_i
end

function perception(pos_g, vel_g, x_g, dx_g)
    pos_g, vel_g, x_g, dx_g
end

function communication(μ_j, dμ_j)
    μ_j, dμ_j
end

function control(μ_i, dμ_i)
    - μ_i  # Gradient descent on the new estimated gathering position 
end

function agent_flock_step!(r_i, model)
    neighbours = nearby_agents(r_i, model, r_i.vis_range)

    σ = 1.0
    dpos = r_i.pos

    for r_j ∈ neighbours
        dpos = dpos - σ * (r_i.pos - r_j.pos)
    end

    r_i.dx[1] = 0
    r_i.vel = dpos

    r_i.pos = r_i.pos + r_i.vel * model.δt
    r_i.x[1] = atan(r_i.vel[2], r_i.vel[1])
end


# a sim behaviour for the robots
# - they keep a low speed with 0 rotation speed
# - unless they reach further than X meters from the nearest neighbor,
# then they rotate
function agent_test_step!(robot, model)
    neighbor_ids = nearby_ids(robot, model, robot.vis_range)

    robot.pos = robot.pos + robot.vel * model.δt
    robot.x[1] += robot.dx[1] * model.δt

    robot.vel = (
        robot.pos[1]*cos(robot.x[1]) - robot.pos[2]*sin(robot.x[1]),
        robot.pos[1]*sin(robot.x[1]) + robot.pos[2]*cos(robot.x[1])
    )

    for id in neighbor_ids
        if euclidean_distance(robot.pos, model[id].pos, model) > robot.com_range
            robot.dx[1] += 0.01
            return
        end
    end

    robot.dx[1] -= 0.005

    return
end
