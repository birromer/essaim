using DrWatson
@quickactivate "Essaim"

# applies the laplacian from the task context
function agent_generic_step!(r_i, model)
    N = nagents(model)

    # WARN: The dimension of  ontic does not consider the D and R.
    # Whenever ontic state is needed, use 'pos', 'vel' and 'onc' fields.
    # Whenever epistemic state is needed, use 'epi' field.
    D, R, O, E = model.dimensions  # coordinates, rotations, ontic, epistemic 

    # generate observation
    y_i = observation(r_i.pos, r_i.vel, r_i.onc)

    # perception step (i observes j)
    p_neighbours = nearby_agents(r_i, model, r_i.vis_range)

    p_i = Dict{Int, Tuple{  # Dictionary to store the id and their communication
                           SVector{D,Float64}, 
                           SVector{D,Float64}, 
                           model.ontic_state }}()

    for r_j ∈ p_neighbours
        # R_i perceives the ontic state of R_j
        p_i[r_j.id] = perception(r_j.pos, r_j.vel, r_j.onc, r_i)
    end

    # communication step (i receives from j)
    c_neighbours = nearby_agents(r_i, model, r_i.com_range)

    c_i = Dict{Int, model.epistemic_state}()

    for r_j ∈ c_neighbours
        # R_i receives a communication of R_j's epistemic state
        c_i[r_j.id] = communication(r_j.epi, r_i)
    end

    println("Position [", r_i.pos, "]\nEstimation [", r_i.epi.μ, "]")
    # evolve epistemic state
    epi = epistemic_evolution(r_i.epi, y_i, p_i, c_i)
    r_i.epi = epi
    println("New estimation [", r_i.epi.μ, "]\n")

    # generate control
    u_i = control(r_i.epi)

    # evolve ontic state
    pos, vel, onc = ontic_evolution(r_i.pos, r_i.vel, r_i.onc, u_i, model.δt)
    r_i.pos = pos
    r_i.vel = vel
    r_i.onc = onc

    return
end

# ======================= Task specific functions  ============================

function ontic_evolution(pos_i, vel_i, onc_i, u_i, δt)
    f_point_mass(pos_i, vel_i, onc_i, u_i, δt)
end

function epistemic_evolution(epi_i, y_i, p_i, c_i)
    φ_average_memory(epi_i, c_i)
end

function observation(pos_i, vel_i, onc_i)
    g_perfect_observation(pos_i, vel_i, onc_i)
end

function perception(pos_j, vel_j, onc_j, r_i)
    η_perfect_perception(pos_j, vel_j, onc_j)
end

function communication(epi_j, r_i)
    λ_perfect_communication(epi_j)
end

function control(epi_i)
    h_follow_memory(epi_i)
end

# =========================== Implementations =================================

function g_perfect_observation(pos_i, vel_i, onc_i)
    pos_i, vel_i, onc_i
end

function η_perfect_perception(pos_j, vel_j, onc_j)
    pos_j, vel_j, onc_j
end

function λ_perfect_communication(epi_j)
    epi_j
end

# Gradient descent on the new estimated gathering position 
function h_follow_memory(epi_i)
    [- epi_i.μ               ;
     atan(epi_i.μ[2], epi_i.μ[1])] # The heading is not being controlled,
                                   # so we just display it according to velocity
end

function f_point_mass(pos_i, vel_i, onc_i, u_i, δt)
    D = length(pos_i)
    O = length(onc_i.x)
    @assert D == length(vel_i)
    @assert D + O == length(u_i)

    pos = pos_i
    vel = vel_i
    onc = onc_i

    vel = u_i[1:D]
    pos = pos + δt * vel
    onc.x = u_i[D+1:D+O]

    pos, vel, onc 
end

function φ_average_memory(epi_i, c_i)
    epi = epi_i
    dpos = epi.μ
#    E = size(epi.μ)[1]

    for (id_j, epi_j) ∈ c_i
        dpos = dpos - (epi.μ - epi_j.μ)
    end

    epi.μ = dpos
    epi
end


function agent_flock_step!(r_i, model)
    neighbours = nearby_agents(r_i, model, r_i.vis_range)

    σ = 1.0
    dpos = r_i.pos

    for r_j ∈ neighbours
        dpos = dpos - σ * (r_i.pos - r_j.pos)
    end

    r_i.onc.dx[1] = 0
    r_i.vel = dpos

    r_i.pos = r_i.pos + r_i.vel * model.δt
    r_i.onc.x[1] = atan(r_i.vel[2], r_i.vel[1])
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
