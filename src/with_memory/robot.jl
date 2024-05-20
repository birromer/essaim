using DrWatson
@quickactivate "essaim_v2"

function agent_laplacian_step!(r_i, model)
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

# applies the laplacian from the task context
#function agent_step!(r_i, model)
#    # observation step
#    y_i = observation(r_i.pos, r_i.vel, r_i.x, r_i.dx)
#
#    # perception step (i observes j)
#    p_neighbours = nearby_agents(r_i, model, r_i.vis_range)
#
#    for r_j ∈ p_neighbours
#
#    end
#
#    # communication step (i receives from j)
#    c_neighbours = nearby_agents(r_i, model, r_i.com_range)
#
#    for r_j ∈ c_neighbours
#
#    end
#
#    # epistemic evolution
#    μ_i = epistemic_evolution(r_i.μ, r_i.dμ, y_i, p_i, c_i)
#    
#    σ = 1.0
#
#    dpos = r_i.pos
#
#    for r_j ∈ neighbours
#        dpos = dpos .- σ .* (r_i.pos .- r_j.pos)
#    end
#
#    r_i.x[1] = 0
#    r_i.vel = dpos
#
#    r_i.pos = r_i.pos .+ r_i.vel .* model.δt
#    r_i.x[1] = atan(r_i.vel[2], r_i.vel[1])
#
#    return
#end
#
#function ontic_evolution(x_i, u_i)
#end
#
#function epistemic_evolution(μ_i, y_i, p_i, c_i)
#end
#
#function observation(x_i)
#end
#
#function perception(x_j)
#end
#
#function communication(μ_j)
#end
#
#function control(μ_i)
#end
#
#
#
## a sim behaviour for the robots
## - they keep a low speed with 0 rotation speed
## - unless they reach further than X meters from the nearest neighbor,
## then they rotate
#function agent_test_step!(robot, model)
#    neighbor_ids = nearby_ids(robot, model, robot.vis_range)
#
#    robot.pos = robot.pos .+ robot.vel .* model.δt
#    robot.θ += robot.dθ * model.δt
#
#    robot.vel = (
#        robot.pos[1]*cos(robot.θ) - robot.pos[2]*sin(robot.θ),
#        robot.pos[1]*sin(robot.θ) + robot.pos[2]*cos(robot.θ)
#    )
#
#    for id in neighbor_ids
#        if euclidean_distance(robot.pos, model[id].pos, model) > robot.com_range
#            robot.dθ += 0.01
#            return
#        end
#    end
#
#    robot.dθ -= 0.005
#
#    return
#end
