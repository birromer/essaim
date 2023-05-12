using DrWatson
@quickactivate "Essaim"

using Agents
using GLMakie
using DataStructures: CircularBuffer
using Random

# our robots are in the dubin's car model
# cannot subtype ContinuousAgent as it's concrete
mutable struct Robot{D} <: AbstractAgent
    id::Int
    pos::NTuple{D,Float64} # pos and vel required like this for ContinuousAgent
    vel::NTuple{D,Float64} # required for using update_vel! in the model
    θ::Float64
    θ̇::Float64
    vis_range::Float64  # how far can it see?
    com_range::Float64  # how far can it communicate?
    alive::Bool  # is it alive?
end

# here we initialize the model with its parameters and populate with robots
function initialize_model(;
                          N = 5,     # number of agents
                          extent = (20.0,20.0),  # size of the world
                          speed = 1.0,      # their initial velocity
                          vis_range = 5.0,  # visibility range
                          com_range = 5.0,  # communication range
                          δt = 0.05,         # time step of the model
                          seed = 42         # random seed
)
    # initialize model
    space = ContinuousSpace(extent)
    rng = Random.MersenneTwister(seed)
    scheduler = Schedulers.fastest
    D = length(extent)
    properties = Dict(:δt => δt)

    model = AgentBasedModel(Robot{D}, space;
                rng = rng,
                scheduler = scheduler,
                properties = properties
                )

    # now we add the agents
    for n ∈ 1:N
        # get random position and heading
        pos = Tuple(rand(model.rng, 2) .* extent)
        heading = rand(model.rng) * 2π
        vel = (speed, speed)

        # initialize the agents with argument values and no heading change
        agent = Robot{D}(n, pos, vel, heading, 0.0, vis_range, com_range, true)
        add_agent!(agent, model)
    end

    return model
end

function initialize_figure(model)
    # initialize pos_history if not exists
    if !haskey(model.properties, :pos_history)
        model.properties[:pos_history] = Dict(i => [] for i in 1:length(model.agents))
    end

    fig = Figure()
    ax = Axis(fig[1,1], limits=FRect((0,0), model.space.extent))

    # create an observable for each robot's current position
    current_positions = Observable([a.pos for a in allagents(model)])

    # create an observable for the history of each robot's position
    position_history = Observable([model.properties[:pos_history][a.id] for a in allagents(model)])

    # draw the robots at their current positions
    robot_poly = map(current_positions) do positions
        [poly!(ax, [pos[1], pos[2], pos[1]], [pos[2], pos[1], pos[2]], color=:red) for pos in positions]
    end

    # draw the historical positions
    history_poly = map(position_history) do histories
        polys = []
        for history in histories
            alpha_values = range(1, stop=0, length=length(history))
            for (pos, alpha) in zip(history, alpha_values)
                push!(polys, poly!(ax, [pos[1], pos[2], pos[1]], [pos[2], pos[1], pos[2]], color=RGBA(0,0,0,alpha)))
            end
        end
        polys
    end

    # return the figure and the observables
    return fig, current_positions, position_history, robot_poly, history_poly
end

function agent_step!(agent, model)
    # This is a placeholder function. Replace with your actual agent_step function.
    # Here, I'll just randomly change the agent's position as an example.
    agent.pos += model.δt * agent.vel
    agent.vel = rotate(agent.vel, agent.θ̇ * model.δt)
    agent.θ += agent.θ̇ * model.δt
end

function animation_step!(model, agent_step!, current_positions, position_history, robot_poly, history_poly)
    # run the agent_step function on each agent
    for agent in allagents(model)
        agent_step!(agent, model)

        # update the position history
        push!(model.properties[:pos_history][agent.id], agent.pos)
        if length(model.properties[:pos_history][agent.id]) > 150
            popfirst!(model.properties[:pos_history][agent.id])
        end
    end

    # update the observables
    current_positions[] = [a.pos for a in allagents(model)]
    position_history[] = [model.properties[:pos_history][a.id] for a in allagents(model)]

    # remove old polygons
    for poly in robot_poly
        delete!(ax.scene, poly)
    end
    for poly in history_poly
        delete!(ax.scene, poly)
    end

    # create new polygons for the current positions
    robot_poly = map(current_positions) do positions
        [poly!(ax, [pos[1], pos[2], pos[1]], [pos[2], pos[1], pos[2]], color=:red) for pos in positions]
    end

    # create new polygons for the historical positions
    history_poly = map(position_history) do histories
        polys = []
        for history in histories
            alpha_values = range(1, stop=0, length=length(history))
            for (pos, alpha) in zip(history, alpha_values)
                push!(polys, poly!(ax, [pos[1], pos[2], pos[1]], [pos[2], pos[1], pos[2]], color=RGBA(0,0,0,alpha)))
            end
        end
        polys
    end
end


model = initialize_model()
figure, cur_pos, past_pos, cur_poly, past_poly = initialize_figure(model)
animation_step!(model, agent_step!, cur_pos, past_pos, cur_poly, past_poly)
