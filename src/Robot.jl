using DrWatson
@quickactivate "Essaim"

using Agents
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
                          N = 5,                 # number of agents
                          extent = (100.0,100.0),  # size of the world
                          speed = 1.0,           # their initial velocity
                          vis_range = 5.0,       # visibility range
                          com_range = 5.0,       # communication range
                          δt = 0.01,             # time step of the model
                          history_size = 300,     # amount of saved past states
                          seed = 42              # random seed
)
    # initialize model
    space = ContinuousSpace(extent)  # 2D euclidean space
    rng = Random.MersenneTwister(seed)  # reproducibility
    scheduler = Schedulers.fastest  # they are executed semi-synchronously,
                                    # in order of their indexing
    D = length(extent)  # number of dimensions
    properties = Dict(  # save the time step in the model
        :δt => δt,
        :history_size => history_size,
        :colors => distinguishable_colors(N),
        :seed => seed
    )

    model = AgentBasedModel(Robot{D}, space;
                rng = rng,
                scheduler = scheduler,
                properties = properties
    )

    # now we add the agents
    for n ∈ 1:N
        # get random position and heading
        pos = Tuple(rand(model.rng, 2)) .* (extent./2) .+ (extent./4)
        heading = rand(model.rng) * 2π
        vel = speed.* (
            pos[1]*cos(heading) - pos[2]*sin(heading),
            pos[1]*sin(heading) + pos[2]*cos(heading)
        )

        # initialize the agents with argument values and no heading change
        agent = Robot{D}(n, pos, vel, heading, 0.0, vis_range, com_range, true)
        add_agent!(agent, model)
    end

    return model
end

# we define how each robot behaves, at
function agent_laplacian_step!(r1, model)
    neighbours = nearby_agents(r1, model, r1.vis_range)
    σ = 0.2

    dpos = r1.pos
    for r2 ∈ neighbours
        dpos = dpos .- σ .* (r1.pos .- r2.pos)
    end

    r1.θ̇ = 0
    r1.vel = dpos

    r1.pos = r1.pos .+ r1.vel .* model.δt
    r1.θ = atan(r1.vel[2], r1.vel[1])

    return
end

# a sim behaviour for the robots
# - they keep a low speed with 0 rotation speed
# - unless they reach further than X meters from the nearest neighbor,
# then they rotate
function agent_simple_step!(robot, model)
    neighbor_ids = nearby_ids(robot, model, robot.vis_range)

    robot.pos = robot.pos .+ robot.vel .* model.δt
    robot.θ += robot.θ̇ * model.δt

    robot.vel = (
        robot.pos[1]*cos(robot.θ) - robot.pos[2]*sin(robot.θ),
        robot.pos[1]*sin(robot.θ) + robot.pos[2]*cos(robot.θ)
    )

    for id in neighbor_ids
        if euclidean_distance(robot.pos, model[id].pos, model) > robot.com_range
            robot.θ̇ += 0.01
            return
        end
    end

    robot.θ̇ -= 0.005

    return
end

