using DrWatson
@quickactivate "Essaim"

using Agents
using DataStructures: CircularBuffer
using Random
using Colors

# our robots are in the dubin's car model
# cannot subtype ContinuousAgent as it's concrete
mutable struct Robot{D} <: AbstractAgent
    id::Int
    pos::NTuple{D,Float64} # pos and vel required like this for ContinuousAgent
    vel::NTuple{D,Float64} # required for using update_vel! in the model
    θ::Float64
    dθ::Float64
    vis_range::Float64  # how far can it see?
    com_range::Float64  # how far can it communicate?
    alive::Bool  # is it alive?
end

# here we initialize the model with its parameters and populate with robots
function initialize_model(;
                          N = 15,                  # number of agents
                          extent = (100.0,100.0), # size of the world
                          speed = 1.0,            # their initial velocity
                          vis_range = 25.0,        # visibility range
                          com_range = 25.0,        # communication range
                          δt = 0.001,              # time step of the model
                          history_size = 500,     # amount of saved past states
                          seed = 42               # random seed
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
        :colors => distinguishable_colors(N, [RGB(1,1,1), RGB(0,0,0)], dropseed=true),
        :seed => seed,
        :speed => speed,
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
