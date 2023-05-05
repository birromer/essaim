using DrWatson
@quickactivate "Essaim"

using Agents
using GLMakie
using InteractiveDynamics
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
                          seed = 42         # random seed
)
    # initialize model
    space = ContinuousSpace(extent)
    rng = Random.MersenneTwister(seed)
    scheduler = Schedulers.fastest
    D = length(extent)

    model = AgentBasedModel(Robot{D}, space;
                rng = rng,
                scheduler = scheduler,
                properties = nothing
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

model = initialize_model()

# we define how each robot behaves, at
function agent_laplacian_step!(robot, model)

end
