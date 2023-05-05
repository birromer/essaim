using DrWatson
@quickactivate "Essaim"

using Agents
using InteractiveDynamics
#using CairoMakie
using GLMakie


# first we have to choose in which space the agents live, in this case it's a
# discrete grid of size 10x10
space = GridSpace((10,10))

# then we define the agent type, which has to be a subtype of AbstractAgent
mutable struct Schelling <: AbstractAgent
    id::Int
    pos::Tuple{Int,Int}
    group::Int
    happy::Bool
end

# now we have the properties of the model in a dictionary
properties = Dict(
    :min_to_be_happy => 3
)

# we can now create the model
# model = AgentBasedModel(Schelling, space; properties=properties)

# agents are activate at random in each step
scheduler = Schedulers.randomly

model = AgentBasedModel(Schelling, space;
                        properties=properties,
                        scheduler=scheduler
                        )

function initialize(;
                          N = 320, # number of agents
                          M = 20,  # size of the grid
                          min_to_be_happy = 3)

    # initialize model
    space = GridSpace((M,M))
    scheduler = Schedulers.randomly
    properties = Dict(:min_to_be_happy => min_to_be_happy)

    model = AgentBasedModel(Schelling, space;
                            properties=properties,
                            scheduler=scheduler)

    # now we add the agents
    for n ∈ 1:N
        # half of the agents are of each group and all start unhappy
        agent = Schelling(n, (1,1), n < N/2 ? 1 : 2, false)
        # add agent at the model at a random empty place, empty as problem def. 
        add_agent_single!(agent,model)
    end

    return model
end

# we need to define how the agents behave
# it can be either agent or model steps, we'll have for the agent only
function agent_step!(agent, model)
    # if the agent is happy, do nothing
    agent.happy && return

    # now we have to count the agents around that are the same
    nearby_same = 0

    # this function gives an iterator over who is nearby
    for neighbour in nearby_agents(agent, model)
        if agent.group == neighbour.group
            nearby_same += 1
        end
    end

    if nearby_same ≥ model.min_to_be_happy
        agent.happy = true
    else
        # moves to random new position
        move_agent_single!(agent, model)
    end
    return
end

# this evolves the model in one step,
# it schedules all the agents and applies the agent_step for each of them
step!(model, agent_step!)
# step!(model, agent_step!, 3) # same but 3 steps
# # we can have both a step for agent and other for the model
# step!(model, agent_step!, dummystep, 3)

# termination criteria, given a model and some thing to be checked
# in this case it only checks if the step is 3, but it can use anything
# from on the model
t(model, s) = s == 3

model = initialize()

# we execute the model until the termination function returns true
step!(model, agent_step!, dummystep, t) 

# now it is possible to visualize and animate the system
fig, _ = abm_plot(model)
display(fig)

groupcolor(agent) = agent.group == 1 ? :blue : :orange
groupmarker(agent) = agent.group == 1 ? :circle : :rect

fig = Figure()
ax = Axis(fig[1,1])
abmplot!(ax, model; ac = groupcolor, am = groupmarker)

model = initialize()
abmplot!(ax, model;
         agent_step! = agent_step!,
         ac = groupcolor,
         am = groupmarker,
         as = 12
         )


# finally it also possible to collect data related to the model

# for this we have to specify which data to collect
x(agent) = agent.pos[1]  # to collect for a single agent
adata = [:group, :happy, x]

# run has the same behaviour of step, but also collects data
# it returns the agent data frame and the model data fram
# it collects data before stepping the moel
adf, _ = run!(model, agent_step!, dummystep, 3; adata)


using Statistics: mean
# if we want to colelct from the entire group of agents
adata = [
    (:happy, sum),  # we want the sum of the happiness
    (x, mean)       # and the mean of position
]


adf, _ = run!(model, agent_step!, dummystep, 3; adata)
