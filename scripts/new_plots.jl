using DrWatson
@quickactivate "Essaim"

include(srcdir("Robot.jl"))

using Agents
using GLMakie
using GraphMakie
using Graphs
using InteractiveDynamics
using DataStructures: CircularBuffer
using Random
using Makie.Colors

# base robot, center (0,0), facing east (0rad), with length 1 and base 0.5
const robot_poly = Polygon(Point2f[(0.5, 0.), (-0.5, 0.25), (-0.5,-0.25), (0.5, 0.)])

# helper functions for plotting
robot_status(robot::Robot) = robot.alive == true ? :darkgreen : :red

function visibility_graph(model)
#    neighbours = [nearby_ids_exact(r, model, r.vis_range) for r in allagents(model)]

    g = SimpleGraph(nagents(model))
    for (id1,r1) in enumerate(allagents(model))
        for id2 in nearby_ids_exact(r1, model, r1.vis_range)
            if id1 != id2
                add_edge!(g, id1, id2)
            end
        end
    end

    g
end

# for testing
model = initialize_model(; history_size = 500, δt = 0.01, N = 10, com_range = 10., vis_range = 10., seed = 1)

# initialize figure for the animation
function make_figure(model; interactive=true)
    N = nagents(model)

    # first we generate the markers for the current state of the robots
    rob_pos = Observable([Point2f(r.pos[1], r.pos[2]) for r in allagents(model)])

    # generate the tail with all potions in the same place initially
    rob_hist = [Observable(CircularBuffer{Point2f}(model.history_size)) for _ in 1:N]
    for (id, buf) in enumerate(rob_hist)
        fill!(buf[], rob_pos[][id])
    end

    # make the properties that change according to the robot observables
    rob_status = Observable(robot_status.(allagents(model)))  # the countour color
    rob_rot = Observable([r.θ for r in allagents(model)])

    # make the togglable properties of the plot observables too
    rob_vis = [(Observable(r.pos), Observable(r.vis_range)) for r in allagents(model)]
    rob_com = [(Observable(r.pos), Observable(r.com_range)) for r in allagents(model)]

    # different color for each robot
    traj_color = [Observable([RGBAf(model.colors[r].r, model.colors[r].g, model.colors[r].b, (i/model.history_size)^2) for i in 1:model.history_size]) for r in 1:N]

    # make the visibility graph to be displayed in the side
    vis_graph = Observable(visibility_graph(model))

    plot_dict = Dict(
        :pos => rob_pos,
        :hist => rob_hist,
        :status => rob_status,
        :rot => rob_rot,
        :vis => rob_vis,
        :com => rob_com,
        :graph => vis_graph,
        :traj_color => traj_color
    )

    # create a new figure
    fig = Figure(;resolution=(1000,700)); display(fig)

    ax_main  = Axis(
        fig[1:16,1:4],
        title="Dynamic continuous average consensus",
        aspect = DataAspect(),
        limits = (-model.space.extent[1],model.space.extent[1], -model.space.extent[2], model.space.extent[2]),
        xminorticksvisible = true,
        xminorticks = IntervalsBetween(5),
        yminorticksvisible = true,
        yminorticks = IntervalsBetween(5)
    )

    ax_graph = Axis(
        fig[(interactive ? 9 : 2):16 ,5:6],
        title="Visibility graph",
        tellwidth=false
    )
    hidedecorations!(ax_graph)
    ax_graph.aspect = DataAspect()

    if interactive
        # add the control and parameter input options
        fig[2:6, 5:6] = control_grid = GridLayout(tellwidth = false, tellheight = false)
        texts = ["Number of agents:", "Time step (s):", "Visibility radius (m):", "Seed:"]
        placeholders = [Observable(string(nagents(model))), Observable(string(model.δt)), Observable(string(model[1].vis_range)), Observable(string(model.seed))]
        control_grid[1:4,1] = [Label(fig, text=t; justification = :left, halign = :left) for t in texts]
        textboxes = control_grid[1:4,2] = [Textbox(fig, placeholder=p; halign = :right, textpadding = (5,5,5,5), width=80) for p in placeholders]

        fig[8, 5:6] = button_grid = GridLayout(tellwidth = false)
        buttonlabels = ["Run/Stop", "Step", "Reset"]
        buttons = button_grid[1, 1:3] = [Button(fig, label = l) for l in buttonlabels]
    else
        textboxes = buttons = nothing
    end

    # now add the data
    scatter!(ax_main, plot_dict[:pos];
             marker = robot_poly,
             rotations = plot_dict[:rot],
             strokewidth = 3,
             strokecolor = plot_dict[:status], # contour color is status (alive/dead)
             color = model.colors,     # unique color, same for trail
             markersize = 11
             )

    # plot trajectories
    for id in allids(model)
        lines!(ax_main, plot_dict[:hist][id];
               linewidth = 3,
               color = traj_color[id]
               )
    end

    # plot visibility range
    for id in allids(model)
        println("Id vis: ", id, " at pos", plot_dict[:vis][id][1][], " and radius ", plot_dict[:vis][id][2][])
        arc!(ax_main, plot_dict[:vis][id][1], plot_dict[:vis][id][2], 0, 2π;
             linestyle = :dash,
             linewidth = 1,
             color = model.colors[id]
             )
    end

    # generate visibility graph with id colors
    graphplot!(ax_graph, plot_dict[:graph], node_color=model.colors)

    return fig, plot_dict, buttons, textboxes
end


# this function updates the data in the model using the robot step then updates
# the observables (positions, trajectory and markers) related to the plot
#function animation_step!(model, agent_step!, rob_pos, rob_hist, rob_status, rob_rot, vis_graph)
function animation_step!(model, agent_step!, plot_dict)
    # update robot positions
    step!(model, agent_step!)

    # update positions
    plot_dict[:pos][] = [Point2f(r.pos[1], r.pos[2]) for r in allagents(model)]

    # update trajectories
    for (id, buf) in enumerate(plot_dict[:hist])
        push!(buf[], plot_dict[:pos][][id])
        buf[] = buf[]
    end

    # update the markers orientation and color
    plot_dict[:status][] = robot_status.(allagents(model))
    plot_dict[:rot][] = [r.θ for r in allagents(model)]

    # update the visibility and communication ranges
    for (id,r) in enumerate(allagents(model))
        plot_dict[:vis][id][1][] = r.pos
        plot_dict[:vis][id][2][] = r.vis_range

        plot_dict[:com][id][1][] = r.pos
        plot_dict[:com][id][2][] = r.com_range
    end

    rob_vis = [(r.pos, r.vis_range) for r in allagents(model)]
    rob_com = [(Observable(r.pos), Observable(r.com_range)) for r in allagents(model)]

    # update visibility graph
    plot_dict[:graph][] = visibility_graph(model)

    return
end

function run_animation!(model, agent_step!; n_steps)
    fig, plot_dict, buttons, textboxes = make_figure(model, interactive=false)

    for _ ∈ 1:n_steps
        animation_step!(model, agent_step!, plot_dict)
        sleep(model.δt)
    end
end

function make_animation!(model, agent_step!; n_frames, steps_per_frame=3)
    frames = 1:n_frames

    fig, plot_dict, buttons, textboxes = make_figure(model, interactive=false)

    filename = string("essaim_A", nagents(model), "_T", model.δt, "_V", model[1].vis_range, "_S", model.seed, "__f", n_frames, "__sf", steps_per_frame, ".mp4")

    record(fig, plotsdir(filename), frames; framerate = 60) do i
        for j in 1:steps_per_frame  # each frame is stepped n times
            animation_step!(model, agent_step!, plot_dict)
        end
    end
end

function restart_plot(fig, model_obs, plot_dict_obs)

    # update the observables that don't need to allocate more space
    plot_dict[][:pos][] = [Point2f(r.pos[1], r.pos[2]) for r in allagents(model[])]
    plot_dict[][:status][] = robot_status.(allagents(model[]))
    plot_dict[][:rot][] = [r.θ for r in allagents(model[])]
    plot_dict[][:graph][] = visibility_graph(model[])

    oldN = size(plot_dict[][:hist],1)
    N = nagents(model[])

    ax_main = fig.content[1]

#    model[oldN+1:N]

    # if we increse the number of agents, we have to create the observables
    # for the visibility radius and the trajectory, then add to the existing plot
    if N > oldN

        # the old agents must simply be updated
        for id ∈ 1:oldN
            fill!(plot_dict[][:hist][id][], plot_dict[][:pos][][3])
            plot_dict[][:hist][id][] = plot_dict[][:hist][id][]

            plot_dict[][:vis][id][1][] = model[][id].pos
            plot_dict[][:vis][id][2][] = model[][id].vis_range

            plot_dict[][:com][id][1][] = model[][id].pos
            plot_dict[][:com][id][2][] = model[][id].com_range
        end

        # the new agents have to have their memory allocated
        for id ∈ oldN+1:N
            id = 3
            id = 4
            push!(plot_dict[][:hist], Observable(CircularBuffer{Point2f}(model[].history_size)))
            fill!(plot_dict[][:hist][id][], plot_dict[][:pos][][id])
            plot_dict[][:hist][id][] = plot_dict[][:hist][id][]

            push!(plot_dict[][:traj_color], Observable([RGBAf(model[].colors[id].r, model[].colors[id].g, model[].colors[id].b, (i/model[].history_size)^2) for i in 1:model[].history_size]))


            lines!(ax_main, plot_dict[][:hist][id];
               linewidth = 3,
               color = plot_dict[][:traj_color][id]
            )

            push!(plot_dict[][:vis], (Observable(model[][id].pos), Observable(model[][id].vis_range)))
            push!(plot_dict[][:com], (Observable(model[][id].pos), Observable(model[][id].com_range)))

            arc!(ax_main, plot_dict[][:vis][id][1], plot_dict[][:vis][id][2], 0, 2π;
               linestyle = :dash,
               linewidth = 1,
               color = model[].colors[id]
            )
        end
    else # if we have less agents, we have to remove the extra plots
#        for id ∈ N+1:oldN
#            delete!(ax_main[id*2+1]) # delete
#        end
    end

#    # TODO: check if the plot updates with this, it probably only has the original indexes there
#    for (id, r) in enumerate(allagents(model))
#        fill!(plot_dict["hist"][id][], plot_dict["pos"][][id])
#        plot_dict["hist"][id][] = plot_dict["hist"][id][]
#
#        lines!(ax_main, plot_dict["hist"][id];
#               linewidth = 3,
#               color = traj_color[id]
#               )
#    end

    # plot visibility range
#    for id in allids(model)
#        arc!(ax_main, plot_dict["vis"][id][1], plot_dict["vis"][id][2], 0, 2π;
#             linestyle = :dash,
#             linewidth = 1,
#             color = model.colors[id]
#             )
#    end
#
##    rob_hist = [Observable(CircularBuffer{Point2f}(model.history_size)) for _ in 1:N]
##    for (id, buf) in enumerate(rob_hist)
##        fill!(buf[], rob_pos[][id])
##    end
#
#
#    # TODO: increase size of visibility and communication lists
#    for (id,r) in enumerate(allagents(model))
#        plot_dict["vis"][id][1][] = r.pos
#        plot_dict["vis"][id][2][] = r.vis_range
#
#        plot_dict["com"][id][1][] = r.pos
#        plot_dict["com"][id][2][] = r.com_range
#    end
#
#    for id in allids(model)
#        arc!(ax_main, plot_dict["vis"][id][1], plot_dict["vis"][id][2], 0, 2π;
#             linestyle = :dash,
#             linewidth = 1,
#             color = model.colors[id]
#             )
#    end

#    rob_com = [(Observable(r.pos), Observable(r.com_range)) for r in allagents(model)]
     return fig
end


function run_simulator!(model, agent_step!)
    fig, plot_dict, buttons, textboxes = make_figure(model; interactive=true)

    # make everything used by the listeners also observable
    model = Observable(model)
    plot_dict = Observable(plot_dict)

    # TODO: Remove
    agent_step! = agent_laplacian_step!

    # get starting values for the model
    seed = Observable(model[].seed)
    nb_agents = Observable(nagents(model[]))
    vis_range = Observable(model[][1].vis_range)
    dt = Observable(model[].δt)

    # deconstruct to access buttons and textboxes
    b_run, b_step, b_reset = buttons
    tb_agents, tb_dt, tb_vis, tb_seed = textboxes

    # this is a simple flag for the information to be running or not
    is_running = Observable(false)

    # what happens when we click on the button:
    # (1) we flip the running flag
    on(b_run.clicks) do clicks
        is_running[] = !is_running[]
    end

    # (2) we make it execute the animation step
    on(b_run.clicks) do clicks
        println("Pressed RUN")
        @async while is_running[]       # while the running flag is true
            isopen(fig.scene) || break  # and the simulation is still open
            animation_step!(model[], agent_step!, plot_dict[])
            sleep(model[].δt)

            model[] = model[]
            plot_dict[] = plot_dict[]
        end
    end

    # the step button only advances one cycle
    on(b_step.clicks) do clicks
        println("Pressed STEP")
        if !is_running[]
            animation_step!(model[], agent_step!, plot_dict[])
            model[] = model[]
            plot_dict[] = plot_dict[]
        end
    end

    #TODO add listener to update textboxes, no need to do it in the animation function
    on(tb_agents.stored_string) do s
        println("received new number of agents ", s)
        nb_agents[] = parse(Int64, tb_agents.stored_string[])
    end

    on(tb_dt.stored_string) do s
        println("received new δt ", s)
        dt[] = parse(Float64, tb_dt.stored_string[])
    end

    on(tb_seed.stored_string) do s
        println("received new seed ", s)
        seed[] = parse(Int64, tb_seed.stored_string[])
    end

    on(tb_vis.stored_string) do s
        println("received new visibility range ", s)
        vis_range[] = parse(Float64, tb_vis.stored_string[])
    end

    # the reset button starts a new model with the parameters and relaunches the figure
    on(b_reset.clicks) do clicks
        println("Pressed RESET")
        model[] = initialize_model(; seed=seed[], δt=dt[], N=nb_agents[], com_range=vis_range[], vis_range=vis_range[],
                                   history_size=model[].history_size, extent=model[].space.extent, speed=1.0)

#        fig, plot_dict, buttons, textboxes = make_figure(model[]; interactive=true)

       fig = restart_plot(fig, model, plot_dict)
    end
end

# example usage
#
# r1 = Robot(1, (1.,1.), (0.1, 0.1), 0., 0.1, 5., 5., true)
# robot_marker(r1)
# robot_color(r1)
# model = initialize_model(;extent=(10.,10.))
# agent_simple_step!(r1, model)

model = initialize_model(; seed = 1, δt = 0.001, N = 2, com_range = 25., vis_range = 50., history_size = 500, extent=(300.,300.), speed=1.0)

# run_animation!(model, agent_simple_step!; n_steps=1000)

run_animation!(model, agent_laplacian_step!; n_steps=2500)

make_animation!(model, agent_laplacian_step!; n_frames=1000, steps_per_frame=1)

run_simulator!(model, agent_laplacian_step!)