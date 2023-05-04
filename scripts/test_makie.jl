using DrWatson
@quickactivate "Essaim"

using GLMakie
using DynamicalSystems
using OrdinaryDiffEq
using DataStructures: CircularBuffer

# observable initialized with value 1
o = Observable(1)

# listener attached to observable, action happens each time
# it is updated
l1 = on(o) do val
    println("Observable now has value $val")
end

# the observable is updated with an empty index
o[] = 5

# 4 random values from 1 to 4
ox = 1:4
oy = Observable(rand(4))  # values (observable)
lw = Observable(2)  # line width (observable)

fig, ax = lines(ox, oy; linewidth=lw)
ylims!(ax,0,1)

lw[] = 50
oy[] = rand(4)

# now to test with interactions
const L1 = 1.0
const L2 = 0.9
M = 2
u0 = [π/3, 0, 3π/4, -2]
dp = Systems.double_pendulum(u0; L1, L2)

# solve diff eq with constant step
diffeq = (alg = Tsit5, adaptative = false, dt = 0.005)

integ = dp.integ

function xycoords(state)
    θ1 = state[1]
    θ2 = state[3]
    x1 = L1 * sin(θ1)
    y1 = -L1 * cos(θ1)
    x2 = x1 + L2 * sin(θ2)
    y2 = y1 - L2 * cos(θ2)
    return x1,x2,y1,y2
end

function progress_for_one_step!(integ)
    step!(integ)
    return xycoords(integ)
end

# gotta decide what is static an what is animated

# the coordinates have to be animated, so they become observables
x1, x2, y1, y2 = xycoords(u0)

# here we have two observables because they will be plotted
# differently (one a line plot, the other a scatter plot)
rod = Observable([
    Point2f(0,0),
    Point2f(x1,y1),
    Point2f(x2,y2),
])

balls = Observable([
    Point2f(x1,y1),
    Point2f(x2,y2)
])


# we also want the tail to be animated
tail = 300

# initialize a LIFO structure of size tail
traj = CircularBuffer{Point2f}(tail)
fill!(traj, Point2f(x2,y2))
traj = Observable(traj)

# now we initialize the figure
fig = Figure(); display(fig)

ax = Axis(fig[1,1])

lines!(ax, rod;
         linewidth = 4, 
         color = :purple
)

scatter!(ax, balls;
         marker = :circle,
         strokewidth = 2,
         strokecolor = :purple,
         color = :black,
         markersize = [8, 12]
)


c = to_color(:purple)  # color with r, g, b and alpha
# make the alpha change over the tail
tailcol = [RGBAf(c.r, c.g, c.b, (i/tail)^2) for i in 1:tail]
lines!(ax, traj;
       linewidth = 3,
       color = tailcol
       )

# static elements of the plot
ax.title = "double pendulum"
ax.aspect = DataAspect()
l = 1.05(L1+L2)
xlims!(ax, -l, l)
ylims!(ax, -l, 0.5l)

# now we need the animation stepping function, this progresses
# the plot for each frame
# we first progress the data
# then we update the observables according to the new data
function animstep!(integ, rod, balls, traj)
    # update data
    x1,x2,y1,y2 = progress_for_one_step!(integ)
    # update observables
    rod[] = [Point2f(0,0), Point2f(x1,y1), Point2f(x2,y2)]
    balls[] = [Point2f(x1,y1), Point2f(x2,y2)]
    push!(traj[], Point2f(x2,y2))
    # we need to update because we used the in-place update
    traj[] = traj[]
end

# test!
# in order to test it we just call the animation function
# over time with some sleep in the middle
for i ∈ 1:1000
    animstep!(integ, rod, balls, traj)
    sleep(0.005)
end

# record!

# helper function to initialize the animation
function makefigure(u0)
    dp = Systems.double_pendulum(u0; L1, L2)
    diffeq = (alg = Tsit5, adaptative = false, dt = 0.005)
    integ = dp.integ

    x1, x2, y1, y2 = xycoords(u0)
    rod = Observable([Point2f(0,0), Point2f(x1,y1), Point2f(x2,y2),])
    balls = Observable([Point2f(x1,y1), Point2f(x2,y2)])

    tail = 300
    traj = CircularBuffer{Point2f}(tail)
    fill!(traj, Point2f(x2,y2))
    traj = Observable(traj)

    fig = Figure(); display(fig)
    ax = Axis(fig[1,1])
    lines!(ax, rod;
           linewidth = 4,
           color = :purple
           )
    scatter!(ax, balls;
             marker = :circle,
             strokewidth = 2,
             strokecolor = :purple,
             color = :black,
             markersize = [8, 12]
             )

    c = to_color(:purple)
    tailcol = [RGBAf(c.r, c.g, c.b, (i/tail)^2) for i in 1:tail]
    lines!(ax, traj;
           linewidth = 3,
           color = tailcol
           )

    ax.title = "double pendulum"
    ax.aspect = DataAspect()
    l = 1.05(L1+L2)
    xlims!(ax, -l, l)
    ylims!(ax, -l, 0.5l)

    return fig, integ, rod, balls, traj
end

# to record a video we use the record function
fig, integ, rod, balls, traj = makefigure(u0)
frames = 1:200

record(fig, plotsdir("double_pendulum.mp4"), frames; framerate = 60) do i
    # this is executed for each frame i
    for j in 1:5  # each frame is stepped 5 times
        animstep!(integ, rod, balls, traj)
    end
end

