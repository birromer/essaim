using DrWatson
@quickactivate "Essaim"

include(srcdir("Essaim.jl"))

using GLMakie
using DynamicalSystems

# receive a robot and returns the edges of the polygon representing it
function triangle_poly(X::Robot, base, height)::Vector{Point2f,2}
    # base triangle
    triangle::Matrix{Float64} = [
         height/2       0
        -height/2  base/2
        -height/2 -base/2
    ]

    # first rotate p' = R * p
    c::Float64, s::Float64 = cos(X.θ), sin(X.θ)
    triangle = [
        c -s
        s  c
    ] * triangle'

    # then translate p = p .+ x
    triangle = triangle' .+ [X.x X.y]

    # repeat the first point to close the polygon
    t = vcat(triangle, triangle[1,:]')

    # create a list of points required by Makie
    poly = [Point2f(t[i,:][1],t[i,:][2]) for i in axes(t,1)]

    poly
end

function makefigure(config_0, triangle_size=(0.5,0.5))
    # triangle parameters
    base, height = triangle_size

    #

end

function animstep!(configuration, δt)

end

function draw_mission(mission, filename, fps=5, triangle_size=(0.5,0.5))

  # start animation
  anim = Animation()

  for idx_m ∈ eachindex(mission[2:end])
    plt = plot(base_plt) # clear the plot
    colors = ["blue", "red", "green"]
    alpha = range(0,1,length=idx_m+1)

    for i ∈ 1:idx_m

      N = length(mission[1].R) # number of robots

      triangles = [triangle(robot, base, height) for robot ∈ mission[i].R]
      shapes = [triangle_shape(triangle) for triangle ∈ triangles]

      for r ∈ 1:N
        plot!(Shape(shapes[r]), label="Robot {$r}",
         fillcolor = if i == idx_m colors[r] else "white" end,
         linewidth = if i == idx_m 1 else 0.3 end,
         fillalpha = alpha[i],
         aspect_ratio = :equal
         )
      end
    end

    frame(anim, plt)
  end

  # generate animation
  gif(anim, plotsdir(filename), fps=5)
end
