using GLMakie

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

fig, ax = lines(ox, oy, linediwidth=lw)
ylims!(ax,0,1)

lw[] = 50
oy[] = rand(4)

# now to test with interactions
