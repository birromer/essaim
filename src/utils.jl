using DrWatson
@quickactivate "Essaim"

using StaticArrays

# Generate 2D rotation matrix
function rot2D(θ)
    @SMatrix [  cos(θ) -sin(θ) ;
                sin(θ)  cos(θ) ]
end

# Generate 3D rotation matrix
function rot3D(ϕ, θ, ψ)
    @SMatrix [  cos(θ)cos(ψ) sin(ϕ)sin(θ)cos(ψ)-cos(ϕ)sin(ψ) cos(ϕ)sin(θ)cos(ψ)+sin(ϕ)sin(ψ) ;
                cos(θ)sin(ψ) sin(ϕ)sin(θ)sin(ψ)+cos(ϕ)cos(ψ) cos(ϕ)sin(θ)sin(ψ)-sin(ϕ)cos(ψ) ;
                -sin(θ)      sin(ϕ)cos(θ)                    cos(ϕ)cos(θ)                    ]
end
