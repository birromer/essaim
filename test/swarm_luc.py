from roblib import *


def f(x,u):
    θ,v=x[2,0],x[3,0]
    return array([[v*cos(θ)],[v*sin(θ)],[u[0,0]],[u[1,0]]])

def xi(X,i):  return X[:,i%m].flatten().reshape(4,1)


def gj(X,i,j):
    Xi=xi(X,i)
    Xi_=xi(X,j)
    p,θ,pj,θj=Xi[[0,1]],Xi[2,0],Xi_[[0,1]],Xi_[2,0]
    Rθ=array([[cos(θ),sin(θ)],[-sin(θ),cos(θ)]])
    dθ=θj-θ
    dp=Rθ@(pj-p)
    return dp,dθ

def control(X,i) :
    v=X[3,i]
    w,dθ=gj(X,i,i+1)
    w=w-2*array([[cos(dθ)],[sin(dθ)]])
    for j in range(m):
        if (j!=i):
            dp,_=gj(X,i,j)
            w=w-10*dp/(norm(dp)**3)
    Xi = xi(X, i)
    Xi_ = xi(X,i+1)
    plot([Xi[0,0],Xi_[0,0]],[Xi[1,0],Xi_[1,0]], "green", linewidth=1)
    return array([[3*sawtooth(angle(w))],[norm(w)-v]])


m=6
np.random.seed(0)
dt = 0.05
ax=init_figure(-15,15,-15,15)
X=5*randn(4,m)
while True:
    clear(ax)
    X_=X.copy()
    for i in range(m):
        x=xi(X_,i)
        draw_tank(x[[0,1,2]],'red',0.25)
        u=control(X_,i)
        X[:,i]=(x+dt*f(x,u)).flatten()
    pause(0.01)
