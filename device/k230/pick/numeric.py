"""Small deterministic linear algebra, suitable for CanMV MicroPython."""
import math


def dot(a,b):return sum(x*y for x,y in zip(a,b))
def cross(a,b):return [a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]]
def norm(a):return math.sqrt(dot(a,a))
def unit(a):
    n=norm(a)
    if n<1e-14:raise ValueError('Degenerate vector')
    return [v/n for v in a]
def transpose(a):return [list(row) for row in zip(*a)]
def matmul(a,b):return [[dot(row,col) for col in zip(*b)] for row in a]
def matvec(a,v):return [dot(row,v) for row in a]
def identity(n):return [[float(i==j) for j in range(n)] for i in range(n)]
def solve(a,b):
    n=len(b);m=[list(row)+[float(v)] for row,v in zip(a,b)]
    for k in range(n):
        p=max(range(k,n),key=lambda i:abs(m[i][k]))
        if abs(m[p][k])<1e-15:raise ValueError('Singular system')
        m[k],m[p]=m[p],m[k];scale=m[k][k]
        for j in range(k,n+1):m[k][j]/=scale
        for i in range(n):
            if i==k:continue
            scale=m[i][k]
            for j in range(k,n+1):m[i][j]-=scale*m[k][j]
    return [row[-1] for row in m]
def least_squares(a,b):
    at=transpose(a)
    return solve(matmul(at,a),matvec(at,b))
def rotation(axis,angle):
    x,y,z=axis;c=math.cos(angle);s=math.sin(angle);v=1-c
    return [[c+x*x*v,x*y*v-z*s,x*z*v+y*s],
            [y*x*v+z*s,c+y*y*v,y*z*v-x*s],
            [z*x*v-y*s,z*y*v+x*s,c+z*z*v]]
def rigid(r,t):return [list(row)+[t[i]] for i,row in enumerate(r)]+[[0.,0.,0.,1.]]
def transform(t,v):return [dot(row[:3],v)+row[3] for row in t[:3]]
