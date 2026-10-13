"""Local FK and STL geometry, unchanged numerically from the arm baseline."""
from pathlib import Path
import numpy as np
from arm_pkg.urdf_fk import URDFForwardKinematics

ROOT=Path(__file__).resolve().parents[1]
LINK_MESHES={'yao':'BBB.stl','jian1':'CCC.stl','jian2':'CCC2.stl',
             'wan':'DDD.stl','wan2':'EEE.stl','zhua':'FFF.stl'}
MESH_DTYPE=np.dtype([('normal','<f4',3),('vertices','<f4',(3,3)),('attr','<u2')])

def stl_vertices(path):
    data=Path(path).read_bytes()
    count=int.from_bytes(data[80:84],'little')
    if len(data)!=84+50*count:raise ValueError(f'expected binary STL: {path}')
    return np.frombuffer(data,dtype=MESH_DTYPE,count=count,offset=84)['vertices'].reshape(-1,3)

def mesh_floor_clearance(fk,angles,mesh_cache):
    mesh_dir=ROOT/'ros2_ws/src/arm_pkg/urdf/meshes'
    lows={}
    for link,filename in LINK_MESHES.items():
        if filename not in mesh_cache:mesh_cache[filename]=stl_vertices(mesh_dir/filename)
        transform,_,_=fk.forward_kinematics(angles,target_link=link)
        vertices=mesh_cache[filename]
        lows[link]=float((vertices@transform[2,:3]+transform[2,3]).min())
    return lows

class PickGeometry:
    def __init__(self,root,mapping):
        self.mapping=mapping
        self.names=[mapping.joint(i)['urdf_joint_name'] for i in range(1,7)]
        self.fk=URDFForwardKinematics(str(root/'ros2_ws/src/arm_pkg/urdf/genkiarm.urdf'))
        self.vertices={link:np.unique(stl_vertices(root/'ros2_ws/src/arm_pkg/urdf/meshes'/name),axis=0)
                       for link,name in [('wan2','EEE.stl'),('zhua','FFF.stl')]}
        self.tip={link:vertices[vertices[:,2]>=vertices[:,2].max()-.002].mean(axis=0)
                  for link,vertices in self.vertices.items()}
    def angles(self,counts):
        return np.array([self.mapping.count_to_radians(i,int(counts[i-1])) for i in range(1,7)])
    def counts(self,q):
        return [self.mapping.radians_to_count(i,float(q[i-1])) for i in range(1,7)]
    def metrics(self,q):
        angles=dict(zip(self.names,q));points={};lows={};transforms={}
        for link,vertices in self.vertices.items():
            T=self.fk.forward_kinematics(angles,target_link=link)[0]
            transforms[link]=T
            points[link]=(T@np.r_[self.tip[link],1])[:3]
            lows[link]=float(np.min(vertices@T[2,:3]+T[2,3]))
        return (points['wan2']+points['zhua'])/2,points,lows,transforms
