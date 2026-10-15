"""Map-only viewpoints that expose unknown space behind compact occluders.

These are visibility hypotheses, never person positions. Motion is restricted
through clearance-inflated known free space. Unknown cells are allowed only
along predicted camera rays, never along a driving route.
"""
import math
import cv2
import numpy as np
from .explorer import Goal
from .geodesic import geodesic_distance
from .grid import GridMap


def ray_clear(grid, start, end):
    length=math.dist(start,end)
    count=max(2,int(math.ceil(length/grid.resolution*2)))
    points=np.linspace(start,end,count)
    rows=np.floor((points[:,1]-grid.origin[1])/grid.resolution).astype(int)
    cols=np.floor((points[:,0]-grid.origin[0])/grid.resolution).astype(int)
    if np.any(rows<0) or np.any(cols<0) or np.any(rows>=grid.height) or np.any(cols>=grid.width):return False
    return not bool(grid.occupied[rows,cols].any())


def observation_candidates(grid, robot_xy, *, anchors=(), visited=(), safety_radius_m=.45, max_candidates=6):
    clearance=grid.clearance_m()
    safe=grid.traversable & (clearance>=safety_radius_m)
    row,col=(math.floor(v) for v in grid.world_to_grid(*robot_xy))
    if not grid.in_bounds(row,col) or clearance[row,col]<.26:return ()
    # Permit egress from the current footprint only. Everywhere beyond 0.30 m
    # still requires the full 0.45 m clearance, including every viewpoint.
    rr,cc=np.indices(grid.data.shape)
    egress=(np.hypot(rr-row,cc-col)*grid.resolution<=.30)&grid.free&(clearance>=.26)
    routes=safe|egress
    distance=geodesic_distance(routes,(row,col),None)*grid.resolution
    # Allow for sparse laser endpoints and SLAM edge error when predicting vision.
    margin=max(1,int(math.ceil(.20/grid.resolution)))
    occupied=cv2.dilate(grid.occupied.astype(np.uint8),np.ones((2*margin+1,2*margin+1),np.uint8))
    vision=GridMap(np.where(occupied,100,grid.data),grid.resolution,grid.origin)
    count,labels,stats,_=cv2.connectedComponentsWithStats(grid.occupied.astype(np.uint8),8)
    pool=[]
    for label in range(1,count):
        _,_,width,height,area=stats[label]
        major=max(width,height)*grid.resolution;minor=min(width,height)*grid.resolution
        if not .55<=major<=2.5 or minor>.5 or area<3:continue
        rc=np.argwhere(labels==label)
        center=np.array(grid.grid_to_world(rc[:,0].mean(),rc[:,1].mean()))
        if not .8<math.dist(center,robot_xy)<5.:continue
        if anchors and not any(.6<math.dist(center,a[:2])<3. for a in anchors):continue
        forward=center-np.asarray(robot_xy);forward/=np.linalg.norm(forward)
        axis=np.array([1.,0.]) if width>height else np.array([0.,1.])
        shadows=[center+forward*depth+axis*offset for depth in (.65,.95,1.25) for offset in (-.25,0.,.25)]
        shadows=[p for p in shadows if grid.in_bounds(*(math.floor(v) for v in grid.world_to_grid(*p)))
                 and grid.unknown[tuple(math.floor(v) for v in grid.world_to_grid(*p))]
                 and not ray_clear(vision,robot_xy,p)]
        if not shadows:continue
        for radius in (2.,2.6,3.2):
            for angle in (-85,-65,-55,-45,45,55,65,85):
                theta=math.radians(angle)
                rotation=np.array([[math.cos(theta),-math.sin(theta)],[math.sin(theta),math.cos(theta)]])
                point=center-rotation@forward*radius
                r,c=(math.floor(v) for v in grid.world_to_grid(*point))
                if not grid.in_bounds(r,c) or not safe[r,c] or distance[r,c]<.75:continue
                point=np.array(grid.grid_to_world(r,c))
                if any(math.dist(point,p[:2])<.75 for p in visited):continue
                visible=[p for p in shadows if math.dist(point,p)>=2.8 and ray_clear(vision,point,p)]
                if not visible:continue
                mean=np.mean(visible,axis=0)
                look=min(visible,key=lambda p:np.linalg.norm(p-mean))
                gain=len(visible)/len(shadows)
                score=gain/(1.+.15*distance[r,c])
                pool.append(Goal(float(point[0]),float(point[1]),math.atan2(look[1]-point[1],look[0]-point[0]),
                    label,score,float(distance[r,c]),gain,purpose='observe_occlusion',
                    look_at=tuple(map(float,look)),occluder=tuple(map(float,center))))
    selected=[]
    for goal in sorted(pool,key=lambda g:g.gain,reverse=True):
        if all(math.hypot(goal.x-g.x,goal.y-g.y)>=.6 for g in selected):selected.append(goal)
        if len(selected)>=max_candidates:break
    return tuple(selected)
