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


def tracked_observation_candidates(grid, robot_xy, *, anchors, safety_radius_m=.45, max_candidates=6):
    """Short safe advances facing measured targets, preserving a 2.5 m standoff.

    Anchors are image-time map detections, never simulator truth. Unknown space
    is forbidden for travel; person endpoints are excluded from sight rays.
    """
    points=np.asarray(anchors,dtype=float)
    if points.ndim!=2 or points.shape[1]!=2 or len(points)<1 or not np.isfinite(points).all():return ()
    center=points.mean(axis=0);delta=center-np.asarray(robot_xy)
    if np.linalg.norm(delta)<2.5:return ()
    heading=math.atan2(delta[1],delta[0]);clearance=grid.clearance_m()
    row,col=(math.floor(v) for v in grid.world_to_grid(*robot_xy))
    if not grid.in_bounds(row,col) or not grid.free[row,col] or clearance[row,col]<.26:return ()
    rr,cc=np.indices(grid.data.shape)
    safe=grid.free&(clearance>=safety_radius_m)
    egress=(np.hypot(rr-row,cc-col)*grid.resolution<=.30)&grid.free&(clearance>=.26)
    distances=geodesic_distance(safe|egress,(row,col),None)*grid.resolution
    pool=[]
    for r,c in np.argwhere(safe & (distances>=.5) & (distances<=1.4)):
        point=np.array(grid.grid_to_world(r,c));distance=distances[r,c]
        direction=math.atan2(point[1]-robot_xy[1],point[0]-robot_xy[0])
        angle=math.degrees(math.atan2(math.sin(direction-heading),math.cos(direction-heading)))
        if abs(angle)>55:continue
        # With a fixed forward camera, a lateral driving heading can hide the
        # very people this waypoint is meant to observe. Prefer both ends of
        # the straight approach to keep measured target centres within 20 deg.
        aligned=not any(abs(math.atan2(math.sin(math.atan2(p[1]-origin[1],p[0]-origin[0])-direction),
                             math.cos(math.atan2(p[1]-origin[1],p[0]-origin[0])-direction)))>.35
               for p in points for origin in (robot_xy,point))
        target_distances=np.linalg.norm(points-point,axis=1)
        if np.any(target_distances<2.5) or np.any(target_distances>5.):continue
        look=center-point;yaw=math.atan2(look[1],look[0])
        if any(abs(math.atan2(math.sin(math.atan2(p[1]-point[1],p[0]-point[0])-yaw),
                             math.cos(math.atan2(p[1]-point[1],p[0]-point[0])-yaw)))>.45 for p in points):continue
        # A 2-D lidar obstacle may sit below a recently observed torso.
        # Collision-free travel is mandatory; sight is a visibility hypothesis.
        visible=sum(ray_clear(grid,point,p-(p-point)/np.linalg.norm(p-point)*.45) for p in points)
        score=(.8+.2*visible/len(points))/(1.+abs(angle)/20.+distance*.15)
        pool.append((Goal(float(point[0]),float(point[1]),yaw,0,score,float(distance),1.,
                         purpose='observe_targets',look_at=tuple(map(float,center))),aligned))
    # Sparse initial scans can leave unknown gaps outside the first footprint.
    # A short known-free advance can fill them; it never counts as the mission's
    # >=0.75 m completed exploration goal. Prefer longer views once available.
    aligned=[g for g,visible in pool if visible]
    # If straight observation is blocked, only a short side step may change the
    # viewpoint. Final gaze and the observation wait still apply, with fresh
    # identity evidence required before task completion.
    pool=aligned or [g for g,_ in pool if g.geodesic_dist_m<=.65]
    longer=[g for g in pool if g.geodesic_dist_m>=.75]
    return tuple(sorted(longer or pool,key=lambda g:g.gain,reverse=True)[:max_candidates])
