"""Independent closed-recording checks for a planning map with unknown borders."""
import math


def planning_map_preserves(raw, planned, padding_m):
    try:
        r,p=raw.info,planned.info
        if not 0<r.resolution or not math.isfinite(r.resolution) or not math.isfinite(padding_m) or padding_m<=0 or r.width<=0 or r.height<=0:
            return False
        cells=math.ceil(padding_m/r.resolution)
        if (raw.header.frame_id!=planned.header.frame_id or raw.header.stamp!=planned.header.stamp
            or p.resolution!=r.resolution or p.width!=r.width+2*cells or p.height!=r.height+2*cells
            or len(raw.data)!=r.width*r.height or len(planned.data)!=p.width*p.height):
            return False
        rq,pq=r.origin.orientation,p.origin.orientation
        if not all(math.isfinite(getattr(q,k)) for q in (rq,pq) for k in ('x','y','z','w')):
            return False
        if abs(sum(getattr(rq,k)**2 for k in ('x','y','z','w'))-1)>.01:return False
        if any(abs(getattr(rq,k)-getattr(pq,k))>1e-9 for k in ('x','y','z','w')):
            return False
        yaw=math.atan2(2*(rq.w*rq.z+rq.x*rq.y),1-2*(rq.y*rq.y+rq.z*rq.z))
        offset=cells*r.resolution
        expected=(r.origin.position.x-offset*(math.cos(yaw)-math.sin(yaw)),
                  r.origin.position.y-offset*(math.sin(yaw)+math.cos(yaw)),r.origin.position.z)
        if any(not math.isfinite(a) or abs(a-b)>1e-8
               for a,b in zip((p.origin.position.x,p.origin.position.y,p.origin.position.z),expected)):
            return False
        for row in range(p.height):
            line=planned.data[row*p.width:(row+1)*p.width]
            if cells<=row<cells+r.height:
                source=raw.data[(row-cells)*r.width:(row-cells+1)*r.width]
                if list(line[cells:cells+r.width])!=list(source):return False
                if any(v!=-1 for v in (*line[:cells],*line[cells+r.width:])):return False
            elif any(v!=-1 for v in line):return False
        return True
    except (AttributeError,TypeError,ValueError,OverflowError):
        return False
