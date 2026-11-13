"""Conservative destination checks against a real Nav2 OccupancyGrid snapshot."""
import math


def destination_check(data,width,height,resolution,origin,goal,length,width_m,padding):
    values=[resolution,*origin,*goal,length,width_m,padding]
    if (not all(math.isfinite(float(v)) for v in values) or resolution<=0 or
            length<=0 or width_m<=0 or padding<0 or len(data)!=width*height):
        raise ValueError('Invalid grid or measured footprint')
    gx,gy,yaw=goal
    ox,oy,oa=origin
    ca,sa=math.cos(oa),math.sin(oa)
    def grid(x,y):
        dx,dy=x-ox,y-oy
        return ((ca*dx+sa*dy)/resolution,(-sa*dx+ca*dy)/resolution)
    center=grid(gx,gy)
    cx,cy=math.floor(center[0]),math.floor(center[1])
    if not (0<=cx<width and 0<=cy<height):
        return dict(passed=False,reason='target_outside_costmap')
    cost=int(data[cy*width+cx])
    if cost<0 or cost>=99:
        return dict(passed=False,reason='target_unknown_inscribed_or_lethal',center_cost=cost)
    l,w=length/2+padding,width_m/2+padding
    c,s=math.cos(yaw),math.sin(yaw)
    polygon=[grid(gx+c*x-s*y,gy+s*x+c*y) for x,y in ((l,w),(l,-w),(-l,-w),(-l,w))]
    minx,maxx=math.floor(min(p[0] for p in polygon)),math.floor(max(p[0] for p in polygon))
    miny,maxy=math.floor(min(p[1] for p in polygon)),math.floor(max(p[1] for p in polygon))
    if minx<0 or miny<0 or maxx>=width or maxy>=height:
        return dict(passed=False,reason='goal_footprint_outside_costmap',center_cost=cost)
    axes=[(1.,0.),(0.,1.)]
    for a,b in zip(polygon,polygon[1:]+polygon[:1]):
        axes.append((-(b[1]-a[1]),b[0]-a[0]))
    def intersects(x,y):
        cell=[(x,y),(x+1,y),(x+1,y+1),(x,y+1)]
        for ax,ay in axes:
            pp=[ax*px+ay*py for px,py in polygon]
            cc=[ax*px+ay*py for px,py in cell]
            if max(pp)<min(cc)-1e-9 or max(cc)<min(pp)-1e-9:
                return False
        return True
    checked=0
    for y in range(miny,maxy+1):
        for x in range(minx,maxx+1):
            if intersects(x,y):
                checked+=1
                value=int(data[y*width+x])
                if value<0 or value>=100:
                    return dict(passed=False,reason='goal_footprint_unknown_or_lethal',
                                center_cost=cost,blocked_cell=[x,y],blocked_cost=value)
    return dict(passed=True,reason='destination_snapshot_clear',center_cost=cost,
                footprint_cells_checked=checked,inflation_is_not_automatically_lethal=True)
