import carla
from shapely.geometry import Polygon
import numpy as np
import math

def get_distance(v1, v2):
    dist = math.sqrt( (v1[0] - v2[0])**2 + (v1[1] - v2[1])**2 )
    return dist


'''
def is_between(v1, v2, v3):
    r = get_distance(v2,v1) + get_distance(v3,v1) == get_distance(v2,v3)
    return r
'''


def check_collision(rect0, rect1):
    p1 = Polygon(tuple(map(tuple, rect0)))   
    p2 = Polygon(tuple(map(tuple, rect1)))   
    collision = (p1.intersection(p2).area > 0.0)
    return collision


def get_dist_vert_vert(rect0, rect1):
    perm = [[x,y] for x in range(0, np.shape(rect0)[0]) for y in range(np.shape(rect1)[0])]
    # Take among all 4 * 4 = 16 pairs of points (points from different rectangles)
    D = []
    for pair in perm:
        v0 = rect0[pair[0]]
        v1 = rect1[pair[1]]
        D.append(get_distance(v0, v1))
    d = min(D)
    return d

def get_dist_vert_segm(rect_vert, rect_segm):
    perm = [[x,y] for x in range(0, np.shape(rect_vert)[0]) for y in range(np.shape(rect_segm)[0])]
    ##############################
    D = []
    for pair in perm:
        # take one point from rect_vert
        ind_vert = pair[0]
        # take two consequtive points from rect_segm
        ind_s0 = pair[1]
        ind_s1 = (pair[1]+1) % np.shape(rect_segm)[0]  # if 4 --> 0

        point_vert = rect_vert[ind_vert]
        point_s0 = rect_segm[ind_s0]
        point_s1 = rect_segm[ind_s1]

        vect_segm = point_s0-point_s1
        vect_0 = point_s0-point_vert
        vect_1 = point_vert-point_s1
        
        angle0 = math.acos(np.dot(vect_segm, vect_0) / (np.linalg.norm(vect_segm) * np.linalg.norm(vect_0)))
        angle1 = math.acos(np.dot(vect_segm, vect_1) / (np.linalg.norm(vect_segm) * np.linalg.norm(vect_1)))

        # check if perpendicular belongs to the line p1--p2
        notBetween = False
        if (abs(math.degrees(angle0)) >90 or abs(math.degrees(angle1)) > 90):
            notBetween = True
        if not notBetween:
            #print('point_s0 = (%.2f, %.2f)' % (point_s0[0], point_s0[1]))
            #print('point_s1 = (%.2f, %.2f)' % (point_s1[0], point_s1[1]))
            #print('point_vert = (%.2f, %.2f)' % (point_vert[0], point_vert[1]))
            #print(math.degrees(angle0))
            #print(math.degrees(angle1))
            
            # perp length from vert to segment
            perp = np.linalg.norm(np.cross(point_s1-point_s0, point_s0-point_vert))/np.linalg.norm(point_s0-point_s1)
            D.append(perp) 
    try:
        d = min(D)
    except ValueError:
        d = float('Inf')
    return d


def evaluate_dist(vehicles):
    V = [] # array of vertices
    for vehicle in vehicles:
        transform = vehicle.get_transform() 
        bounding_box = vehicle.bounding_box
        #print(bounding_box)
        #print(transform)

        # 8 bounding box vertices relative to (0,0,0)
        ext = bounding_box.extent
        points = np.array([
            [  ext.x,   ext.y],
            [- ext.x,   ext.y],
            [- ext.x, - ext.y],
            [  ext.x, - ext.y]
        ])
        
        for point in points:
            ll = carla.Location(x=point[0],y=point[1],z=1)
            ll = transform.transform(ll)

            point[0] = ll.x
            point[1] = ll.y
            #world.world.debug.draw_point(ll, color=carla.Color(r=255, g=0, b=0))
        V.append(points)
        #world.world.debug.draw_box(bounding_box, transform.rotation)
    
    rect0 = V[0]
    rect1 = V[1]

    bool_collision = check_collision(rect0, rect1)
    #print(bool_collision)
    d = 0.0
    if not bool_collision:
        min1 = get_dist_vert_vert(rect0, rect1)
        min2 = get_dist_vert_segm(rect0, rect1)
        min3 = get_dist_vert_segm(rect1, rect0)

        #print('min1 = %.2f' % min1)
        #print('min2 = %.2f' % min2)
        #print('min3 = %.2f' % min3)
        d = min(min1, min2, min3)
    return d


