import heapq as hq
import numpy as np
import cv2
import time
# import visualize as vis

exp_color = (255, 0, 0)

global index 
index = 1
def plot(surface,start,goal_path):
    nodes = {}
    nodes[start] = (0,0,None)
    
    open_loop = []
    close_loop = set()

    def north(surface, x_ynode, cost):
        x,y = x_ynode
        x = x
        if y < surface.shape[0]-1:
            y += 1
            cost += 1
            node_new = (x,y)
            return (surface,node_new,cost)
        return (surface,None, None)
        
    def northeast(surface, x_ynode, cost):
        x,y = x_ynode
        if x < (surface.shape[1]-1) and y < (surface.shape[0]-1) :
            x += 1
            y += 1
            cost += 1.4
            node_new = (x,y)
            return (surface,node_new, cost)
        return (surface,None, None)

    def east(surface, x_ynode, cost):
        x,y = x_ynode
        y = y
        if x < (surface.shape[1]-1):
            x += 1
            cost += 1
            node_new = (x,y)
            return (surface,node_new,cost)
        return (surface,None, None)

    def southeast(surface, x_ynode, cost):
        x,y = x_ynode
        if x < (surface.shape[1]-1) and y > 0 :
            x += 1
            y -= 1
            cost += 1.4
            node_new = (x,y)
            return (surface,node_new,cost)
        return (surface,None, None)

    def south(surface, x_ynode, cost):
        x,y = x_ynode
        x = x
        if y > 0 :
            y -= 1
            cost += 1
            node_new = (x,y)
            return (surface,node_new,cost)
        return (surface,None, None)

    def southwest(surface, x_ynode, cost):
        x,y = x_ynode
        if x > 0 and y > 0 :
            x -= 1
            y -= 1
            cost += 1.4
            node_new = (x,y)
            return (surface,node_new,cost)
        return (surface,None, None)

    def west(surface, x_ynode, cost):
        x,y = x_ynode
        y = y
        if x > 0:
            x -= 1
            cost += 1
            node_new = (x,y)
            return (surface,node_new,cost)
        return (surface,None, None)
    
    def northwest(surface, x_ynode, cost):
        x,y = x_ynode
        if x > 0 and y < surface.shape[0]-1 :
            x -= 1
            y += 1
            cost += 1.4
            node_new = (x,y)
            return (surface,node_new,cost)
        return (surface,None,None)
    
    
    
    def to_be_visited(surface,x_ynode):
        parent = x_ynode
        peak_cost = nodes[x_ynode][0]
        for i in [north,northeast,east,southeast,south,southwest,west,northwest]:
            surface,node_new,cost = i(surface,x_ynode, peak_cost)
            if node_new == None:
                continue
            (x,y)  = node_new
            y_up = (surface.shape[0]-1)-y
            if node_new in close_loop:
                continue
            layer = visual(surface,node_new)
            if np.any(layer[:,:,1]):
                continue

            if node_new not in nodes or cost < nodes[node_new][0]:
                global index
                index += 1
                nodes[(x,y)] = (cost, index, parent)
                
                hq.heappush(open_loop, (cost,node_new))
            if node_new == goal_path:
                break 


    def visual(frame,x_ynode):
        x,y = x_ynode
        r_circle = 5
        layer = np.zeros_like(frame)
        cv2.circle(layer, (x, y), r_circle, (255,255,255), -1)
        mask_circle = cv2.bitwise_and(frame, layer)
        
        return mask_circle
        