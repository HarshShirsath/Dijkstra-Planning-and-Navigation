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
        r_circle = 5
        x,y = x_ynode
        layer = np.zeros_like(frame)
        mask_circle = cv2.bitwise_and(frame, layer)
        cv2.circle(layer, (x, y), r_circle, (255,255,255), -1)
        
        
        return mask_circle
        

    def visualisation(surface, nodes):
        parents_node = {}
        for x_ynode, (cost, index, parent) in nodes.items():
            if parent is not None:
                if parent not in parents_node:
                    parents_node[parent] = []
                parents_node[parent].append(x_ynode)    
        color = (0,0,255)                               
        initial = 0
        for parent, visited_nodes in parents_node.items():
            surface_shape = surface.shape[0]-1
            x_ycoo = np.array([(surface_shape-x_ynode[1], x_ynode[0]) for x_ynode in visited_nodes]).transpose()
            surface[x_ycoo[0], x_ycoo[1], :] = color
            initial += len(visited_nodes)
            if initial % 100 == 0:
                cv2.waitKey(1)

    def pointer(surface,trace):
        r_circle = 1
        color = (0, 255, 255)
        
        for x_ynode in trace:
            x,y = x_ynode
            
            cv2.circle(surface, (x,y), r_circle, color, 1) 
            cv2.imshow("Map", surface)
            cv2.waitKey(2)
            cv2.circle(surface, (x,y), r_circle+1, (255,0,0), -1)
            cv2.imshow("Map", surface)
            cv2.waitKey(1)

        cv2.waitKey(2000)
        cv2.destroyAllWindows()

    def tracking(nodes, start, goal_path):
        trace = []      
        x_ynode = goal_path      
        while x_ynode != start:
            trace.append(x_ynode)
            parent = nodes[x_ynode][2]      
            x_ynode = parent
        trace.append(start)
        trace.reverse() 
        shift = surface.shape[0]-1
        traced = [((x,shift-y)) for (x,y) in trace]
        return traced
    
    hq.heappush(open_loop,(0,start))
    
    while open_loop:
        cost, x_ynode = hq.heappop(open_loop)
        if x_ynode in close_loop:
            continue

        close_loop.add(x_ynode)
        if x_ynode == goal_path:
            # print("Goal Reached\n\n")
            visualisation(surface, nodes)
            trace = tracking(nodes,start,goal_path)
            # track_animate(surface,trace)
            pointer(surface,trace)
            cv2.waitKey(400000)
            cv2.destroyAllWindows()
            return nodes

        if goal_path in open_loop:
            continue
        to_be_visited(surface,x_ynode)


    def show(frame):
        cv2.imshow('Surface',frame) 

    def pop_color(frame,node,color):   

        x,y = node
        x = x
        y = (frame.shape[0]-1)-y
        color = np.array([255, 0, 0])      
        frame[x][y] = color
    
def visualize(start,goal):
    def shift_origin(frame,node):      
        x,y = node
        x = x
        y = (frame.shape[0]-1)-y
        return (x,y)

    def start_init(frame,node):
        x,y = shift_origin(frame,node)
        cv2.circle(frame,(x,y),5,(0,0,255),-1)
        return frame

    def goal_init(frame,node):
        x,y = shift_origin(frame,node)
        cv2.circle(frame,(x,y),5,(0,0,255),-1)
        return frame
    obstacles = (0,255,0)
