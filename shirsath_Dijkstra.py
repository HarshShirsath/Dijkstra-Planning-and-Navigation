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