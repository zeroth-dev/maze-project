#%%
#!/bin/usr/env python3
import cv2
import matplotlib.pyplot as plt
import numpy as np
from collections import deque

# TODO: Make a different check for walls
# TODO: Find entrance to the labyrinth


class Vertex:                                           # For each point we need to know:
    def __init__(self,x_coord,y_coord, parent):         # 
        self.x=x_coord                                  # Coordinates of the point
        self.y=y_coord                                  #
        self.R=0                                        #
        self.G=0                                        # RGB values of the point (default 0 because python)
        self.B=0                                        #
        self.rgb=0                                      # Sum of rgb values for isolating black area
        self.d=-1                                       # Distance from source (optional for this application)                 
        self.parent=parent                              # Previous point for path finding
        self.processed=False                            # If the point has been processed so we don't return to it

    def find_rgb(self):                                 # This saves some space later
        self.rgb=int(self.R)+int(self.G)+int(self.B)    # Cast to int is important because python saves memory and assingns short instead of int
    
def is_wall(mat, r, c, direction, n=1):                 # Check if there is a wall in the 1 to n distance in the given direction from the given point

    wall=False                                  
    if direction=='r':
        for i in range(1, n+1):            
            if not (c+i)>(mat.shape[1]-1) and mat[r][c+i].rgb<400:  # Check for out of bounds and if the pixel is "black" (needs a different check TODO)
                wall=True
                break
    elif direction=='l':
        for i in range(1, n+1):
            if not (c-i)<0 and mat[r][c-i].rgb<400:
                wall=True
                break
    elif direction=='u':
        for i in range(1, n+1):
            if not (r-i)<0 and mat[r-i][c].rgb<400:
                wall=True
                break
    elif direction=='d':
        for i in range(1, n+1):
            if not (r+i)>(mat.shape[0]-1) and mat[r+i][c].rgb<400:
                wall=True
                break
    return wall

def has_wall(mat, vertex, n):                                   # Helper function used to check if the neighbor has walls 
    wall=False                                                  # inside a square size 2n-1x2n-1

    for row in range(-n+1,n):
        for col in range (-n+1,n):
            if ((vertex.y+row)<(mat.shape[1]-1) and (vertex.y+row)>0 
            and (vertex.x+col)<(mat.shape[0]-1) and (vertex.x+col)>0
            and mat[vertex.y+row][vertex.x+col].rgb<400):
                wall=True
                break
        if wall:
            break

    return wall


def get_neighbors(mat,r,c, n=1):                                # Return neighbor directly above, below, right, and left
    shape=mat.shape
    neighbors=[]
    if r > 0 and not is_wall(mat, r, c, 'u', n) and not mat[r-1][c].processed : 
        if not has_wall(mat,mat[r-1][c], n):                    # If the neighbour doesn't have wall nearby, add it to the queue
            neighbors.append(mat[r-1][c])
    if r < shape[0] - 1 and not is_wall(mat, r, c,'d', n) and not mat[r+1][c].processed:
        if not has_wall(mat,mat[r+1][c], n):
            neighbors.append(mat[r+1][c])
    if c > 0 and not is_wall(mat, r, c, 'l', n) and not mat[r][c-1].processed:
        if not has_wall(mat,mat[r][c-1], n):    
            neighbors.append(mat[r][c-1])
    if c < shape[1] - 1 and not is_wall(mat, r, c,'r', n) and not mat[r][c+1].processed:
        if not has_wall(mat,mat[r][c+1], n):
            neighbors.append(mat[r][c+1])
    return neighbors

def drawPath(img,path, thickness=1):
    '''path is a list of (x,y) tuples'''
    x0,y0=path[0]
    for vertex in path[1:]:
        x1,y1=vertex
        cv2.line(img,(x0,y0),(x1,y1),(255,0,0),thickness)
        x0,y0=vertex

def find_shortest_path(img,src, dist=1):
    pq = deque()                                                # FIFO queue
    source_x=src[0]                                             # Source coordinates
    source_y=src[1]
    imagerows,imagecols=img.shape[0],img.shape[1]               # Get the size of the image
    matrix = np.full((imagerows, imagecols), None)              # Create an empty 2D array
    for r in range(imagerows):
        for c in range(imagecols):                              # Fill the array with vertices of given coordinate and set parent to "None"
            matrix[r][c]=Vertex(c,r,None)                       # Remember that rows are y coordinate and collumn is x coordinate
    matrix[source_y][source_x].d=0                              # Distance of source node is 0
    for r in range(imagerows):
        for c in range(imagecols):
            matrix[r][c].R=img[(r, c)][0]                       # Set the RGB values of each vertex
            matrix[r][c].G=img[(r, c)][1]                       # img is a 3D array, two dimensions for location and third dimension for RGB
            matrix[r][c].B=img[(r, c)][2]
            matrix[r][c].find_rgb()
    dest_x=-1                                                   # Set the destination variables to -1 (no destination)
    dest_y=-1
    out = False                                                 # out is True if exit has been found
    pq.append(matrix[source_y][source_x])                       # Add the source node to the FIFO queue
    while len(pq) > 0:                                          # Algorithm runs while there are nodes(points) left in the queue or we break out if we found the exit
        node=pq.popleft()
        node.processed=True
        neighbors = get_neighbors(matrix,node.y,node.x, n=dist) # Get the neighboring points
        for v in neighbors:
            v.d = node.d+1                                      # Increase the distance by 1
            v.parent=node                                       # Set parent node 
            v.processed=True                                    # Ignore on next pass
            pq.append(v)                                        # Add to queue
            if v.rgb>500 and (v.x==0 or v.y==0                  # Hard coded exit requirement, needs change!!! (TODO)
                or v.x == img.shape[0]-1                        # Program is finished once it has reached boundaries of image(labyrinth)
                or v.y == img.shape[1]-1 and v.d>15):           # AND the distance is greater than a dummy value (needed so it doesn't go backwards towards the entrance)
                dest_x=v.x                                      # Set the destination coordinates
                dest_y=v.y
                out=True                                        # Exit was found
                break
        if out:
            break                                              

    path=[]
    iter_v=matrix[dest_y][dest_x]
    path.append((dest_x,dest_y))                                # Construction the path using coordinates from parent nodes
    while(iter_v.y!=source_y or iter_v.x!=source_x):
        #print("x coord:%d y coord:%d"%(iter_v.x, iter_v.y))    # Debug purpose
        iter_v=matrix[iter_v.parent.y][iter_v.parent.x]
        path.append((iter_v.x, iter_v.y))
                   
    return path                                                 # Return the constructed path

img = cv2.imread('maze.jpg') # read an image from a file using opencv (cv2) library
p = find_shortest_path(img, (469,496), dist=4) #pass the image, starting location and distance from walls+1
drawPath(img,p)
plt.figure(figsize=(7,7))
plt.imshow(img) # show the image on the screen 
plt.show()
# %%
