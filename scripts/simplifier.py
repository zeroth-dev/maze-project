
#!/bin/usr/env python3
import sys
import cv2
import matplotlib.pyplot as plt
import numpy as np
from collections import deque

def print_board(board): 
    row=len(board)
    col=len(board[0])
    for i in range(0, row):
        for j in range(0, col):
            print(board[i][j], end="")
        print('')

BLACK = 0
WHITE = 255

def count_pixels(maze, limits):
    min_row=limits[0][0]
    min_col=limits[0][1]
    max_row=limits[1][0]
    max_col=limits[1][1]
    white_px = 0
    black_px = 0
    if max_row>649:
        max_row=649
    if max_col>649:
        max_col=649
    for row in range(min_row, max_row):
        for col in range(min_col, max_col):
            if maze[row][col] == WHITE:
                white_px+=1
            else:
                black_px+=1
            maze[row][col]=100
    
  
    return (white_px, black_px)


def fill_errors(maze):
    for col in range(0, 650):
        dbg=maze[0][col]
        if maze[0][col]==WHITE and (maze[1][col]==BLACK or maze[2][col]==BLACK or maze[3][col]==BLACK):
            flood_fill(maze, (0, col))
        dbg=maze[649][col]
        if maze[649][col]==WHITE and (maze[648][col]==BLACK or maze[648][col]==BLACK or (col == 0 or col == 649)):
            flood_fill(maze, (649, col))

def flood_fill(maze, position):
    row = position[0]
    col = position[1]
    if maze[row][col]==WHITE and (maze[1][col]==BLACK or maze[2][col]==BLACK or maze[3][col]==BLACK): 
        dbg=maze[row][col]
        maze[row][col]=0
        dbg2=maze[row][col]
        if row>0:
            flood_fill(maze, (row-1, col))
        if row<649:
            flood_fill(maze, (row+1, col))
        if col>0 and col<649:
            flood_fill(maze, (row, col+1))
            flood_fill(maze, (row, col-1))


def find_min(maze, limits):
    minimum=1000
    row=0
    col=0
    
    min_row=limits[0][0]+6
    min_col=limits[0][1]+6
    max_row=limits[1][0]-1
    max_col=limits[1][1]-1

    ### White width
    for i in range(min_col+1, max_col):
        count = 0
        if maze[min_row][i]==WHITE and not maze[min_row][i-1] ==WHITE:
            while(maze[min_row][i+count]== WHITE):
                count+=1
                if i+count>max_col:
                    break
        if count<minimum and count != 0 and count > 20:
            minimum = count
            row=min_row
            col=i

        
        count2 = 0
        if maze[max_row][i]==WHITE and not maze[max_row][i-1] ==WHITE:
            while(maze[max_row][i+count2]== WHITE):
                count2+=1
        if count2<minimum and count2 != 0 and count2 > 20:
            minimum = count2
            row=max_row
            col=i

    for i in range(min_row+1, max_row):
        count = 0
        if maze[i][min_col]==WHITE and not maze[i-1][min_col] ==WHITE:
            while(maze[i+count][min_col]== WHITE):
                count+=1
        if count<minimum and count != 0 and count > 20:
            minimum = count
            row=i
            col=min_col

        count2 = 0
        if maze[i][max_col]==WHITE and not maze[i-1][max_col] ==WHITE:
            while(maze[i+count2][max_col]== WHITE):
                count2+=1
        if count2<minimum and count2 != 0 and count2 > 20:
            minimum = count2
            row=i
            col=max_col

        ### Black width
    minimum_black=100
    row_b=0
    col_b=0
    if row == max_row or row == min_row:
        for i in range(min_row+1, max_row):
            count = 0
            if maze[i][col]==BLACK and not maze[i-1][col] ==BLACK:
                while(maze[i+count][col]== BLACK):
                    count+=1
                    if i+count>=max_row:
                        break
            if count<minimum_black and count != 0:
                minimum_black = count
                row_b=i
                col_b=col

    if col == max_col or col == min_col+1:
        for i in range(min_col+1, max_col):
            count = 0
            if maze[row][i]==WHITE and not maze[row][i-1] ==WHITE:
                while(maze[row][i+count]== WHITE):
                    count+=1
            if count<minimum_black and count != 0:
                minimum_black = count    
    return(minimum, minimum_black)

def simplify_maze(maze, limits):
    (minimum_white, minimum_black) = find_min(maze, limits)
    minimum_white=55
    minimum_black=9
    min_row=limits[0][0]
    min_col=limits[0][1]
    max_row=limits[1][0]
    max_col=limits[1][1]
    curr_row=min_row
    curr_col=min_col
    new_maze=[]
    wall=True
    new_row=0
    new_col=0
    while(new_row<21):
        row_wall=[]
        
        while(curr_col<max_col-minimum_black and new_col<21):                  
            if(new_row % 2 == 0 and new_col % 2 == 0):              # Corners of the walls are always "black" and cannot be passed through
                new_col+=1
                row_wall.append('#')
                curr_col+=minimum_black
            elif(new_row % 2 == 1 and new_col % 2 == 1):              # Between the walls is always empty space
                new_col+=1
                row_wall.append('.')
                curr_col+=minimum_white
            else:
                if new_row % 2 == 0:
                    dimensions = ((curr_row, curr_col),(curr_row+minimum_black, curr_col+minimum_white))
                    curr_col+=minimum_white
                else:
                    dimensions = ((curr_row, curr_col),(curr_row+minimum_white, curr_col+minimum_black))
                    curr_col+=minimum_black
                white_px, black_px = count_pixels(maze, dimensions)
                if white_px>black_px and black_px<60:
                    row_wall.append('.')
                else:
                    row_wall.append('#')
                new_col+=1

        
        new_maze.append(row_wall)

        if new_row % 2 == 0:
            curr_row+=minimum_black
        else:
            curr_row+=minimum_white

        new_row+=1
        new_col=0

        curr_col=min_col
    
    plt.figure(figsize=(7,7))
    plt.imshow(maze) # show the image on the screen 
    plt.show()

    return new_maze
'''
        K = 0

        if wall:
            K=int((minimum_black+1)/2)
        else:
            K=int(minimum/2)


        while(curr_col<max_col-minimum_black):
          
            if maze[curr_row+K][curr_col] == WHITE:
                row_wall.append('.')
            if maze[curr_row+K][curr_col] == BLACK:
                row_wall.append('#')
            curr_col+=minimum_white

            if maze[curr_row+K][curr_col] == WHITE:
                row_wall.append('.')
            if maze[curr_row+K][curr_col] == BLACK:
                row_wall.append('#')
            curr_col+=minimum_black+1

        if wall:
            curr_row+=minimum_black+1
            wall = not wall
        else:
            curr_row+=minimum_white
            wall = not wall
'''
       

  


'''
originalImage = cv2.imread('new_rot.jpg')
grayImage = cv2.cvtColor(originalImage, cv2.COLOR_BGR2GRAY)
(thresh, blackAndWhiteImage) = cv2.threshold(grayImage,90, 255, cv2.THRESH_BINARY)
blackAndWhiteImage = cv2.medianBlur(blackAndWhiteImage,5)
plt.figure(figsize=(7,7))
plt.imshow(blackAndWhiteImage) # show the image on the screen 
plt.show()
simple_maze=simplify_maze(blackAndWhiteImage)

print(minimum)
print(minimum_black)
print(row_b)
print(col)
'''