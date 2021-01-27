import numpy as np
import cv2
from matplotlib import pyplot as plt
import maze_solver as ms
import PPPKod as testing

if __name__ == "__main__":
    imageFrame = cv2.imread("Labyrinth2.png")

    pts1 = testing.izracunaj_koordinate_vrhova(imageFrame)

    pts1 = np.array(pts1)
    print(f"Points to transform: {pts1}")
    pts2 = np.float32([[0, 0],  [0, 649],[649, 0], [649, 649]])
    #pts2=pts2[::-1]
    M = cv2.getPerspectiveTransform(pts1.astype(np.float32),pts2)

    new_img = cv2.warpPerspective(imageFrame, M, (650, 650))
    #new_img = cv2.flip(new_img, 1)
    show = cv2.cvtColor(new_img, cv2.COLOR_BGR2RGB)
        
    grayImage = cv2.cvtColor(new_img, cv2.COLOR_BGR2GRAY)
 
    (thresh, blackAndWhiteImage) = cv2.threshold(grayImage,90, 255, cv2.THRESH_BINARY)
    maze_image = new_img
    edges = cv2.Canny(blackAndWhiteImage,0, 255,apertureSize = 3)
    minLineLength = 100
    maxLineGap = 5
    lines = cv2.HoughLinesP(edges,1,np.pi/180,10, minLineLength, maxLineGap)
    for line in lines:
        x1, y1, x2, y2 = line[0]
        cv2.line(maze_image, (x1, y1), (x2, y2), (0, 0, 0), 4)
    plt.figure(figsize=(7,7))
    plt.imshow(blackAndWhiteImage) # show the image on the screen 
    plt.show()

        # TODO Maze solving
        #simplified_maze = ms.simplify(blackAndWhiteImage)
    ms.find_path(blackAndWhiteImage, (1,1))