
import cv2
import numpy as np
import matplotlib.pyplot as plt
import PPPKod as testing
import maze_solver as ms

if __name__ == "__main__":

    rospy.init_node("Main_controller", anonymous=True)
    
    while(True):
        _,imageFrame = webcam.read()
    
        cv2.startWindowThread()
        cv2.namedWindow("image")
        cv2.imshow('image', imageFrame)
        pts1 = np.array(0)
        while pts1.shape !=(4,2):
            try:
                pts1 = testing.izracunaj_koordinate_vrhova(imageFrame)
            except Exception as e:
                pass

            _,imageFrame = webcam.read() 
            pts1 = np.array(pts1)
            #plt.imshow(imageFrame)
            #plt.show()
            
        pts1 = np.array(pts1)
        print(f"Points to transform: {pts1}")
        pts2 = np.float32([[0, 650], [650, 650], [0, 0],  [650, 0] ])
        M = cv2.getPerspectiveTransform(pts1.astype(np.float32),pts2)

        new_img = cv2.warpPerspective(imageFrame, M, (650, 650))
        new_img = cv2.flip(new_img, 1)
        cv2.imshow("transformirano", new_img)

        
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
        ms.find_path(blackAndWhiteImage)


        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
    webcam.release()
cv2.destroyAllWindows()
