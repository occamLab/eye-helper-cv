import cv2 
import numpy as np 

def labelData(basename, startframe, endframe):
    """Takes in the video and the section where the object is visible
    allows the user to create a box around the ojbect being tracked. The 
    corners of this box are saved as the position of the object"""
    c1 = (0,0)
    c2 = (20,20)
    for i in range(startframe, endframe + 1):
        frame = (4 -len(str(i))) * '0' + str(i) #names start w/ 4 '0's this gets the correct number
        
        while(1):
            temp = cv2.imread(basename + frame +'.jpg')         #draws the rectangle over the image resizing/ translating based on user imnput
            k = cv2.waitKey(5)
            cv2.rectangle(temp,c1, c2, (0, 0, 255),3) 
            cv2.imshow('rec', temp)
            if k == ord('s'):
                c1 = (c1[0], c1[1]+5)
            elif k == ord('d'):
                c1 = (c1[0]+5, c1[1])
            elif k == ord('a'):
                c1= (c1[0]-5, c1[1])
            elif k == ord('w'):
                c1= (c1[0], c1[1]-5)
            elif k == 65363:        #Right
                c1 = (c1[0]+5, c1[1])
                c2 = (c2[0]+5, c2[1])
            elif k == 65361:        #Left
                c1 = (c1[0]-5, c1[1])
                c2 = (c2[0]-5, c2[1])
            elif k == 65364:        #up
                c1 = (c1[0], c1[1]+5)
                c2 = (c2[0], c2[1]+5)
            elif k == 65362:        #down
                c1 = (c1[0], c1[1]-5)
                c2 = (c2[0], c2[1]-5)

            elif k == 32:
                break

        file = open(basename+".csv", "w")

        file.write(frame +"," + str(c1) +"," + str(c2))

        file.close()



