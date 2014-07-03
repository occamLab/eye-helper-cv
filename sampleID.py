import cv2 
import numpy as np 
import csv
import time

def labelData(basename, startframe, endframe):
    """Takes in the video and the section where the object is visible
    allows the user to create a box around the ojbect being tracked. The 
    corners of this box are saved as the position of the object

    ***Assumes that the input will be a jpg file
    ***endframe is inclusive (eg. if you choose 2 you will end on frame 2
        not on frame 1)
    ***Assumes use of linux in way filepaths are defined
    ***Creates a csv file where the first entry is the frame, the second and 
    third represent the x,y coordinates of the top left corner and the fourth
    and fifth represent the x,y coordinates of the bottom right corner
    """
    c1 = (0,0)
    c2 = (20,20)
    #creates a new csv file to store data
    path = './gstore-snippets/%s_snippet/' %basename
    print('Begining calibration, please press the up key')
    temp = cv2.namedWindow('calibrate')
    k = cv2.waitKey(0)
    if k != -1:
        up = k
    print('Please press the down key')
    k = cv2.waitKey(0)
    if k != -1:
        down = k
    print('Please press the left key')
    k = cv2.waitKey(0)
    if k != -1:
        left = k 
    print('Please press the right key')
    k = cv2.waitKey(0)
    if k != -1:
        right = k
    cv2.destroyAllWindows()
    print('Thank you. Done Calibrating')
    with open(path + basename +".csv", "w") as csvfile:
        keypointwriter = csv.writer(csvfile, delimiter= ',',
                               quotechar='|', quoting=csv.QUOTE_MINIMAL)  
        for i in range(startframe, endframe + 1)[::-1]:
            #names start w/ 4 '0's below makes correct number to add to basename
            frame = (5 -len(str(i))) * '0' + str(i) 
            while(1):
                #draws rectangle over image resizing/ translating from user imnput
                temp = cv2.imread(path + basename +'_'+ frame +'.jpg')
                cv2.rectangle(temp,c1, c2, (0, 0, 255),2) 
                cv2.imshow('rec', temp)
                k = cv2.waitKey(5)
                if k == ord('s'):
                    c1 = (c1[0], c1[1]+5)
                elif k == ord('d'):
                    c1 = (c1[0]+5, c1[1])
                elif k == ord('a'):
                    c1= (c1[0]-5, c1[1])
                elif k == ord('w'):
                    c1= (c1[0], c1[1]-5)
                elif k == right:        #expand box width
                    c1 = (c1[0]+5, c1[1])
                    c2 = (c2[0]+5, c2[1])
                elif k == left:        #decrease box width
                    c1 = (c1[0]-5, c1[1])
                    c2 = (c2[0]-5, c2[1])
                elif k == down:        #expand box height
                    c1 = (c1[0], c1[1]+5)
                    c2 = (c2[0], c2[1]+5)
                elif k == up:        #decrease box height
                    c1 = (c1[0], c1[1]-5)
                    c2 = (c2[0], c2[1]-5)

                elif k == 32:
                    break

            keypointwriter.writerow([frame, c1[0], c1[1], c2[0], c2[1]])

if __name__ == '__main__':
    labelData('catfood', 266, 762)

