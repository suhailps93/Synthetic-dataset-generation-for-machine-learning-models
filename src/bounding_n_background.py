import cv2
import numpy as np
import glob
import pandas as pd
import xml.etree.ElementTree as ET
import os

bgcount=0
bgimages=[]

def get_bounding_box(imagepath):
    img = cv2.pyrDown(cv2.imread(imagepath, cv2.IMREAD_UNCHANGED))

    # threshold image
    ret, threshed_img = cv2.threshold(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY),
                    127, 255, cv2.THRESH_BINARY)
    # find contours and get the external one
    image, contours, hier = cv2.findContours(threshed_img, cv2.RETR_TREE,
                    cv2.CHAIN_APPROX_SIMPLE)

    # with each contour, draw boundingRect in green
    # a minAreaRect in red and
    # a minEnclosingCircle in blue
    for c in contours:

        # get the bounding rect
        x, y, w, h = cv2.boundingRect(c)

        if x!=0 and y!=0:
            return x, y, x+w, y+h

def generate_csv(path):

    xml_list = []
    for image in glob.glob(path + '/*.jpg'):

        xml_list.append(changebg(str(image),path))

    column_name = ['filename', 'width', 'height', 'class', 'xmin', 'ymin', 'xmax', 'ymax']
    xml_df = pd.DataFrame(xml_list, columns=column_name)
    return xml_df

def changebg(imagepath,path):
    global bgcount
    print bgimages[bgcount]
    dest = cv2.pyrDown(cv2.imread(bgimages[bgcount], cv2.IMREAD_UNCHANGED))
    img = cv2.pyrDown(cv2.imread(imagepath, cv2.IMREAD_UNCHANGED))


    height,width,x=dest.shape
    height=height*2
    width=width*2
    xmin,ymin,xmax,ymax=get_bounding_box(imagepath)

#read as argument
    classname='tide'

    filename=imagepath[(len(path)+1):]
    value = (filename,width,height,classname,int(xmin),int(ymin),int(xmax),int(ymax))


    # threshold image
    ret, threshed_img = cv2.threshold(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY),
                    127, 255, cv2.THRESH_BINARY)
    # find contours and get the external one
    image, contours, hier = cv2.findContours(threshed_img, cv2.RETR_TREE,
                    cv2.CHAIN_APPROX_SIMPLE)

    # with each contour, draw boundingRect in green
    # a minAreaRect in red and
    # a minEnclosingCircle in blue
    for c in contours:
        # get the bounding rect
        x, y, w, h = cv2.boundingRect(c)
        # draw a green rectangle to visualize the bounding rect
        if x!=0 and y!=0:
            # cv2.rectangle(img, (x, y), (x+w, y+h), (0, 255, 0), 2)

            # get the min area rect
            rect = cv2.minAreaRect(c)
            box = cv2.boxPoints(rect)
            # convert all coordinates floating point values to int
            box = np.int0(box)
            # draw a red 'nghien' rectangle
            # cv2.drawContours(img, [box], 0, (0, 0, 255))


        else:
            contours.remove(c)


    print(len(contours))
    # print(img.shape)
    mask = np.zeros(img.shape, dtype = "uint8")
    cv2.drawContours(mask, contours, -1, (255, 255, 255), -1)

    maskedImg = cv2.bitwise_and(img, mask)
    mask_inv = cv2.bitwise_not(mask)

    rows,cols,channels = img.shape
    roi = dest[0:rows, 0:cols ]

    print "Printing roi and mask"
    print roi.shape
    print img.shape

    destImg=cv2.bitwise_and(roi, mask_inv)
    final=destImg+maskedImg



    dest[0:rows, 0:cols ] = final

    print imagepath
    # cv2.rectangle(dest, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)
    cv2.imwrite(imagepath,dest)
    print "Saved image"

    if bgcount>2:
        bgcount=0
    else:
        bgcount=bgcount+1

    return value



def main():
    # print (1,2,3,4)
    global bgcount,bgimages
    os.chdir("../images")

    bgpath = os.path.join(os.getcwd(), 'background_images/')
    bgimages=glob.glob(bgpath + '/*.jpg')

    for directory in ['test','train']:

        path = os.path.join(os.getcwd(), 'cv_input/{}'.format(directory))
        # print (path)
        xml_df = generate_csv(path)
        print (path)
        xml_df.to_csv('../data/{}_labels.csv'.format(directory), index=None)
        print('Successfully generated bounding boxes.')



main()
