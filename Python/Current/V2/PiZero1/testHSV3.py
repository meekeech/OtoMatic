import numpy as np
import cv2

# Callback function, not used
def nothing(x):
    pass

# saved Filename
filename = 'saved1.jpg'

#boolean that tracks if a new slider is being changed
newSlider = True
firstTime = True
currentSlider = 'N'


# Convert BGR image to HSV image
win_img = "new"
win_img_old = "old"
slider1 = "Sliders"

pic = cv2.imread("/home/pi/Documents/ColorMods/test.jpg", cv2.IMREAD_UNCHANGED)
pic2 = pic.copy()

# Display the original image for comparison
#cv2.namedWindow(win_img_old, cv2.WINDOW_NORMAL) 
#cv2.imshow(win_img_old, pic) 

# New image window
cv2.namedWindow(win_img, cv2.WINDOW_NORMAL) 
cv2.imshow(win_img, pic)

cv2.namedWindow(slider1, cv2.WINDOW_AUTOSIZE) 


dim1 = np.zeros(shape=(380,400))
dim2 = np.zeros(shape=(380,400))
dim3 = np.zeros(shape=(380,400))

lastS = 1
lastH = 1
lastV = 1
lastG = 1
lastB = 1
lastR = 1
lastCl = 1
lastGd = 1


def HueChange(new_val):
    global pic,pic2, dim1,dim2,dim3,currentSlider,lastH
    if currentSlider is 'H' or currentSlider is 'N':
        pass
    else:
        pic = pic2
        
    currentSlider = 'H'
    
    pic_temp = cv2.cvtColor(pic, cv2.COLOR_BGR2HSV)
    dim1,dim2,dim3 = cv2.split(pic_temp)
    lastH = new_val/100
    dim1 = np.array(dim1*lastH,dtype = np.float64) 
    dim1 = np.clip(dim1,0,180) #use 180 for hue
    UpdatePic('hsv')
    
def SatChange(new_val):
    global pic,pic2 ,dim1,dim2,dim3,currentSlider,lastS
    #print(currentSlider)
    if currentSlider is 'S' or currentSlider is 'N':
        pass
    else:
        pic = pic2
        
    currentSlider = 'S'

    pic_temp = cv2.cvtColor(pic, cv2.COLOR_BGR2HSV)
    dim1,dim2,dim3 = cv2.split(pic_temp)
    lastS = new_val/100
    dim2 = np.array(dim2*lastS,dtype = np.float64) 
    dim2 = np.clip(dim2,0,255)
    UpdatePic('hsv')
    
def LightChange(new_val):
    global pic,pic2, dim1,dim2,dim3,currentSlider,lastV
     
    if currentSlider == 'V' or currentSlider is 'N':
        pass
    else :
        pic = pic2
        
    currentSlider = 'V'
    
    pic_temp = cv2.cvtColor(pic, cv2.COLOR_BGR2HSV)
    dim1,dim2,dim3 = cv2.split(pic_temp)
    lastV = new_val/100
    dim3 = np.array(dim3*lastV,dtype = np.float64) 
    dim3 = np.clip(dim3,0,255)
    UpdatePic('hsv')
    
def BlueChange(new_val):
    global pic,pic2,dim1,dim2,dim3,currentSlider,lastG
    
    if currentSlider == 'B' or currentSlider is 'N':
        pass
    else :
        pic = pic2
        
    currentSlider = 'B'
    
    dim1,dim2,dim3 = cv2.split(pic)
    lastG = new_val/100
    dim1 = np.array(dim1*lastG,dtype = np.float64) 
    dim1 = np.clip(dim1,0,255)
    UpdatePic('bgr')
    
def GreenChange(new_val):
    global pic,pic2,dim1,dim2,dim3,currentSlider,lastB
    
    if currentSlider == 'G' or currentSlider is 'N':
        pass
    else :
        pic = pic2
        
    currentSlider = 'G'
    
    dim1,dim2,dim3 = cv2.split(pic)
    lastB = new_val/100
    dim2 = np.array(dim2*lastB,dtype = np.float64) 
    dim2 = np.clip(dim2,0,255)
    UpdatePic('bgr')
    
def RedChange(new_val):
    global pic,pic2,dim1,dim2,dim3,currentSlider,lastR
    if currentSlider == 'R' or currentSlider is 'N':
        pass
    else :
        pic = pic2
        
    currentSlider = 'R'
    
    dim1,dim2,dim3 = cv2.split(pic)
    lastR = new_val/100
    dim3 = np.array(dim3*lastR,dtype = np.float64) 
    dim3 = np.clip(dim3,0,255)
    UpdatePic('bgr')

def ClipChange(new_val):
    global currentSlider, lastCl, lastGd, pic, pic2
    if currentSlider == 'Cl' or currentSlider is 'N':
        pass
    else :
        pic = pic2
    currentSlider = 'Cl'
    lastCl = new_val/100
    
    yuv = cv2.cvtColor(pic, cv2.COLOR_BGR2YUV)
    #dim1,dim2,dim3 = cv2.split(yuv)
    clahe = cv2.createCLAHE(clipLimit=lastCl,tileGridSize=(lastGd,lastGd))
    #this won't work with the dim1 method and new function - figure out why
    yuv[:, :, 0] = clahe.apply(yuv[:, :, 0])
    pic2 = cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR)
    cv2.imshow(win_img, pic2)
    #UpdatePic('yuv')
    
def GridChange(new_val):
    global currentSlider, lastCl, lastGd, pic, pic2
    if currentSlider == 'Gd' or currentSlider is 'N':
        pass
    else :
        pic = pic2
    currentSlider = 'Gd'
    lastGd = round(new_val/100)
    
    yuv = cv2.cvtColor(pic, cv2.COLOR_BGR2YUV)
    #dim1,dim2,dim3 = cv2.split(yuv)
    clahe = cv2.createCLAHE(clipLimit=lastCl, tileGridSize=(lastGd,lastGd))
    yuv[:, :, 0] = clahe.apply(yuv[:, :, 0])
    pic2 = cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR)
    cv2.imshow(win_img, pic2)
    #UpdatePic('yuv')

def UpdatePic(changeType):
    global pic, pic2, dim1,dim2,dim3,win_img
    pic2 = cv2.merge([np.uint8(dim1) , np.uint8(dim2) , np.uint8(dim3)])
    
    if changeType == 'hsv':
        pic2 = cv2.cvtColor(pic2, cv2.COLOR_HSV2BGR)
    if changeType == 'yuv':
        pic2 = cv2.cvtColor(pic2, cv2.COLOR_YUV2BGR)

    cv2.imshow(win_img, pic2)
    

        
#Initialize the scroll bar
cv2.createTrackbar("H",slider1, 100, 200, HueChange)
cv2.createTrackbar("S",slider1, 100, 200, SatChange)
cv2.createTrackbar("V",slider1, 100, 200, LightChange)

cv2.createTrackbar("B",slider1, 100, 300, BlueChange)
cv2.createTrackbar("G",slider1, 100, 200, GreenChange)
cv2.createTrackbar("R",slider1, 100, 200, RedChange)

cv2.createTrackbar("Cl",slider1, 100, 300, ClipChange)
cv2.createTrackbar("Gd",slider1, 100, 800, GridChange)

while True:
	# ESC press to exit
    if cv2.waitKey(10) == 27:
        # Saving the image
        cv2.imwrite(filename, pic2)
        print(lastH,end = ' ')
        print(lastS,end = ' ')
        print(lastV,end = ' ')
        print(lastG,end = ' ')
        print(lastB,end = ' ')
        print(lastR)
        break

cv2.destroyAllWindows()
