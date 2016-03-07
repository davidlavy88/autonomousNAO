# autonomousNAO

Final code for EC520 Project called "Autonomous Navigation with NAO"

Description of the most important files used in this project:

COM.py : Calculates center of mass for a picture with a red ball. <br />
Contour.py : Calculates the contour of a red ball using HSV thresholding <br />
ContourErr.py : Calculates the contour of a red ball, applying error metrics (specified in the report). <br />
circle.py : Calculates the contour of a red ball using minimum enclosing circle method. <br />
circleErr - print center.py : Calculates contour and center of a red ball adding same metrics as ContourErr file. It is possible to run and see the output with one of the images in the folder <br />
Full_Search_and_Kick.py : Final version for the project using NAO robot. Replace robot IP with your local robot IP if running this module remotely from your PC. If you want to run this file on your NAO directly, replace the IP number with 'nao.local' <br />

For a full demo of how this work you can check the video here: https://www.youtube.com/watch?v=Lp79IdTRV4Q <br />
For a detailed explanation of how this project was done, please refer to the report posted here: http://sites.bu.edu/jkonrad/courses/ <br />
