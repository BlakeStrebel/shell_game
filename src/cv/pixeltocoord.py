# this version has the camera rotated
# steps to try:
#   rotate camera
#   switch so that its not 640 minus
#   figure out how to send to location
#   test this

x = 0.651650130649
y =-0.0697635654082
z = 0.277211254272
zoffset = -0.287811174717

pix_size = .0023 #Camera calibration (meter/pixels); Pixel size at 1 meter height of camera
h = z-zoffset #Height from table to camera, when at vision place
x0b = x # x position of camera in baxter's base frame, when at vision place
y0b = y # y position of camera in baxter's base frame, when at vision place
x_camera_offset = .02 #x camera offset from center of the gripper
y_camera_offset = -.02 #y camera offset from center of the gripper
# height, width, depth = cv_image.shape #camera frame dimensions
height = 400
width = 640

#Position of the object in the baxter's stationary base frame
xb = (cy - (height/2))*pix_size*h + x0b + x_camera_offset
yb = (cx - (width/2))*pix_size*h + y0b  + y_camera_offset
print xb, yb
