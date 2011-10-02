# stereo vision calculation
# x1 is from camera A (left), x2 from right camera

import math

B=2.7 # cm between cameras
P=320 # image width/2 pixels wide
M=1.3613568165555769/2 # FOV/2 in rad. 78 degrees=FOV on logitech pro webcams

def depth(x1, x2, baseline=B, max_pixels=P, max_theta=M):
	if (x1 < max_pixels) and (x2 < max_pixels):
		# object in left portion of screen on both images
		# pass these values, no changes needed
		print "lefthalf"
		return depth_lefthalf(x1, x2, baseline, max_pixels, max_theta)

	elif (x1 > max_pixels) and (x2 > max_pixels):
		# object is in right portion of screen on both images
		print "righthalf"
		return depth_righthalf(x1, x2, baseline, max_pixels, max_theta)
	elif (x1 <= max_pixels) and (x2 >= max_pixels):
		# object in middle of screen
		print "middle"
		return depth_middle(x1, x2, baseline, max_pixels, max_theta)
	else:
		# unknown - frames taken too far apart?
		print "x1", x1, "x2", x2
		return None, None
		

def depth_lefthalf(x1, x2, baseline, max_pixels, max_theta):
	inv = math.pi/2
	theta_a = inv+find_theta(x1)
	theta_b = inv-find_theta(x2)
	#print "a", theta_a, "b", theta_b
	top = (baseline / 2) * math.sin(theta_a)
	g = math.pi - (theta_a + theta_b)
	bottom = math.sin(g/2)
	if bottom == 0: return None, None
	return abs(top / bottom), -(inv-(math.pi-(g/2+theta_a)))

def depth_righthalf(x1, x2, baseline, max_pixels, max_theta):
	inv = math.pi/2
	theta_a = inv-find_theta(x1)
	theta_b = inv+find_theta(x2)
	#print "a", theta_a, "b", theta_b
	top = (baseline / 2) * math.sin(theta_b)
	g = math.pi - (theta_b + theta_a)
	bottom = math.sin(g/2)
	if bottom == 0: return None, None
	return abs(top / bottom), (inv-(math.pi-(g/2+theta_b)))

def depth_middle(x1, x2, baseline, max_pixels, max_theta):
	inv = math.pi/2
	theta_a = inv-find_theta(x1)
	theta_b = inv-find_theta(x2)
	#print "a", theta_a, "b", theta_b
	top = (baseline / 2) * math.sin(theta_a)
	g = math.pi - (theta_a + theta_b)
	bottom = math.sin(g/2)
	if bottom == 0: return None, None
	return abs(top / bottom), (inv-(math.pi-(theta_a+g/2)))

def find_theta(x, baseline=B, max_pixels=P, max_theta=M):
	# set x to the offset from the middle
	if x > max_pixels:
		x = x - max_pixels
	else:
		x = max_pixels - x
	w = max_pixels / math.tan(max_theta)
	theta = math.atan( (x) / (w) )
	return theta
