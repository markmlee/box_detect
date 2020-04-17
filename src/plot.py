import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib
import numpy as np

import csv
import numbers

x1 = []
y1 = []

x2 = []
y2 = []

x3 = []
y3 = []



row_count = 0

with open('/home/rainbow/Desktop/com_compare.csv','r') as csvfile:
    plots = csv.reader(csvfile, delimiter=',')

    for row in plots:
    	row_count = row_count + 1

    	if(row_count >501 and row_count < 1501):

	    	x1.append(-1*float(row[1]))
	    	y1.append(-1*float(row[2]))

	    	x2.append(float(row[4]))
	    	y2.append(float(row[5]))

    	# if(row_count > 160 and row_count < 4800):
    	# 	#print(float(row[4]))
    	# 	x1.append(float(row[4]))
    	# 	y1.append(float(row[5]))

    	# 	x2.append(float(row[6]))
    	# 	y2.append(float(row[7]))


    	# 	x3.append(float(row[8]))
    	# 	y3.append(float(row[9]))
	    	
    	
    	#if(isinstance(row[0],float)):
	    #    x.append(int(row[0]))
        #if(isinstance(row[1],float)):
        # 	y.append(int(row[1]))

    print(row_count)


plt.scatter(x1,y1, label='visual odom', marker='x', color='b')
plt.scatter(x2,y2, label='kinematic reference', marker='x', color='r')
#plt.scatter(x3,y3, label='3', marker='x', color='b')

plt.xlabel('x')
plt.ylabel('y')
plt.title('step position Graph')
plt.legend()
plt.show()