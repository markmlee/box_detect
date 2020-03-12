import matplotlib.pyplot as plt
import csv
import numbers

x1 = []
y1 = []

x2 = []
y2 = []

x3 = []
y3 = []

x4 = []
y4 = []

x5 = []
y5 = []


x6 = []
y6 = []

x7 = []
y7 = []

x8 = []
y8 = []

x9 = []
y9 = []

xTrue = [0.586560, 0.881329, 0.304070, 1.177336, 1.462612, 1.771886, 2.083009, 2.387885, 2.416145]
yTrue = [0.083537, -0.237826, -0.230996, 0.071278, -0.257263, 0.062993, -0.267609, 0.047428, -0.279313]

row_count = 0

with open('/home/rainbow/Desktop/data/step_position_plot_filtered2.csv','r') as csvfile:
        plots = csv.reader(csvfile, delimiter=',')

        count = 0

        for row in plots:

            count = count + 1
            #print(float(row[4]))
            if row[0] is not None and row[0] != "":
                x1.append(float(row[0]))
                y1.append(float(row[1]))

            if row[2] is not None and row[2] != "":
                x2.append(float(row[2]))
                y2.append(float(row[3]))
           
            if row[4] is not None and row[4] != "":
                x3.append(float(row[4]))
                y3.append(float(row[5]))

            if row[6] is not None and row[6] != "":
                x3.append(float(row[6]))
                y3.append(float(row[7]))

            if row[8] is not None and row[8] != "":
                x4.append(float(row[8]))
                y4.append(float(row[9]))

            if row[10] is not None and row[10] != "":
                x5.append(float(row[10]))
                y5.append(float(row[11]))

            if row[12] is not None and row[12] != "":
                x6.append(float(row[12]))
                y6.append(float(row[13]))


            if row[14] is not None and row[14] != "":
                x7.append(float(row[14]))
                y7.append(float(row[15]))

            if row[16] is not None and row[16] != "":
                x8.append(float(row[16]))
                y8.append(float(row[17]))

            if row[18] is not None and row[18] != "":
                x9.append(float(row[18]))
                y9.append(float(row[19]))


            
                
            print(count)
            #if(isinstance(row[0],float)):
            #    x.append(int(row[0]))
                #if(isinstance(row[1],float)):
                #   y.append(int(row[1]))

     

plt.scatter(x1,y1, label='1', marker='.', color='b')
plt.scatter(x2,y2, label='2', marker='.', color='g')
plt.scatter(x3,y3, label='3', marker='.', color='r')

plt.scatter(x4,y4, label='4', marker='.', color='c')
plt.scatter(x5,y5, label='5', marker='.', color='m')
plt.scatter(x6,y6, label='6', marker='.', color='y')

plt.scatter(x7,y7, label='7', marker='.', color='b')
plt.scatter(x8,y8, label='8', marker='.', color='w')
plt.scatter(x9,y9, label='9', marker='.', color='r')


plt.scatter(xTrue,yTrue, label='mean', marker='o', color='k')


plt.xlabel('detected X (m)')
plt.ylabel('detected Y (m)')
plt.title('Step Position Graph')
plt.legend()
plt.show()