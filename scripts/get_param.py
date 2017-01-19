#!/usr/bin/python
import csv
import matplotlib.pyplot as plt
import numpy as np
from scipy import stats

a = [];
i=0;
csvReader = csv.reader(open('data1.txt', 'rb'), delimiter=',', quotechar='|');
for row in csvReader:
	a.append(row);

command = np.array([row[0] for row in a]).astype(np.float)
voltage = np.array([row[1] for row in a]).astype(np.float) * 267/1023

slope, intercept, r_value, p_value, std_err = stats.linregress(command,voltage)
print str(slope) + ' ' + str(intercept)

fit = np.polyfit(command, voltage,1)
fit_fn = np.poly1d(fit)

plt.plot(command,voltage, 'rx',command,fit_fn(command),'b')
plt.ylabel('voltage')
plt.xlabel('command')
plt.grid()
plt.show()

# plt.plot(range(len(voltage)),voltage, 'rx')
# plt.ylabel('voltage')
# plt.xlabel('command')
# plt.grid()
# plt.show()



# for i in range(0, len(a)):
# 	print a[i];
