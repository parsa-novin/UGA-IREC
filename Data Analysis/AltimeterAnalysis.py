import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

# data = np.loadtxt(fname = 'Cloud ROSCO.csv', delimiter = ',')
# print(data)

data = pd.read_csv('Cloud ROSCO.csv')
print(data.head())
