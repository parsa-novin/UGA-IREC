import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

# data = np.loadtxt(fname = 'Cloud ROSCO.csv', delimiter = ',')
# print(data)

data = pd.read_csv('KJ4SAE01517_04-13-2025_12_59_18.csv')
print(data.head())

