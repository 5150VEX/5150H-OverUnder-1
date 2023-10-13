import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

data = pd.read_csv('vs.csv', encoding='UTF-16', sep=',')

reading = data["reading"]
setpoint = data["setpoint"]

plt.plot(setpoint, label="setpoint")
plt.plot(reading, label="reading")
plt.legend(loc="lower right")
plt.xlabel("time")
plt.ylabel("cm")

plt.show()