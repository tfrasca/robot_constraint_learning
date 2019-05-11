import matplotlib.pyplot as plt
import csv

filename = "data.csv"
data = []
with open(filename, "r") as f:
  reader = csv.reader(f, delimiter=",")
  for row in reader:
    data.append(row)

for row in range(len(data)):
  plt.plot(data[row])
  plt.xlabel("episodes")
  if row % 2:
    plt.ylabel("numSteps")
  else:
    plt.ylabel("reward")
  plt.show()
