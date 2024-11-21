import matplotlib.pyplot as plt
from datetime import datetime



numbers = []
file_data = open("./data.txt", "r")
for line in file_data.readlines():
    split = line.split(" ")
    value = split[len(split)-1]
    numbers.append(float(value.strip()))
    
file_data.close()   


x_values = range(len(numbers))

# Create a scatter plot
plt.scatter(x_values, numbers)
plt.title(datetime.now().strftime("%H:%M:%S"))
plt.xlabel("Index")
plt.ylabel("Value")

plt.savefig("output_plot.png")

print("Done")
#plt.show()