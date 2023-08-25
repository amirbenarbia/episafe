import matplotlib.pyplot as plt

# Initialize empty list to hold the values you want to plot
value_last = []

# Read the file line by line
with open("3.txt", "r") as file:
    for line in file:
        parts = line.strip().split('/')
        if len(parts) < 2:
            continue
        measurements = parts[1].split(';')
        value_last.append(float(measurements[-1]))

# Scale the values down to the 0-20 range
value_last_scaled = [(2/5) * xi for xi in value_last]

# Calculate mean and variance of scaled values
mean_scaled = sum(value_last_scaled) / len(value_last_scaled)
variance_scaled = sum((xi - mean_scaled) ** 2 for xi in value_last_scaled) / len(value_last_scaled)

# Calculate "scaled values - mean"
values_minus_mean_scaled = [xi - mean_scaled for xi in value_last_scaled]

# Calculate the difference between each scaled value and the scaled variance
values_minus_variance_scaled = [xi - variance_scaled for xi in value_last_scaled]

# Create variance list with the same length as value_last_scaled for plotting
var_list_scaled = [variance_scaled] * len(value_last_scaled)

# Plot the scaled data
plt.figure(figsize=(10, 6))

plt.plot(value_last_scaled, label='Scaled EDA Values')
plt.plot(var_list_scaled, label='Scaled EDA Variance', linestyle='--')
plt.plot(values_minus_mean_scaled, label='Scaled EDA Values - Mean', linestyle='-.')

plt.xlabel('Index')
plt.ylabel('Scaled Value')
plt.title(f'Plot of Scaled EDA Values, Scaled Variance: {variance_scaled}, and Scaled EDA Values - Mean')
plt.legend()

plt.show()

# Plot the difference between each scaled value and the scaled variance
plt.figure(figsize=(10, 6))

plt.plot(values_minus_variance_scaled, label='Scaled EDA Values - Variance')
plt.axhline(y=0, color='r', linestyle='--')

plt.xlabel('Index')
plt.ylabel('Scaled Difference')
plt.title('Plot of Difference Between Each Scaled EDA Value and Scaled Variance')
plt.legend()

plt.show()
