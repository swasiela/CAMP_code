import matplotlib.pyplot as plt

# Read Frobenius norms from file
frobenius_norms = []
with open('/home/swasiela/CAMP/src/results/pnorm_profil.txt', 'r') as file:
    for line in file:
        if "Frobenius Norm:" in line:
            norm_value = float(line.split(': ')[1].strip())
            frobenius_norms.append(norm_value)

# Plotting the Frobenius norms
plt.figure(figsize=(10, 6))
plt.plot(frobenius_norms, marker='', linestyle='-', color='b', lw = 3)
plt.xlabel("Time steps",fontsize = 35)
plt.ylabel("Radii P-Norm",fontsize = 35)
plt.tick_params(axis='both', which='major', labelsize=35) 
plt.tick_params(axis='both', which='minor', labelsize=35)
plt.grid(True, which='both', axis='y', linestyle='--', color='gray', alpha=0.7)
plt.grid(True, which='both', axis='x', linestyle='--', color='gray', alpha=0.7)
plt.show()