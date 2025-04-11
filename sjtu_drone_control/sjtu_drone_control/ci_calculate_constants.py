import numpy as np

# Given constants
rho = 1.225  # kg/m^3 (air density at sea level)
s = 0.127  # Rotor solidity
delta = 0.012  # Blade profile drag coefficient
k = 0.15  # Induced power correction factor
d0 = 1.155  # Parasite drag ratio
R = 14* 2.54 / 100  # Convert cm to meters (0.3556 m)
Omega = 22*250*2*np.pi/60  # rad/s (maximum angular velocity)
print(f"Omega = {Omega:.4f} rad/s")
W = 2*9.81  # N (total weight of the drone)

# Compute rotor disc area
A = np.pi * R**2

# Compute tip speed (U_tip)
U_tip = Omega * R

# Compute constants
C1 = (delta / 8) * rho * s * A * Omega**3 * R**3
C2 = 3 / (U_tip**2)
C3 = (1 + k) * W**(3/2) / np.sqrt(2 * rho * A)
C4 = W / (rho * A)
C5 = (1/2) * d0 * rho * s * A

# Display results
print(f"C1 = {C1:.4f} W")
print(f"C2 = {C2:.8f} (m/s)^-2")
print(f"C3 = {C3:.4f} W")
print(f"C4 = {C4:.4f} (m/s)^2")
print(f"C5 = {C5:.8f} (m/s)^-3 * W")
