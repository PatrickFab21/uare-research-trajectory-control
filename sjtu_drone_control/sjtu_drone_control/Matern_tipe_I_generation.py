import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from scipy.spatial import distance

# Parámetros
area_size = 100  # Tamaño del área (100x100)
num_points = 30  # Número de usuarios deseados
min_distance = 1  # Distancia mínima entre usuarios

# Generación de puntos candidatos con un proceso de Poisson
lambda_poisson = num_points / (area_size ** 2)  # Densidad esperada
num_candidates = int(1.5 * num_points)  # Generamos más puntos de los necesarios

# Generar candidatos aleatorios en el área
candidates = np.random.uniform(0, area_size, (num_candidates, 2))

# Aplicar el filtro de Matérn tipo-I (exclusión por distancia mínima)
selected_points = []
for point in candidates:
    if all(distance.euclidean(point, p) >= min_distance for p in selected_points):
        selected_points.append(point)
    if len(selected_points) >= num_points:
        break

# Convertir a array numpy
selected_points = np.array(selected_points)

# Gráfica de la distribución
plt.figure(figsize=(6, 6))
plt.scatter(selected_points[:, 0], selected_points[:, 1], c='b', label='Usuarios')
plt.xlim(0, area_size)
plt.ylim(0, area_size)
plt.xlabel('X (m)')
plt.ylabel('Y (m)')
plt.title('Distribución de usuarios (Matérn Tipo-I)')
plt.grid(True)
plt.legend()
plt.show()

# Crear un DataFrame con los puntos generados
df = pd.DataFrame(selected_points, columns=["X", "Y"])
df.index = [f"P{i+1}" for i in range(len(df))]  # Nombres de los puntos

# Guardar en un archivo CSV
csv_filename = "matern_type1_distribution.csv"
df.to_csv(csv_filename, index_label="Point")

print(f"Archivo CSV generado: {csv_filename}")
