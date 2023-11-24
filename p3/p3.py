import numpy as np

# Supongamos que tienes una lista de puntos en el formato (x, y)
# Puedes modificar esta lista con tus propios puntos
puntos = [(1, 2), (2, 4), (3, 6), (4, 8), (5, 10)]

# Convierte la lista de puntos en un array de NumPy para facilitar los cálculos
datos = np.array(puntos)

# Centra los datos restando la media de cada columna
datos_centrales = datos - np.mean(datos, axis=0)

# Calcula la matriz de covarianza
covarianza_matrix = np.cov(datos_centrales, rowvar=False)

# Calcula los autovectores y autovalores
autovalores, autovectores = np.linalg.eigh(covarianza_matrix)

# Ordena los autovalores y autovectores de mayor a menor
orden = np.argsort(autovalores)[::-1]
autovalores = autovalores[orden]
autovectores = autovectores[:, orden]

# Las componentes principales son los autovectores
componente_principal_1 = autovectores[:, 0]
componente_principal_2 = autovectores[:, 1]

print("Componente Principal 1:", componente_principal_1)
print("Componente Principal 2:", componente_principal_2)
