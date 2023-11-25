import numpy as np

def calcular_pendiente(puntos):
    # Extraer las coordenadas x e y de los puntos
    x = np.array([p[0] for p in puntos])
    y = np.array([p[1] for p in puntos])

    # Calcular la pendiente usando la fórmula (cambio en y) / (cambio en x)
    pendiente = np.diff(y) / np.diff(x)

    # Devolver la pendiente promedio si hay más de un punto
    return np.std(pendiente)

# Ejemplo de puntos bidimensionales
puntos_ejemplo = [(1, 2), (2, 2), (3, 2), (4,2)]

# Calcular la pendiente
pendiente_calculada = calcular_pendiente(puntos_ejemplo)

print("Pendiente calculada:", pendiente_calculada)
