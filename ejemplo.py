import math

# Coordenadas de los puntos
x1, y1 = 21, 19  # Primer punto
x2, y2 = 20, 18  # Segundo punto

# Calcular la arcotangente
arcotangente_rad = math.atan2(y2 - y1, x2 - x1)

# Convertir a grados si es necesario
arcotangente_grados = math.degrees(arcotangente_rad)

print("Arcotangente en radianes:", arcotangente_rad)
print("Arcotangente en grados:", arcotangente_grados)
