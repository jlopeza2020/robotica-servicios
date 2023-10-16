import math

def calcular_arcotangente(punto1, punto2):
    # Puntos representados como tuplas (x, y)
    x1, y1 = punto1
    x2, y2 = punto2
    
    # Calcula la diferencia en coordenadas x e y
    delta_x = x2 - x1
    delta_y = y2 - y1
    
    # Calcula la arcotangente en radianes
    arcotangente_rad = math.atan2(delta_y, delta_x)
    
    # Convierte la arcotangente a grados
    arcotangente_grados = math.degrees(arcotangente_rad)
    
    return arcotangente_rad, arcotangente_grados

# Ejemplo de uso
punto1 = (21, 19)
punto2 = (20, 18)

arcotangente_rad, arcotangente_grados = calcular_arcotangente(punto1, punto2)

print("Arcotangente en radianes:", arcotangente_rad)
print("Arcotangente en grados:", arcotangente_grados)