import math

def convertir_orientaciones(orientaciones):
    # Convierte cada orientación en el primer sistema al segundo sistema restando π.
    orientaciones_convertidas = [orientacion - math.pi for orientacion in orientaciones]
    return orientaciones_convertidas

# Ejemplo de uso:
orientaciones_sistema1 = [math.pi, -math.pi/2, 0, math.pi/2]
orientaciones_sistema2 = convertir_orientaciones(orientaciones_sistema1)
print("Orientaciones en el primer sistema:", orientaciones_sistema1)
print("Orientaciones convertidas al segundo sistema:", orientaciones_sistema2)

