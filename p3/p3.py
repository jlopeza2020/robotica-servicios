import matplotlib.pyplot as plt
import numpy as np

# Generar 180 medidas del láser (de 0 a 180 grados, en incrementos de 1 grado)
grados = np.arange(0, 181, 1)

# Simular medidas reales del láser (puedes reemplazar esto con tus propios datos)
medidas_laser = np.random.rand(181) * 10  # Simulación de medidas aleatorias en un rango de 0 a 10

# Definir el rectángulo por su largo y ancho de detección
largo_deteccion = (60, 120)  # Definir el rango de largo de detección
ancho_deteccion = (3, 5)     # Definir el rango de ancho de detección

# Filtrar las medidas dentro del rango del rectángulo
indices_rango = np.where(
    (grados >= largo_deteccion[0]) & (grados <= largo_deteccion[1]) &
    (medidas_laser >= ancho_deteccion[0]) & (medidas_laser <= ancho_deteccion[1])
)
print(indices_rango)

# Visualizar las medidas del láser y resaltar el rango del rectángulo
plt.plot(grados, medidas_laser, label='Medidas del láser')
plt.scatter(grados[indices_rango], medidas_laser[indices_rango], color='red', label='Objetos detectados')  # Marcar objetos detectados en rojo

# Configurar la visualización
plt.title('Medidas del Láser y Rectángulo de Detección')
plt.xlabel('Ángulo (grados)')
plt.ylabel('Medidas del Láser')
plt.legend()
plt.grid(True)
plt.show()
