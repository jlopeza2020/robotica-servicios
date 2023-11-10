import numpy as np
import matplotlib.pyplot as plt

# Coordenadas iniciales
initial_x = 40
initial_y = -29.3

# Parámetros para la espiral
a = 5  # Puedes ajustar este parámetro según sea necesario
b = 0.2  # Puedes ajustar este parámetro según sea necesario
t = np.linspace(0, 30, 10000)  # Valores de t desde 0 hasta 10

# Calcular las coordenadas de la espiral
x = a * t * np.cos(t)
y = a * t * np.sin(t)

# Ajustar las coordenadas para iniciar desde el punto inicial
x += initial_x
y += initial_y

# Graficar la espiral
plt.plot(x, y, label='Espiral de Arquímedes')
plt.scatter(initial_x, initial_y, color='red', label='Punto Inicial')
plt.title('Espiral de Arquímedes que Comienza en el Punto Inicial')
plt.xlabel('X-axis')
plt.ylabel('Y-axis')
plt.legend()
plt.show()
