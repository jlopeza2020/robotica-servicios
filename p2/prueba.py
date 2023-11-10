import numpy as np
import matplotlib.pyplot as plt

# Función para calcular las coordenadas de la espiral de Arquímedes creciente
def arquimedes_spiral_creciente(a, b, theta):
    r = a + b * theta  # Ajusta según sea necesario
    x = r * np.cos(theta)
    y = r * np.sin(theta)
    return x, y

# Parámetros de la espiral
a = 0.2
b = 0.1

# Generar puntos en la espiral
theta_vals = np.linspace(0, 4 * np.pi, 1000)
print(theta_vals)
x_vals, y_vals = arquimedes_spiral_creciente(a, b, theta_vals)

# Dibujar la espiral
plt.plot(x_vals, y_vals)
plt.title('Espiral de Arquímedes Creciente')
plt.xlabel('X')
plt.ylabel('Y')
plt.grid(True)
plt.show()