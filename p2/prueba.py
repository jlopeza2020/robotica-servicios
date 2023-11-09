import matplotlib.pyplot as plt

# Función para generar puntos de la espiral cuadrada
def square_spiral_points(size, turns, side_length_increment):
    x, y = [0], [0]
    angle = 0

    for _ in range(turns):
        for _ in range(4):
            angle_rad = angle * (3.141592653589793 / 180.0)
            x.append(x[-1] + size * side_length_increment * (1 if angle % 180 == 0 else -1) * round(abs(angle % 180 - 90) / 90))
            y.append(y[-1] + size * side_length_increment * (1 if (angle + 90) % 180 == 0 else -1) * round(abs((angle + 90) % 180 - 90) / 90))
            angle += 90

    # Centrar la espiral restando la media de las coordenadas
    x = [coord - sum(x) / len(x) for coord in x]
    y = [coord - sum(y) / len(y) for coord in y]

    return x, y

# Parámetros de la espiral
size = 10
turns = 20
side_length_increment = 5

# Generar puntos de la espiral cuadrada
x, y = square_spiral_points(size, turns, side_length_increment)

# Graficar la espiral cuadrada
plt.plot(x, y, marker='o', linestyle='-')
plt.title('Espiral Cuadrada')
plt.xlabel('Coordenada X')
plt.ylabel('Coordenada Y')
plt.grid(True)
plt.show()
