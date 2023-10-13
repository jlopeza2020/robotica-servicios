from queue import PriorityQueue
import numpy as np

# Definir la celda actual y las posibles soluciones
current_cell = np.array([13, 18])
possible_solutions = np.array([[18, 21], [19, 22], [20, 21], [20, 20], [20, 19], [20, 18], [20, 17], [17, 21]])

# Función para calcular la distancia euclidiana entre dos puntos
def euclidean_distance(point1, point2):
    return np.linalg.norm(point1 - point2)

# Función para encontrar la mejor coordenada usando Best-First Search
def best_first_search(start, possible_solutions):
    priority_queue = PriorityQueue()
    for solution in possible_solutions:
        priority_queue.put((euclidean_distance(start, solution), tuple(solution)))  # Convertir a tupla

    best_coordinate = None
    best_distance = float('inf')

    while not priority_queue.empty():
        _, coordinate = priority_queue.get()
        coordinate = np.array(coordinate)  # Convertir nuevamente a array
        distance = euclidean_distance(start, coordinate)
        if distance < best_distance:
            best_coordinate = coordinate
            best_distance = distance

    return best_coordinate

# Encontrar la mejor coordenada
best_coordinate = best_first_search(current_cell, possible_solutions)

# Imprimir el resultado
print("Celda actual:", current_cell)
print("Posibles soluciones:", possible_solutions)
print("La mejor coordenada encontrada es:", best_coordinate)

