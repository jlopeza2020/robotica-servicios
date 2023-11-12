import math

def calcular_distancia(coord1, coord2):
    """Calcula la distancia euclidiana entre dos coordenadas."""
    return math.sqrt((coord1[0] - coord2[0])**2 + (coord1[1] - coord2[1])**2)

def agregar_coordenada(coord, lista_coordenadas):
    """Agrega una coordenada a la lista si no está dentro de un radio de 3 metros de ninguna coordenada existente."""
    for existente_coord in lista_coordenadas:
        distancia = calcular_distancia(coord, existente_coord)
        if distancia <= 4.5:
            print(f"La coordenada {coord} está a {distancia} metros de {existente_coord}. No se agregará.")
            return lista_coordenadas
    print(f"Agregando la coordenada {coord}.")
    lista_coordenadas.append(coord)
    return lista_coordenadas

# Ejemplo de uso:
coordenadas_vistas = []

nuevas_coordenadas = [(40.272686  , -31.92899704  , 2.99911523)
,(40.33443451, -38.01557541  , 3.07704949) 
,(38.31363678 ,-26.93115425  , 2.96052766) 
,(36.30549622 ,-30.07313728 ,  3.0340519 ) 
,(36.24459076, -35.19386673 ,  3.09911633)
,(32.26813126 ,-38.24960709  , 3.01738071) 
,(26.23882484 ,-34.03120422 ,  3.09855485) 
,(22.26299286 ,-35.52638626 ,  2.94537044)]


# Imprimir los resultados que cumplen con la distancia mínima
for resultado in nuevas_coordenadas:
    coordenadas_vistas = agregar_coordenada(resultado, coordenadas_vistas)

# Imprimir la lista final de coordenadas
print("Final Coords:")
for coord in coordenadas_vistas:
    print(coord)
