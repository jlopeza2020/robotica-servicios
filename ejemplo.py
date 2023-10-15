def obtener_punto_siguiente():
    # Esta función debería devolver las coordenadas del siguiente punto en el plan
    # Supondremos que devuelve una tupla (x, y)
    return (20.0, 19.0)  # Ejemplo de coordenadas

def celda_a_pixel(celda_x, celda_y, ancho_celda, alto_celda):
    pixel_x = celda_x * ancho_celda + ancho_celda / 2
    pixel_y = celda_y * alto_celda + alto_celda / 2
    return (pixel_x, pixel_y)

def pixel_a_coordenada_mundo(pixel_x, pixel_y):
    mundo_x = pixel_x * 0.02 # Supongamos que 1 pixel equivale a 1/50 unidad en el mundo
    mundo_y = pixel_y * 0.02
    return (mundo_x, mundo_y)

def coordenada_mundo_a_relativa(coordenada_mundo, posicion_robot):
    relativa_x = coordenada_mundo[0] - posicion_robot[0]
    relativa_y = coordenada_mundo[1] - posicion_robot[1]
    return (relativa_x, relativa_y)

# Ejemplo de uso
celda_x = 21
celda_y = 19
ancho_celda = 16
alto_celda = 16

pixel_central = celda_a_pixel(celda_x, celda_y, ancho_celda, alto_celda)
print("Pixel central:", pixel_central)

coordenada_mundo = pixel_a_coordenada_mundo(*pixel_central)
print("Coordenada del mundo:", coordenada_mundo)

posicion_robot = (30.0, 35.0)
coordenada_relativa = coordenada_mundo_a_relativa(coordenada_mundo, posicion_robot)
print("Coordenada relativa al robot:", coordenada_relativa)

# Obtener el siguiente punto del plan y convertirlo a coordenada relativa al robot
siguiente_punto = obtener_punto_siguiente()
siguiente_punto_pixel = celda_a_pixel(*siguiente_punto, ancho_celda, alto_celda)
siguiente_punto_mundo = pixel_a_coordenada_mundo(*siguiente_punto_pixel)
siguiente_punto_relativo = coordenada_mundo_a_relativa(siguiente_punto_mundo, posicion_robot)
print("Siguiente punto relativo al robot:", siguiente_punto_relativo)
