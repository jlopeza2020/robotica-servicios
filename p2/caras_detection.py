import cv2
import time
import copy

# Crear el objeto clasificador de caras
face_detector = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

# Verificar si el clasificador se ha cargado correctamente
if face_detector.empty():
    print("Error al cargar el clasificador.")
    exit()

# Cargar la imagen
img = cv2.imread('ejemplo5.png')

# Verificar si la imagen se ha cargado correctamente
if img is None:
    print("Error al cargar la imagen.")
    exit()

# Convertir la imagen a escala de grises
gray_original = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

# Definir el ángulo de rotación inicial y el incremento
angulo_inicial = 0
angulo_incremento = 20

# Iterar hasta alcanzar 360 grados
while angulo_inicial < 360:
    # Crear una copia de la imagen original
    img_copia = copy.deepcopy(img)

    # Realizar la rotación en la imagen copia
    rows, cols, _ = img_copia.shape
    matriz_rotacion = cv2.getRotationMatrix2D((cols / 2, rows / 2), angulo_inicial, 1)
    img_copia = cv2.warpAffine(img_copia, matriz_rotacion, (cols, rows))

    # Convertir la imagen copia a escala de grises
    gray_copia = cv2.cvtColor(img_copia, cv2.COLOR_BGR2GRAY)

    # Realizar la detección de caras en la imagen rotada
    faces_result = face_detector.detectMultiScale(gray_copia)

    # Dibujar rectángulos alrededor de las caras detectadas
    for (x, y, w, h) in faces_result:
        img_copia = cv2.rectangle(img_copia, (x, y), (x+w, y+h), (255, 0, 0), 2)

    # Mostrar la imagen con las caras detectadas
    cv2.imshow('img', img_copia)
    cv2.waitKey(500)  # Esperar 0.5 segundos antes de pasar al siguiente ángulo

    # Incrementar el ángulo de rotación
    angulo_inicial += angulo_incremento

# Cerrar todas las ventanas al finalizar
cv2.destroyAllWindows()
