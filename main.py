#Filtro de Kalman Unscented
# Diseño de Sistemas Inteligentes
# Corte 3
# Universidad Politécnica de Chiapas
# Creado por Luis Fernando Hernández Morales y David Pérez Sánchez
from utils.Unscented import KalmanUnscented
import matplotlib.pyplot as plt
import numpy as np

def dinamica():
	"""
	Calcular la función de transición del estado ƒ(x),
	al mismo la función de observación h(x).
	
	Es importante definir la función de transición y de observación.
	`Una de ellas puede ser lineal y la otra no lineal.`
	Parámetros:
	No requiere de parámetros.
	"""
	fx = None
	hx = None
	return fx, hx


"""
Establecer las variables iniciales
"""
angulo_lanzamiento = 50  # grados
velocidad_incial = 5  # m/s => V0
x_inicial = 0
y_inicial = 10
ruido_R = 0.0003
iteraciones = 50

estado = [
	[x_inicial], # x
	[y_inicial], # y
	[0], # vx
	[0], # vy
] # nx1

P = [
	[0.1, 0, 0, 0],
	[0, 0.1, 0, 0],
	[0, 0, 0.1, 0],
	[0, 0, 0, 0.1]
] # nx1

Q = [
	[0.0001, 0, 0, 0],
	[0, 0.0001, 0, 0],
	[0, 0, 0.0001, 0],
	[0, 0, 0, 0.0001]
] # nx1

R = [
	[np.random.normal(0, ruido_R), 0, 0, 0, 0],
	[0, np.random.normal(0, ruido_R), 0, 0, 0],
	[0, 0, np.random.normal(0, ruido_R), 0, 0],
	[0, 0, 0, np.random.normal(0, ruido_R), 0],
	[0, 0, 0, 0, np.random.normal(0, ruido_R)]
] #mxm => (n+1)(n+1)

"""
Inicializar la clase del algoritmo de kalman unscented
"""
ukf = KalmanUnscented()
ukf.set_x(estado)
ukf.set_p(P)
ukf.set_q(Q)
ukf.set_r(R)

# Instantes de tiempo
tiempo = np.arange(0, iteraciones, 1) # Arreglo de 0 hasta iteraciones en saltos de 1