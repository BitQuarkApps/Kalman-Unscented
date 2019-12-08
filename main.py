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

angulo_lanzamiento = 50  # grados
velocidad_incial = 5  # m/s
"""
Cuando la velocidad del objeto llega a 0, quiere decir que se encuentra en su altura máxima.

Después de eso, las velocidades en el eje y del objeto tienden a reducir.

[ Descomposición de la velocidad en sus dos componentes ]

"""