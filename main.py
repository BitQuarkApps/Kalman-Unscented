#Filtro de Kalman Unscented
# Diseño de Sistemas Inteligentes
# Corte 3
# Universidad Politécnica de Chiapas
# Creado por Luis Fernando Hernández Morales y David Pérez Sánchez
from utils.Unscented import KalmanUnscented
import matplotlib.pyplot as plt
import numpy as np
import math

def dinamica(t):
	"""
	Calcular la función de transición del estado ƒ(x),
	al mismo la función de observación h(x).
	
	Es importante definir la función de transición y de observación.
	`Una de ellas puede ser lineal y la otra no lineal.`
	Parámetros:
	No requiere de parámetros.
	"""
	# fx = [
	# 	[ukf.xs[ukf.i][0,0]],
	# 	[ukf.xs[ukf.i][1,0]],
	# 	[ukf.xs[ukf.i][2,0]],
	# 	[ukf.xs[ukf.i][3,0]]
	# ] # nx1
	# hx = [
	# 	[ukf.xs[ukf.i][0,0]],
	# 	[ukf.xs[ukf.i][1,0]]
	# ] # mx1 => m = Tamaño de la observación
	fx = [
		[ukf.xs[ukf.i][0,0] * math.cos(angulo_lanzamiento) * t],
		[ukf.xs[ukf.i][1,0] * math.sin(angulo_lanzamiento) * tiempo[i] + ( 1/2 * gravedad * math.pow(tiempo[i], 2) )],
		[ukf.xs[ukf.i][2,0]],
		[ukf.xs[ukf.i][3,0]]
	] # nx1
	hx = [
		[ukf.xs[ukf.i][0,0] * math.cos(angulo_lanzamiento)],
		[ukf.xs[ukf.i][1,0] * math.sin(angulo_lanzamiento) - gravedad * t]
	] # mx1 => m = Tamaño de la observación
	return fx, hx


"""
Establecer las variables iniciales
"""
angulo_lanzamiento = 50  # grados
velocidad_incial = 5  # m/s => V0
gravedad = 9.81 # m/s
x_inicial = 0
y_inicial = 10
ruido_R = 0.003
iteraciones = 25

estado = [
	[x_inicial], # x
	[y_inicial], # y
	[0], # vx
	[velocidad_incial], # vy
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
	[np.random.normal(0, ruido_R), 0],
	[0, np.random.normal(0, ruido_R)]
] #mxm => Tamaño de las observaciones

"""
Inicializar la clase del algoritmo de kalman unscented
"""
ukf = KalmanUnscented()
ukf.set_x(estado)
ukf.set_p(P)
ukf.set_q(Q)
ukf.set_r(R)

PROCESO_REAL_X = []
PROCESO_REAL_Y = []
FILTRADA_X = []
FILTRADA_Y = []
PREDICHA_X = []
PREDICHA_Y = []


# Instantes de tiempo
tiempo = np.arange(1, iteraciones, 1) # Arreglo de 1 hasta iteraciones en saltos de 1
"""
Dinámica del problema
"""
# x = ( velocidad_incial * math.cos(angulo_lanzamiento) ) * tiempo[0] # x = ( V0 cos α ) * t
# y = ( velocidad_incial * math.sin(angulo_lanzamiento) * tiempo[0] ) + ( 1/2 * gravedad * math.pow(tiempo[0], 2) ) # y = ( V0 sin α ) * t + 1/2 * g * t^2
# ruido_dinamica = np.random.normal(0,ruido_R)
# PROCESO_REAL_X.append(x + ruido_dinamica)
# PROCESO_REAL_Y.append(y + ruido_dinamica)

for i in range(len(tiempo)):
	# Proceso real
	x_ = ( velocidad_incial * math.cos(angulo_lanzamiento) ) * tiempo[i] # x = ( V0 cos α ) * t
	y_ = ( velocidad_incial * math.sin(angulo_lanzamiento) * tiempo[i] ) + ( 1/2 * gravedad * math.pow(tiempo[i], 2) ) # y = ( V0 sin α ) * t + 1/2 * g * t^2
	ruido_dinamica = np.random.normal(0,ruido_R)
	PROCESO_REAL_X.append(x_ + ruido_dinamica)
	PROCESO_REAL_Y.append(y_ + ruido_dinamica)
	
	"""
	Realizar observaciones Z
	"""
	Z = [
			[ velocidad_incial * math.cos(angulo_lanzamiento) ], #Vx = v0 cos θ0
			[ velocidad_incial * math.sin(angulo_lanzamiento) - gravedad * tiempo[i] ], #Vy = v0 sin θ0 - g * t
		]

	ukf.iteracion(Z, dinamica, tiempo[i])
	_filtrada = ukf.get_filtrada()
	_predicha = ukf.get_prediccion()
	FILTRADA_X.append(_filtrada[0,0])
	FILTRADA_Y.append(_filtrada[1,0])
	PREDICHA_X.append(_predicha[0,0])
	PREDICHA_Y.append(_predicha[1,0])

plt.plot(FILTRADA_X, FILTRADA_Y, 'r-', label="Filtrada")
plt.plot(PREDICHA_X, PREDICHA_Y, 'b-', label="Predicha")
plt.title('Observaciones del sensor')
plt.legend()
plt.show()
