#Filtro de Kalman Unscented
# Diseño de Sistemas Inteligentes
# Corte 3
# Universidad Politécnica de Chiapas
# Creado por Luis Fernando Hernández Morales y David Pérez Sánchez
from utils.Unscented import ukf
import matplotlib.pyplot as plt
import numpy as np
import math

def transformada(t):
	"""
	Calcular la función de transición del estado ƒ(x),
	al mismo la función de observación h(x).
	
	Es importante definir la función de transición y de observación.
	`Una de ellas puede ser lineal y la otra no lineal.`
	Parámetros:
	No requiere de parámetros.
	"""
	fx = [
		[_ukf.xs[_ukf.i][0,0] + t * _ukf.xs[_ukf.i][1,0]], # Posición en X
		[_ukf.xs[_ukf.i][1,0]],						   # Velocidad en X
		[_ukf.xs[_ukf.i][2,0] + t * _ukf.xs[_ukf.i][3,0]], # Posición en Y
		[_ukf.xs[_ukf.i][3,0]]						   # Velocidad en Y
	]
	hx = [
		[math.sqrt(_ukf.xs[_ukf.i][0,0]**2 + _ukf.xs[_ukf.i][2,0]**2)], # Distancia del radar
		[math.atan2(_ukf.xs[_ukf.i][2,0],_ukf.xs[_ukf.i][0,0])]			# Ángulo del radar
	]
	return fx, hx

"""
Variables iniciales para el estado
"""
posicion_x_inicial = -2900
posicion_y_inicial = 950
velocidad_x_inicial = 80
velocidad_y_inicial = 20

ruido_p = 100
ruido_q = 0.1
ruido_R = 50
ruido_V = 0.005

instantes_tiempo = 60

estado = [
	[posicion_x_inicial], # Posición en X
	[velocidad_x_inicial],	 # Velocidad en X
	[posicion_y_inicial],   # Posición en Y
	[velocidad_y_inicial]	 # Velocidad en Y
]

P = [
	[ruido_p, 0, 0, 0],
	[0, ruido_p, 0, 0],
	[0, 0, ruido_p, 0],
	[0, 0, 0, ruido_p]
] # nx1

Q = [
	[0, 0, 0, 0],
	[0, ruido_q, 0, 0],
	[0, 0, 0, 0],
	[0, 0, 0, ruido_q]
] # nx1

R = [
	[np.random.normal(0, ruido_R**2), 0],
	[0, np.random.normal(0, ruido_V**2)]
] #mxm => Tamaño de las observaciones

"""
Inicializar la clase del algoritmo de kalman unscented
"""
_ukf = ukf()
_ukf.setX(estado)
_ukf.setP(P)
_ukf.setQ(Q)
_ukf.setR(R)
_ukf.setAlpha(0.001)
_ukf.setBeta(2)
_ukf.setKappa(0)
tiempos = np.arange(0, instantes_tiempo, 0.1)
tiempo_anterior = 0

# Fijar los límites de la gráfica
x = np.linspace(-3000,3000,len(tiempos))
y = np.linspace(1000,800,len(tiempos))
r = np.sqrt(x**2+y**2) # Medida
theta = np.arctan2(y,x) # Medida
ruido_r = np.random.normal(0,ruido_R,len(r))
ruido_theta = np.random.normal(0,ruido_V,len(theta))
vy = np.gradient(y,tiempos) # Dinámica de la velocidad en X
vx = np.gradient(x,tiempos) # Dinámica de la velocidad en Y
_r = r + ruido_r
_theta = theta + ruido_theta

posicion_x_filtrada = []
posicion_y_filtrada = []
velocidad_x_filtrada = []
velocidad_y_filtrada = []

for i in range(len(tiempos)):
	t = tiempos[i]-tiempo_anterior
	tiempo_anterior = tiempos[i]
	z = [
		[_r[i]], # Observación del sensor => distancia
		[_theta[i]] # Observación del sensor => distancia
	]
	_ukf.step(z, transformada, t)
	x_filtrada = _ukf.getX()
	posicion_x_filtrada.append(x_filtrada[0,0])
	velocidad_x_filtrada.append(x_filtrada[1,0])
	posicion_y_filtrada.append(x_filtrada[2,0])
	velocidad_y_filtrada.append(x_filtrada[3,0])

posicionesX = np.array(posicion_x_filtrada)
posicionesY = np.array(posicion_y_filtrada)
velocidadesX = np.array(velocidad_x_filtrada)
velocidadesY = np.array(velocidad_y_filtrada)

"""
Graficar
"""

plt.subplot(411)
plt.plot(tiempos,x,'b',label='Truth')
plt.plot(tiempos,posicion_x_filtrada,'r',label='UKF')
plt.title('X position')
plt.xticks([])
plt.subplot(412)
plt.plot(tiempos,vx,'b',label='Truth')
plt.plot(tiempos,velocidad_x_filtrada,'r',label='UKF')
plt.title('X velocity')
plt.xticks([])
plt.subplot(413)
plt.plot(tiempos,y,'b',label='Truth')
plt.plot(tiempos,posicion_y_filtrada,'r',label='UKF')
plt.title('Y position')
plt.xticks([])
plt.subplot(414)
plt.plot(tiempos,vy,'b',label='Truth')
plt.plot(tiempos,velocidad_y_filtrada,'r',label='UKF')
plt.title('Y velocity')
plt.legend()
plt.figure()
plt.plot(x,y,'b',label='Truth')
plt.plot(posicion_x_filtrada,posicion_y_filtrada,'r',label='UKF')
plt.title('XY Position')
plt.legend()
plt.show()
