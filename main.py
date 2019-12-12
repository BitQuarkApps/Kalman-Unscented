#Filtro de Kalman Unscented
# Diseño de Sistemas Inteligentes
# Corte 3
# Universidad Politécnica de Chiapas
# Creado por Luis Fernando Hernández Morales y David Pérez Sánchez
from utils.Unscented import ukf
import numpy as np
import matplotlib.pyplot as plt
import math
import copy


def funciones_fx_hx(t):
	fx = [[ukf.xs[ukf.i][0,0] + t * ukf.xs[ukf.i][1,0]],
		  [ukf.xs[ukf.i][1,0]],
		  [ukf.xs[ukf.i][2,0] + t * ukf.xs[ukf.i][3,0]],
		  [ukf.xs[ukf.i][3,0]]]
	hx = [[math.sqrt(ukf.xs[ukf.i][0,0]**2 + ukf.xs[ukf.i][2,0]**2)],
		  [math.atan2(ukf.xs[ukf.i][2,0],ukf.xs[ukf.i][0,0])]]
	return fx, hx


delta_t = 1
posicion_x = 500
posicion_y = 100
velocidad_x = 20
velocidad_y = 5
iteraciones = 30
ruido_posicion = 0.00005
ruido_velocidad = 0.00003
ruido_medicion_distancia = 0.0005
ruido_medicion_angulo = 0.0001

time = np.arange(0, iteraciones, delta_t)

F = np.array([
	[1, delta_t, 0, 0],
	[0, 1, 0, 0],
	[0, 0, 1, delta_t],
	[0, 0, 0, 1],
])

Xt = np.array([
	[ posicion_x ],
	[ velocidad_x ],
	[ posicion_y ],
	[ velocidad_y ]
])

Xt_inicial = copy.deepcopy(Xt)

real_x = []
real_y = []
velocidad_real_x = []
velocidad_real_y = []

real_x.append(posicion_x)
real_y.append(posicion_y)
velocidad_real_x.append(velocidad_x)
velocidad_real_y.append(velocidad_y)
for i in range(len(time)-1):
	Wt = np.array([
		[ np.random.normal(0, ruido_posicion) ],
		[ np.random.normal(0, ruido_velocidad) ],
		[ np.random.normal(0, ruido_posicion) ],
		[ np.random.normal(0, ruido_velocidad) ]
	])
	_xt = np.dot(F, Xt) + Wt
	f_Xt = _xt
	Xt = copy.deepcopy(_xt)
	real_x.append(f_Xt[0,0])
	real_y.append(f_Xt[2,0])

	velocidad_real_x.append(f_Xt[1,0])
	velocidad_real_y.append(f_Xt[3,0])

real_x = np.array(real_x)
real_y = np.array(real_y)
velocidad_real_x = np.array(velocidad_real_x)
velocidad_real_y = np.array(velocidad_real_y)
"""
Algoritmo Unscented
"""
ukf = ukf()
P = [
	[ ruido_posicion, 0, 0, 0 ],
	[ 0, ruido_velocidad, 0, 0 ],
	[ 0, 0, ruido_posicion, 0 ],
	[ 0, 0, 0, ruido_velocidad ]
]

Q = [
	[ ruido_posicion**2, 0, 0, 0 ],
	[ 0, ruido_velocidad**2, 0, 0 ],
	[ 0, 0, ruido_posicion**2, 0 ],
	[ 0, 0, 0, ruido_velocidad**2 ]
]
R = [
	[ ruido_medicion_distancia**2, 0 ],
	[ 0, ruido_medicion_angulo**2 ]
]
ukf.setX(Xt_inicial)
ukf.setP(P)
ukf.setQ(Q)
ukf.setR(R)

prev = 0
"""
Matrices predichas y filtradas
"""
x_f = [] # X filtrada
y_f = [] # Y filtrada
xv_f = [] # vx filtrada
yv_f = [] # vy filtrada
x_p = []  # X predicha
y_p = []  # Y predicha
xv_p = [] # vx predicha
yv_p = [] # vy predicha

r_raw = np.sqrt(real_x**2+real_y**2) # Valores de la raíz de X^2+Y^2
theta_raw = np.arctan2(real_y,real_x) # Valores de la arcotangente y,x
r_noise = np.random.normal(0,ruido_medicion_distancia,len(r_raw)) # Ruido para la medición de la distancia
theta_noise = np.random.normal(0,ruido_medicion_angulo,len(theta_raw)) # Ruido para la medición del ángulo
r = r_raw + r_noise
theta = theta_raw + theta_noise

for i in range(len(time)):
	t = float(time[i]) - prev
	prev = float(time[i])
	z = [[r[i]], # Observaciones del sensor
		 [theta[i]]]
	ukf.step(z, funciones_fx_hx, t) # Proceso de puntos sigmas, predicción y actualización
	x_filtered = ukf.getX() # Filtrada del estado
	prediction = ukf.xp # Predicción del estado
	
	x_p.append(prediction[0,0])
	y_p.append(prediction[2,0])
	xv_p.append(prediction[1,0])
	yv_p.append(prediction[3,0])

	x_f.append(x_filtered[0,0])
	xv_f.append(x_filtered[1,0])
	y_f.append(x_filtered[2,0])
	yv_f.append(x_filtered[3,0])

x_f = np.array(x_f)
xv_f = np.array(xv_f)
y_f = np.array(y_f)
yv_f = np.array(yv_f)
x_p = np.array(x_p)
y_p = np.array(y_p)
xv_p = np.array(xv_p)
yv_p = np.array(yv_p)

"""
Graficar
"""
plt.plot(real_x, real_y, label="Real")
plt.plot(x_p, y_p, label="Predicha", linestyle=":")
plt.plot(x_f, real_y, label="Filtrada", linestyle="--")
plt.title('Posición del objeto')
plt.legend()
# plt.figure()
# plt.plot(velocidad_real_x, velocidad_real_y, 'r:', label="Real")
# plt.plot(xv_p, yv_p, 'b.', label="Predicha")
# plt.plot(xv_f, xv_f, 'g-', label="Filtrada")
# plt.title('Velocidades del objeto')
# plt.legend()
plt.show()