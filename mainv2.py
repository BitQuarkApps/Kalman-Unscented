#Filtro de Kalman Unscented
# Diseño de Sistemas Inteligentes
# Corte 3
# Universidad Politécnica de Chiapas
# Creado por Luis Fernando Hernández Morales y David Pérez Sánchez
from utils.Unscented_v2 import Unscented
import numpy as np
import matplotlib.pyplot as plt
import math
import copy


delta_t = 1
posicion_x = 0
posicion_y = 0
velocidad_x = 25
velocidad_y = 5
iteraciones = 30
ruido_posicion = 10/3
ruido_velocidad = 0.5/3
ruido_medicion_distancia = 0.005
ruido_medicion_angulo = 0.005

time = np.arange(0, iteraciones, delta_t)


# ----------------------- D I N Á M I C A -----------------------
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
x_p = []  # X predicha
x_f = [] # X filtrada
y_p = []  # Y predicha
y_f = [] # Y filtrada
velocidad_real_x = []
velocidad_real_y = []
xv_p = []  # VX predicha
xv_f = [] # VX filtrada
yv_p = []  # VY predicha
yv_f = [] # VY filtrada

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

r_raw = np.sqrt(real_x**2+real_y**2) # Valores de la raíz de X^2+Y^2
theta_raw = np.arctan2(real_y,real_x) # Valores de la arcotangente y,x
r_noise = np.random.normal(0,ruido_medicion_distancia,len(r_raw)) # Ruido para la medición de la distancia
theta_noise = np.random.normal(0,ruido_medicion_angulo,len(theta_raw)) # Ruido para la medición del ángulo
r = r_raw + r_noise
theta = theta_raw + theta_noise
# ------------- F I N  D I N A M I C A --------------

# ---------------- U N S C E N T E D ----------------
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
ukf = Unscented(Xt_inicial, P, Q, R)
for i in range(len(time)):
	z = [[r[i]], # Observaciones del sensor
		 [theta[i]]]
	predicha, filtrada = ukf.iteracion(z, delta_t)
	x_p.append(predicha[0,0])
	y_p.append(predicha[2,0])
	x_f.append(filtrada[0,0])
	y_f.append(filtrada[2,0])

	xv_p.append(predicha[1,0])
	yv_p.append(predicha[3,0])

	xv_f.append(filtrada[1,0])
	yv_f.append(filtrada[3,0])


# ------------- F I N  U N S C E N T E D ------------

# ----------------- G R A F I C A R -----------------
plt.plot(real_x, real_y, label="Real")
plt.plot(x_p, y_p, label="Predicha", linestyle=":")
plt.plot(x_f, y_f, label="Filtrada", linestyle="--")
plt.title('Posición del objeto')
plt.legend()
plt.figure()
plt.plot(time, velocidad_real_x, label="Real")
plt.plot(time, xv_p, label="Predicha", linestyle=":")
plt.plot(time, xv_f, label="Filtrada", linestyle="--")
plt.title('Velocidad en X del objeto')
plt.legend()
plt.figure()
plt.plot(time, velocidad_real_y, label="Real")
plt.plot(time, yv_p, label="Predicha", linestyle=":")
plt.plot(time, yv_f, label="Filtrada", linestyle="--")
plt.title('Velocidad en Y del objeto')
plt.legend()
plt.show()
# ------------- F I N  G R A F I C A R --------------