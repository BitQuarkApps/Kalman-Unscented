#Filtro de Kalman Unscented
# Diseño de Sistemas Inteligentes
# Corte 3
# Universidad Politécnica de Chiapas
# Creado por Luis Fernando Hernández Morales y David Pérez Sánchez
from utils.Unscented import ukf
import numpy as np
import matplotlib.pyplot as plt
import math

def preparar_algoritmo():
	# Matriz del estado
	ukf.setX([[punto_inicio_x],
			  [velocidad_x],
			  [punto_inicio_y],
			  [velocidad_y]])
	print([[punto_inicio_x],
			  [velocidad_x],
			  [punto_inicio_y],
			  [velocidad_y]])

	# Matriz de covarianzas
	ukf.setP([[0.025,0,0,0],
			  [0,0.025,0,0],
			  [0,0,0.025,0],
			  [0,0,0,0.025]])

	 # Ruido para la predicción
	ukf.setQ([[0,0,0,0],
			  [0,0.1,0,0],
			  [0,0,0,0],
			  [0,0,0,0.1]])

	# Ruido para la actualización
	ukf.setR([[0.005**2,0],
			  [0,0.005**2]])

	ukf.setAlpha(0.001)
	ukf.setBeta(2)
	ukf.setKappa(0)

def funciones_fx_hx(t):
	fx = [[ukf.xs[ukf.i][0,0] + t * ukf.xs[ukf.i][1,0]],
		  [ukf.xs[ukf.i][1,0]],
		  [ukf.xs[ukf.i][2,0] + t * ukf.xs[ukf.i][3,0]],
		  [ukf.xs[ukf.i][3,0]]]
	hx = [[math.sqrt(ukf.xs[ukf.i][0,0]**2 + ukf.xs[ukf.i][2,0]**2)],
		  [math.atan2(ukf.xs[ukf.i][2,0],ukf.xs[ukf.i][0,0])]]
	return fx, hx


punto_inicio_x = 0
punto_inicio_y = 0
velocidad_x = 50
velocidad_y = 25
ruido_posicion = 0.0030
ruido_velocidad = 0.0025
_delta_t = 1


ukf = ukf()
preparar_algoritmo()

time = np.arange(0, 60, _delta_t)
# x = np.linspace(punto_inicio_x, punto_final_x,len(time))
# y = np.linspace(punto_inicio_y, punto_final_y,len(time))


"""
Posiciones x, y
"""
x = np.array([punto_inicio_x], dtype=np.float64)
y = np.array([punto_inicio_y], dtype=np.float64)
xv = np.array([velocidad_x])
yv = np.array([velocidad_y])
for i in range(len(time)-1):
	x = np.append(x, x[-1] + velocidad_x + np.random.normal(0, ruido_posicion))
	y = np.append(y, y[-1] + velocidad_y + np.random.normal(0, ruido_posicion))
	xv = np.append(xv, xv[-1] + np.random.normal(0, ruido_velocidad))
	yv = np.append(yv, yv[-1] + np.random.normal(0, ruido_velocidad))

# xv = np.gradient(x,time)
# yv = np.gradient(y,time)


r_raw = np.sqrt(x**2+y**2)
theta_raw = np.arctan2(y,x)
r_noise = np.random.normal(0,0.005,len(r_raw))
theta_noise = np.random.normal(0,0.005,len(theta_raw))
r = r_raw + r_noise
theta = theta_raw + theta_noise


prev = 0
x_f = []
y_f = []
xv_f = []
yv_f = []
x_p = []
y_p = []
xv_p = []
yv_p = []
for i in range(len(time)):
	t = float(time[i]) - prev
	prev = float(time[i])
	z = [[r[i]], # Observaciones del sensor
		 [theta[i]]]
	ukf.step(z, funciones_fx_hx, t) # Proceso de puntos sigmas, predicción y actualización
	x_filtered = ukf.getX() # Filtrada del estadp
	p_filtered = ukf.getP() # Covarianzas
	g_filtered = ukf.getG() # Ganancia de Kalman
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


plt.plot(time,x,'b',label='Real')
plt.plot(time,x_f,'r',label='Filtrada')
plt.plot(time,x_p,'y-',label='Predicha')
plt.title('Posiciones en X')
plt.figure()
plt.plot(time,xv,'b',label='Real')
plt.plot(time,xv_f,'r',label='Filtrada')
plt.plot(time,xv_p,'y-',label='Predicha')
plt.title('Velocidades en X')
plt.figure()
plt.plot(time,y,'b',label='Real')
plt.plot(time,y_f,'r',label='Filtrada')
plt.plot(time,y_p,'y-',label='Predicha')
plt.title('Posiciones en Y')
plt.figure()
plt.plot(time,yv,'b',label='Real')
plt.plot(time,yv_f,'r',label='Filtrada')
plt.plot(time,yv_p,'y-',label='Predicha')
plt.title('Velocidades en Y')
plt.legend()
plt.figure()
plt.plot(x,y,'b',label='Real')
plt.plot(x_f,y_f,'r',label='Filtrada')
plt.plot(x_p,y_p,'y--',label='Predicha')
plt.title('Trayectoria del avión')
plt.legend()
plt.show()