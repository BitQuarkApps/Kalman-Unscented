from scipy import linalg as scipy_alg
import numpy as np


class KalmanUnscented:
	"""
	Algoritmo de Kalman Unscented.
	Las ecuaciones fueron obtenidas del siguiente Documento:
	[ http://bibing.us.es/proyectos/abreproy/60353/fichero/Diseño+y+simulación+en+Matlab+de+los+algoritmos+EKF+y+UKF+aplicados+a+la+trayectoria+de+una+aeronave+equipada+con+un+sist~1.pdf ]
	"""
	def __init__(self):
		self.Xs = None # 2L+1 puntos sigma para fx
		self.Zs = None # 2L+1 puntos sigma para hx
		self.alfa = 10e-3
		self.beta = 2
		self.kappa = 0
	
	# Inicializadores de variables
	def set_x(self, vector_x):
		"""
		Actualizar la matriz X.

		Parámetros:
		`vector_x`: Arreglo de tamaño n x 1
		"""
		self.x = np.matrix(vector_x)
	
	def set_fx(self, matriz):
		"""
		Actualizar la matriz de F(x)
		
		Parámetros:
		`matriz`: Arreglo de tamaño n x 1
		"""
		self.fx = np.matrix(matriz)
	
	def set_hx(self, matriz):
		"""
		Actualizar la matriz de H(x)
		
		Parámetros:
		`matriz`: Arreglo de tamaño m x 1
		"""
		self.hx = np.matrix(matriz)
	
	def set_p(self, matriz):
		"""
		Actualizar la matriz de covarianzas
		
		Parámetros:
		`matriz`: Arreglo de tamaño n x n
		"""
		self.P = np.matrix(matriz)
	
	def set_q(self, matriz):
		"""
		Establecer el arreglo Q (ruido)
		
		Parámetros:
		`matriz`: Arreglo de tamaño n x n
		"""
		self.Q = np.matrix(matriz)
	
	def set_r(self, matriz):
		"""
		Establecer el arreglo R (ruido)
		
		Parámetros:
		`matriz`: Arreglo de tamaño m x m
		"""
		self.R = np.matrix(matriz)
	
	def puntos_sigma(self):
		"""
		Realizar la transformada y obtener los puntos sigma
		"""
		puntos = []
		self.pesos_x = []  # Pesos para cada punto sigma del estado
		self.pesos_p = []  # Pesos para cada punto sigma de la covarianza
		L = self.x.shape[0]
		lam = (self.alpha**2)*(L+self.kappa) - L
		points.append(self.x)
		add_val = scipy_alg.sqrtm((L + lam) * self.P)
		for i in range(add_val.shape[1]):
			add_val_mat = np.zeros(self.P.shape[0])
			for j in range(add_val.shape[0]):
				add_val_mat[j] = add_val[j][i]
			add_val_mat = np.matrix(add_val_mat).T
			points.append(self.x + add_val_mat)
		for i in range(add_val.shape[1]):
			add_val_mat = np.zeros(self.P.shape[0])
			for j in range(add_val.shape[0]):
				add_val_mat[j] = add_val[j][i]
			add_val_mat = np.matrix(add_val_mat).T
			points.append(self.x - add_val_mat)
		self.pesos_x.append(lam/(L+lam))
		self.pesos_p.append((lam/(L+lam)) + (1-self.alpha**2+self.beta))
		for i in range(2*L):
			self.pesos_x.append(1/(2*(L+lam)))
			self.pesos_p.append(1/(2*(L+lam)))
		self.xs = points
	
	def actualizar_sigmas(self, funcion, *args):
		self.puntos_sigma()
		self.Xs = [] # Xk|k-1
		self.Zs = [] # Yk
		for self.i in range(len(self.xs)):
			Xs_tem, Zs_tem = funcion(*args) # Pasamos el estado y la observación en la función no lineal
			self.Xs.append(np.matrix(Xs_tem))
			self.Zs.append(np.matrix(Zs_tem))

	def prediccion(self):
		"""
		Etapa de predicción
		"""
		self.xp = 0
		self.zp = 0  # Zk
		for i in range(len(self.pesos_x)):
			self.xp += self.pesos_x[i] * self.Xs[i] # K^k|k-1 = SUMATORIA ( Ws[i] * Xk|k[i] )
			self.zp += self.pesos_x[i] * self.Zs[i]
		self.Pp = 0
		for i in range(len(self.pesos_p)):
			self.Pp += self.pesos_p[i] * (self.Xs[i]-self.xp)*(self.Xs[i]-self.xp).T
		self.Pp += self.Q

	def actualizacion(self):
		"""
		Etapa de actualización
		"""
		self.Pyy = 0
		self.Pxy = 0
		for i in range(len(self.weights_P)):
			self.Pyy += self.weights_P[i] * (self.Zs[i]-self.zp)*(self.Zs[i]-self.zp).T
			self.Pxy += self.weights_P[i] * (self.Xs[i]-self.xp)*(self.Zs[i]-self.zp).T
		self.Pyy += self.R
		self.G = self.Pxy*np.linalg.pinv(self.Pyy)  # Ganancia
		self.x = self.xp + self.G*(self.z - self.zp)
		self.P = self.Pp - self.G*self.Pyy*self.G.T

	def iteracion(self, z, funcion_actualizacion, *args):
		"""
		Realizar una iteración => {
			1.- Cálculo de los puntos sigma.
			2.- Predicción.
			3.- Actualización.
		}
		"""
		self.actualizar_sigmas(funcion_actualizacion, *args)
		self.z = np.matrix(z)
		self.prediccion()
		self.actualizacion()
		return 0

	def get_estado(self):
		"""
		Obtener la filtrada del estado
		"""
		return self.x

	def get_matriz_covarianza(self):
		"""
		Obtener la matriz de covarianza Px
		"""
		return self.P

	def get_Ganancia(self):
		"""
		Obtener la ganacia de Kalman
		"""
		return self.G