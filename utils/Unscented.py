from scipy import linalg as scipy_alg
import numpy as np


class KalmanUnscented:
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
		self.weights_x.append(lam/(L+lam))
		self.weights_P.append((lam/(L+lam)) + (1-self.alpha**2+self.beta))
		for i in range(2*L):
			self.weights_x.append(1/(2*(L+lam)))
			self.weights_P.append(1/(2*(L+lam)))
		self.xs = points
	
	def actualizar_sigmas(self, funcion):
		self.puntos_sigma()
		self.Xs = []
		self.Zs = []
