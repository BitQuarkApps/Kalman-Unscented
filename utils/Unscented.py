from numpy.linalg import inv  # Inversa
import numpy as np
import math
import copy

class Unscented:
	def __init__(self, variables_estado, alfa=10e-3, k=0, beta=2):
		"""
		Clase que contiene los métodos que la extensión Unscented del
		filtro de Kalman requiere.

		Parámetros:
		variables_estado (int): Número de variables que definen el estado del sistema,
		es decir, la longitud del vector X.
		alfa (float): Dispersión de los puntos sigma, [default] 10e-3
		k (float): (Kappa) es el segundo parámetro de escala, [default] 0
		beta (fliat): Parámetro para incorporar el conocimiento previo
		de la distribución de X. [default] 2
		"""
		print("=== Filtro de Kalman Unscented ===")
		self.L = variables_estado
		self.alfa = alfa
		self.k = k
		self._lambda = math.pow(alfa, 2) * (self.L + self.k) - self.L
		self.beta = beta
		
		self.Xk_k_menos1 = None # Xk|k-1 y Xk-1|k-1 para el proceso anterior
		self.Xk_k_menos1_predicha = None  # X^ k|k-1
		self.Pk_k_menos1 = None  # Pk|k-1
		self.Ws = None
		self.Wc = None

	def fx(self, Xi):
		"""
		Aquí se define la función de transición de estados NO LINEAL
		para cada punto sigma
		"""
		return Xi + math.sin(math.pow(Xi, 2))

	def hx(self, Xi):
		"""
		Función NO LINEAL de observación
		"""
		return math.sin(Xi) + Xi
	
	def puntos_sigma(self, Xk, Px_k):
		"""
		Calcular los puntos sigma dada una matriz X y una matriz de covarianza.
		k => Instante de tiempo
		Parámetros:
		Xk (list): Estado en el tiempo t
		Px_k (list): Vector de covarianzas

		Actualiza:
		Variable `self.Xk_k_menos1`
		"""
		if self.Xk_k_menos1 is None:
			matriz_sigmas = copy.deepcopy(Xk)
		else
			# matriz_sigmas = copy.deepcopy(self.Xk_k_menos1) # X k-1|k-1
			matriz_sigmas = [] # X k-1|k-1
		for index, x in enumerate(Xk[0]):
			_sigma = Px_k[index][index]
			punto_sigma = x + math.sqrt(_sigma)
			if(index%2 == 0):
				# X
				try:
					matriz_sigmas.append([punto_sigma, Xk[0][index+1]])
				except Exception as e:
					print(e)
					pass
			else:
				# Y
				try:
					matriz_sigmas.append([Xk[0][index-1], punto_sigma])
				except Exception as e:
					print(e)
					pass

		for index, x in enumerate(Xk[0]):
			_sigma = Px_k[index][index]
			punto_sigma = x - math.sqrt(_sigma)
			if(index%2 == 0):
				# X
				try:
					matriz_sigmas.append([punto_sigma, Xk[0][index+1]])
				except Exception as e:
					print(e)
					pass
			else:
				# Y
				try:
					matriz_sigmas.append([Xk[0][index-1], punto_sigma])
				except Exception as e:
					print(e)
					pass
		self.Xk_k_menos1 = matriz_sigmas

	def asignar_pesos(self):
		pesos_estimacion_estado = []  # W[s]
		pesos_matriz_covarianza = []  # W[c]
		for i in range(2*self.L):
			ws = self._lambda/(self.L + self._lambda)
			wc = (self._lambda/(self.L + self._lambda)) + (1-math.pow(self.alfa,2)-self.beta)
			pesos_estimacion_estado.append(ws)
			pesos_matriz_covarianza.append(wc)
		return pesos_estimacion_estado, pesos_matriz_covarianza
	
	def prediccion(self):
		"""
		Etapa de predicción
		"""
		# Pasar cada punto sigma a través de la f(x)
		xk_k_menos1 = []
		for i, xk_menos1 in enumerate(self.Xk_k_menos1):
			# self.Xk_k_menos1[i] = self.fx(xk_menos1)
			xk_k_menos1.append(self.fx(xk_menos1))

		x_k_k_menos1_predicha = []

		ws, wc = self.asignar_pesos()
		self.Ws = ws
		self.Wc = wc
		for i in range(2*self.L):
			x_k_k_menos1_predicha.append(
				ws[i]*xk_k_menos1[i]
			)

		pk_k_menos1 = []
		Qk = np.array(
			[
				[ np.random.normal(0, 0.02**2) ],
				[ np.random.normal(0, 0.02**2) ]
			]
		)
		for i in range(2*self.L):
			parte1 = wc[i]*(xk_k_menos1[i]-x_k_k_menos1_predicha)
			parte2 = parte1 * np.transpose(
				xk_k_menos1[i]-x_k_k_menos1_predicha
			) + Qk
			pk_k_menos1.append(parte2)

		self.Xk_k_menos1_predicha = copy.deepcopy(xk_k_menos1)
		self.Pk_k_menos1 = copy.deepcopy(pk_k_menos1)
	
	def actualizacion(self):
		Yk = []
		for i in range(2*self.L):
			Yk.append(
				self.hx(
					self.Xk_k_menos1[i]
				)
			)
		Zk = 0
		for i in range(2*self.L):
			Zk +=self.Ws[i]*Yk[i]

		Pzk_zk = []
		Rk = np.array(
			[
				[ np.random.normal(0, 0.02**2) ],
				[ np.random.normal(0, 0.02**2) ]
			]
		)
		for i in range(2*self.L):
			parte1 = self.Wc[i] * (Yk[i]-Zk)
			parte2 = parte1 @ np.transpose(
				Yk[i]-Zk
			) + Rk
			Pzk_zk.append(parte2)

		Pxk_zk = []
		for i in range(2*self.L):
			parte1 = self.Wc[i] * ( self.Xk_k_menos1[i] - self.Xk_k_menos1_predicha )
			parte2 = parte1 @ np.transpose(
				Yk[i] - Zk
			)
			Pxk_zk.append(parte2)

			Kk = Pxk_zk @ inv(Pzk_zk)

			Pk_k = self.Pk_k_menos1 - ( np.dot( Kk, np.dot( Pzk_zk, np.transpose(Kk) )) )
			self.Pk_k_menos1 = copy.deepcopy(Pk_k)

	def obtener_puntos_sigma(self, X0, Px):
		"""
		Calcular los puntos sigma dada una matriz X y una matriz de covarianza.

		Parámetros:
		X0 (list): Estado en el tiempo t
		Px (list): Vector de covarianzas

		Regresa:
		Media de los puntos sigma
		"""
		matriz_sigmas = copy.deepcopy(X0)
		for index, x in enumerate(X0[0]):
			_sigma = Px[index][index]
			punto_sigma = x + math.sqrt(_sigma)
			if(index%2 == 0):
				# X
				try:
					matriz_sigmas.append([punto_sigma, X0[0][index+1]])
				except Exception as e:
					print(e)
					pass
			else:
				# Y
				try:
					matriz_sigmas.append([X0[0][index-1], punto_sigma])
				except Exception as e:
					print(e)
					pass

		for index, x in enumerate(X0[0]):
			_sigma = Px[index][index]
			punto_sigma = x - math.sqrt(_sigma)
			if(index%2 == 0):
				# X
				try:
					matriz_sigmas.append([punto_sigma, X0[0][index+1]])
				except Exception as e:
					print(e)
					pass
			else:
				# Y
				try:
					matriz_sigmas.append([X0[0][index-1], punto_sigma])
				except Exception as e:
					print(e)
					pass
		
		"""
		Una vez que se obtienen los puntos sigma, se procede
		a calcular S(x), donde S(x) =
		[ atan(x/y) ]  -------------> Ángulo
		[ raiz(X^2 + Y^2) ] --------> Distancia
		"""
		angulos_s = []
		distancias_s = []

		for index, punto_sigma in enumerate(matriz_sigmas):
			_x = punto_sigma[0]
			_y = punto_sigma[1]
			angulo = math.degrees(math.atan(_x/_y))
			angulos_s.append(angulo)
			distancia = math.sqrt((math.pow(_x, 2) + math.pow(_y, 2)))
			distancias_s.append(distancia)
		
		"""
		Finalmente se calcula la media del vector S(x)
		"""
		_media_angulos = 0
		_media_distancias = 0

		for angulo in angulos_s:
			_media_angulos += angulo
		_media_angulos = _media_angulos/len(angulos_s)

		for distancia in distancias_s:
			_media_distancias += distancia
		_media_distancias = _media_distancias/len(distancias_s)
		return [_media_angulos, _media_distancias]
		