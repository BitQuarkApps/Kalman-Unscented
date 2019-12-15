import math
import copy
import numpy as np
from scipy import linalg as scipy_alg

class Unscented:
	def __init__(self, estado, covarianzas, Q, R, h, k):
		self.Xs = None      # 2L+1 sigma points for fx
		self.Zs = None      # 2L+1 sigma points for hx
		self.x = np.matrix(estado)
		self.P = np.matrix(covarianzas)
		self.Q = np.matrix(Q)
		self.R = np.matrix(R)
		self.predicha = None
		self.filtrada = None
		self.h = h
		self.k = k

	def puntos_sigmas(self):
		puntos = []
		self.pesos_x = []
		self.pesos_P = []
		L = self.x.shape[0]
		puntos.append(self.x)
		raiz = scipy_alg.sqrtm(self.P)
		for i in range(raiz.shape[1]):
			suma_matriz = np.zeros(self.P.shape[0])
			for j in range(raiz.shape[0]):
				suma_matriz[j] = raiz[j][i]
			suma_matriz = np.matrix(suma_matriz).T
			puntos.append(self.x + suma_matriz)

		for i in range(raiz.shape[1]):
			suma_matriz = np.zeros(self.P.shape[0])
			for j in range(raiz.shape[0]):
				suma_matriz[j] = raiz[j][i]
			suma_matriz = np.matrix(suma_matriz).T
			puntos.append(self.x - suma_matriz)

		# Pesos
		for i in range(2*L+1):
			self.pesos_x.append(1/(2*L+1))
			self.pesos_P.append(1/(2*L+1))
		self.xs = puntos

	def actualizar_sigmas(self, t):
		self.puntos_sigmas()
		self.Xs = []
		self.Zs = []
		for self.i in range(len(self.xs)):
			xs_fx = self.fx(t)
			zs_hx = self.hx()
			self.Xs.append(np.matrix( xs_fx ))
			self.Zs.append(np.matrix( zs_hx ))
		print(f'hx => ( {self.Zs[0].shape[0]}, {len(self.Zs)})')

	def prediccion(self):
		self.xp = 0
		self.zp = 0
		for i in range(len(self.pesos_x)): # 2L
			self.xp += self.pesos_x[i] * self.Xs[i]
			self.zp += self.pesos_x[i] * self.Zs[i] # Zk
		self.predicha = copy.deepcopy(self.xp)
		self.Pp = 0 # P k | k-1
		for i in range(len(self.pesos_P)):
			self.Pp += self.pesos_P[i] * (self.Xs[i]-self.xp)*(self.Xs[i]-self.xp).T
		self.Pp += self.Q

	def actualizacion(self):
		self.Pyy = 0
		self.Pxy = 0
		for i in range(len(self.pesos_P)):
			self.Pyy += self.pesos_P[i] * (self.Zs[i]-self.zp) * (self.Zs[i]-self.zp).T
			self.Pxy += self.pesos_P[i] * (self.Xs[i]-self.xp) * (self.Zs[i]-self.zp).T
		self.Pyy += self.R
		self.G = self.Pxy*np.linalg.pinv(self.Pyy)
		self.x = self.xp + self.G*(self.z - self.zp)
		self.P = self.Pp - self.G*self.Pyy*self.G.T
		self.filtrada = copy.deepcopy(self.x)

	def iteracion(self, observacion, t):
		self.actualizar_sigmas(t)
		self.z = np.matrix(observacion)
		self.prediccion()
		self.actualizacion()
		return self.predicha, self.filtrada

	def fx(self, t):
		return [
			[ self.xs[self.i][0,0] + t * self.xs[self.i][1,0] ],
			[ self.xs[self.i][1,0] ],
			[ self.xs[self.i][2,0] + t * self.xs[self.i][3,0] ],
			[ self.xs[self.i][3,0] ]
		]

	# def hx(self):
	# 	return [
	# 		[ math.sqrt(self.xs[self.i][0,0]**2 + self.xs[self.i][2,0]**2) ],
	# 		[ math.atan2(self.xs[self.i][2,0],self.xs[self.i][0,0]) ]
	# 	]
	def hx(self):
		return [
			[ math.sqrt( ( self.xs[self.i][0,0]-self.h)**2 + (self.xs[self.i][2,0]-self.k )**2 ) ],
			[ math.atan2( self.xs[self.i][2,0]-self.k,self.xs[self.i][0,0]-self.h ) ]
		]