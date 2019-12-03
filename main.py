#Filtro de Kalman Unscented
# Diseño de Sistemas Inteligentes
# Corte 3
# Universidad Politécnica de Chiapas
# Creado por Luis Fernando Hernández Morales y David Pérez Sánchez
from utils.Unscented import Unscented
import numpy as np

if __name__ == '__main__':
	X0 = np.array( [ 3, 5 ] )
	Px = np.array( [
		[ 0.2, 0 ],
		[ 0, 0.1 ]
	])
	kf = Unscented(len(X0))
	kf.puntos_sigma(X0, Px)
	for i in range(10):  # 10 iteraciones
		#  Dinámica => Xt = i + Xt^2
		X0 = i + np.power(X0, 2)
		kf.puntos_sigma(X0, Px)
		kf.prediccion()
		kf.actualizacion()