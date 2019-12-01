#Filtro de Kalman Unscented
# Diseño de Sistemas Inteligentes
# Corte 3
# Universidad Politécnica de Chiapas
# Creado por Luis Fernando Hernández Morales y David Pérez Sánchez
from utils.Unscented import Unscented
import numpy as np

if __name__ == '__main__':
	X0 = [[ 3, 5 ]]
	Px = np.array([
		[ 0.2, 0 ],
		[ 0, 0.1 ]
	])
	kf = Unscented(len(X0[0]))
	sigmas = kf.puntos_sigma(X0, Px)
	print(sigmas)
	# media = kf.obtener_puntos_sigma(X0, Px)
	# print(media)