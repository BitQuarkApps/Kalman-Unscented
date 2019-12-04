#Filtro de Kalman Unscented
# Diseño de Sistemas Inteligentes
# Corte 3
# Universidad Politécnica de Chiapas
# Creado por Luis Fernando Hernández Morales y David Pérez Sánchez
from utils.Unscented import Unscented
import matplotlib.pyplot as plt
import numpy as np

if __name__ == '__main__':
	X0 = np.array( [ 3, 5 ] )
	Px = np.array( [
		[ 0.2, 0 ],
		[ 0, 0.1 ]
	])
	kf = Unscented(len(X0))

	REAL_X = []
	REAL_Y = []
	PREDICHA_X = []
	PREDICHA_Y = []
	FILTRADA_X = []
	FILTRADA_Y = []

	REAL_X.append(X0[0])
	REAL_Y.append(X0[1])

	for i in range(3):  # 10 iteraciones
		#  Dinámica => Xt = i + Xt^2
		X0 = i + np.power(X0, 2)
		kf.puntos_sigma(X0, Px)
		filtrada, predicha = kf.prediccion()
		kf.actualizacion()

		REAL_X.append(X0[0])
		REAL_Y.append(X0[1])
		PREDICHA_X.append(predicha[0])
		PREDICHA_Y.append(predicha[1])
		FILTRADA_X.append(filtrada[0])
		FILTRADA_Y.append(filtrada[1])
	# plt.plot(REAL_X, REAL_Y, "-", label="Real")
	plt.plot(PREDICHA_X, PREDICHA_Y, ":", label="Estimada", )
	plt.plot(FILTRADA_X, FILTRADA_Y, "r--", label="Filtrada")
	plt.title('Filtro de Kalman Unscented')
	plt.legend()
	plt.show()