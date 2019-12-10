# Diseño de sistemas inteligentes - 10º A

Cuatrimestre | Septiembre - Diciembre de 2019
:------------: | :-------------:
Cuatrimestre y grupo | 10A
Asignatura | Diseño de Sistemas Inteligentes
Corte | 3
Actividad | DSI.C3.A1
Fecha de asignación | 2019.12.09
Fecha de entrega | 2019.12.10
Matrícula | Nombre
163189 | HERNÁNDEZ MORALES Luis Fernando
163202 | PÉREZ SÁNCHEZ David

<p align="center">
	<img src="https://i.ibb.co/PtV87CY/descripcion-problema.png" alt="Descripción del problema" width="400" height="250">
</p>

##### Descripción del problema

Una antena se sitúa en una coordenada y rastrea un avión que se encuentra en una posición en un plano 2D.

Dicha antena se encuentra realizando observaciones en un instante de tiempo __Δt__, en cada observación, la antena devuelve la distancia aproximada a la que se encuentra el avión y un ángulo &theta; de inclinación.

##### Dinámica

Para la dinámica fijamos un punto de inicio y uno final, simulando la trayectoria __lineal__ del avión.

Para poder llevar a cabo este proceso de manera lineal, decidimos usar la función [_linspace_](https://docs.scipy.org/doc/numpy/reference/generated/numpy.linspace.html) de _numpy_, la cual nos permite generar un conjunto de valores tomando un _inicio_, un _final_ y la cantidad de muestras.

```python
time = np.arange(0, 60, 0.1) # Arreglo de tiempo con delta t de 0.1
# La longitud de time es de: 600

x = np.linspace(-3000,3000,len(time)) # Vector que inicia desde -3000 hasta 3000 en saltos de 600 en 600
y = np.linspace(1000,800,len(time)) # Vector que inicia desde 1000 y desciende hasta 800 en saltos de 600 en 600
```

#### Observación

La antena computa la distancia a la que se encuentra el avión, así como el ángulo de inclinación de la misma.

Mediante la variable z, el cual contiene:
* r = raíz ( _x^2 + y^2_ )
* &theta; = arctan2( _y_, _x_ )

Cabe aclarar que el proceso de observación considera un ruido en la medición, quedando de la siguiente manera:

* r = raíz ( _x^2 + y^2_ ) + _ruido_
* &theta; = arctan2( _y_, _x_ ) + _ruido_

#### Transformación Unscented

La función **_F(x)_** se define de la siguiente manera:

```python
fx = [
		[ ukf.xs[ukf.i][0,0] + t * ukf.xs[ukf.i][1,0] ],
		[ ukf.xs[ukf.i][1,0] ],
		[ ukf.xs[ukf.i][2,0] + t * ukf.xs[ukf.i][3,0] ],
		[ ukf.xs[ukf.i][3,0] ]
	]
```

**_ƒ(x)[0,0]_** calcula la posición en _x_ del avión en función de su velocidad según el instante de tiempo en el que se encuentra, la velocidad en _x_ ( **_ƒ(x)[1,0]_** ) no se ve afectada por la **_ƒ(x)_**; sucede de forma similar para la posición en _y_ y la velocidad en _y_.

La función **_Η(x)_** se define de la siguiente manera:

```python
hx = [
		[ math.sqrt(ukf.xs[ukf.i][0,0]**2 + ukf.xs[ukf.i][2,0]**2) ],
		[ math.atan2(ukf.xs[ukf.i][2,0],ukf.xs[ukf.i][0,0]) ]
	]
```

**_H(x)_** actualiza el ángulo en el que la antena se inclina para poder rastrear al avión.

#### Resultados

![Medidas obtenidas de la antena](https://i.ibb.co/0yGW133/Posiciones-y-velocidades.png)
![Trayectoria del avión](https://i.ibb.co/NWdXJw2/Trayectoria.png)
