# Algoritmo de Kalman Unscented

<center>

Cuatrimestre | Septiembre - Diciembre de 2019
:------------: | :-------------:
Cuatrimestre y grupo | 10° "A"
Asignatura | Diseño de Sistemas Inteligentes
Corte | 3
Actividad | DSI.C3.A1
Fecha de asignación | 2019.12.09
Fecha de entrega | 2019.12.10
Matrícula | Nombre
163189 | HERNÁNDEZ MORALES Luis Fernando
163202 | PÉREZ SÁNCHEZ David

</center>

---

## Problema

Obtener la distancia y ángulo de un objeto en movimiento en el cielo a través de un radar que rastrea su posición.

### Descripción del problema

Un radar se sitúa en una posición definida y rastrea un avión que se encuentra en movimiento en el cielo sobre un plano de dos dimensiones (**2D**).

<p align="center">
	<img src="https://firebasestorage.googleapis.com/v0/b/mechanicapp-project.appspot.com/o/filtro_kalman_unscented%2Fproblema.gif?alt=media&token=fa84c0c5-6e05-449f-a53e-3c3ec22479a0" alt="Descripción del problema" title="Descripción del problema">
</p>

Dicho radar se encuentra realizando observaciones del movimiento del objeto en cada instante de tiempo **Δt** en la cual este devuelve la distancia aproximada (**r**) a la que se encuentra el avión y un ángulo (**&theta;**) de inclinación respecto al observador (radar). El problema también considera la necesidad de calcular no sólo el desplazamiento en el eje *x* e *y*, sino también sus velocidades en los ejes respectivos.

Estos cuatro estados deben estimarse teniendo en cuenta sólo las mediciones de distancia y ángulo antes dichas, más un ruido agregado. Dado que los desplazamientos y las velocidades están relacionados no linealmente con la distancia y el ángulo, este problema será tratado utilizando el algoritmo del Filtro de Kalman Unscented.

## Dinámica

El algoritomo hace los cálculos pertinentes mediante matrices, una de ellas es la *estado*, como la que se muestra a continuación, la cuál llegará a ser nuestra matriz llamada **Fx**. Durante un período de tiempo corto, se puede considerar que el desplazamiento cambia de acuerdo con la aproximación de primer orden,

<p align="center">
	<img src="https://firebasestorage.googleapis.com/v0/b/mechanicapp-project.appspot.com/o/filtro_kalman_unscented%2Festados.gif?alt=media&token=271873ed-2921-471b-a05c-7e8689b9b521" alt="Matriz de estado Fx" title="Matriz de estado Fx">
</p>

La ecuación anterior nos dice que durante un período de tiempo corto la posición cambia en **Δt** veces la velocidad (en ambas direcciones x e y) y que la velocidad permanece constante (en ambas direcciones x e y).

La ecuación de actualización de la medición es un poco más compleja: depende de la diferenciación de una identidad trigonométrica. La distancia y el angulo están relacionados con los desplazamientos "x" e "y" de las ecuaciones,

<p align="center">
	<img src="https://firebasestorage.googleapis.com/v0/b/mechanicapp-project.appspot.com/o/filtro_kalman_unscented%2Fdistancia%20y%20angulo.gif?alt=media&token=449d1ef9-a6c1-4448-bfdf-687bd28060b3" alt="Distancia y ángulo" title="Distancia y ángulo">
</p>

Por lo tanto, la matriz para el cálculo de las equaciones está dada por,

<p align="center">
	<img src="https://firebasestorage.googleapis.com/v0/b/mechanicapp-project.appspot.com/o/filtro_kalman_unscented%2Fmatriz%20h.gif?alt=media&token=7b0ac11e-9b42-4b79-b421-3dfda6894433" alt="Matriz de cálculo Hx" title="Matriz de cálculo Hx">
</p>

Para la dinámica fijamos un punto de inicio y uno final, simulando la trayectoria __lineal__ del avión.

Para poder llevar a cabo este proceso de manera lineal, decidimos usar la función [_linspace_](https://docs.scipy.org/doc/numpy/reference/generated/numpy.linspace.html) de _numpy_, la cual nos permite generar un conjunto de valores tomando un _inicio_, un _final_ y la cantidad de muestras.

```python
time = np.arange(0, 60, 0.1) # Arreglo de tiempo con delta t de 0.1
# La longitud de time es de: 600

x = np.linspace(-3000,3000,len(time)) # Vector que inicia desde -3000 hasta 3000 en saltos de 600 en 600
y = np.linspace(1000,800,len(time)) # Vector que inicia desde 1000 y desciende hasta 800 en saltos de 600 en 600
```

## Observación

El radar calcula la distancia a la que se encuentra el avión, así como el ángulo de inclinación del mismo. Estos son calculados mediante el desplzamiento del avión en los ejes "x" e "y" mediante la integración de las velocidades, y estos a su vez mediante la integración de las aceleraciones.

La variable "*z*" almacena la distancia y el ángulo calculados en un instante de tiempo *Δt*:
- r = raíz ( _x^2 + y^2_ )
- &theta; = arctan2( _y_, _x_ )

Cabe aclarar que el proceso de observación considera un ruido en la medición, quedando de la siguiente manera:

* r = raíz ( _x^2 + y^2_ ) + _ruido_
* &theta; = arctan2( _y_, _x_ ) + _ruido_

### Transformación Unscented

La función **_F(x)_** se define de la siguiente manera:

```python
fx = [
		[ ukf.xs[ukf.i][0,0] + t * ukf.xs[ukf.i][1,0] ],
		[ ukf.xs[ukf.i][1,0] ],
		[ ukf.xs[ukf.i][2,0] + t * ukf.xs[ukf.i][3,0] ],
		[ ukf.xs[ukf.i][3,0] ]
	]
```

**_ƒ(x)[0,0]_** calcula la posición en "_x_" del avión en función de su velocidad según el instante de tiempo en el que se encuentra. La velocidad en "_x_" ( **_ƒ(x)[1,0]_** ) no se ve afectada por la **_ƒ(x)_**.

Sucede de forma similar para la posición en "_y_" del objeto y la velocidad en "_x_" e "_y_" del mismo.

La función **_Η(x)_** se define de la siguiente manera:

```python
hx = [
		[ math.sqrt(ukf.xs[ukf.i][0,0]**2 + ukf.xs[ukf.i][2,0]**2) ],
		[ math.atan2(ukf.xs[ukf.i][2,0],ukf.xs[ukf.i][0,0]) ]
	]
```

**_H(x)_** actualiza el ángulo en el que el radar se inclina para poder rastrear el avión.

## Ejemplo

Para este ejemplo, se asume que el objeto comienza en el noroeste y viaja hacia el este a **100 m/s**. Esto corresponde a la configuración dada en la siguiente tabla,

Tipo | Valor
:------------: | :-------------:
Posición x inicial | -1000 m
Posición y inicial | 1000 m
Velocidad x inicial | 100 m/s
Velocidad y inicial | 0 m/s
Ruido de desplazamiento | Aleatorio

### Resultados

![Medidas obtenidas de la antena](https://i.ibb.co/0yGW133/Posiciones-y-velocidades.png)

![Trayectoria del avión](https://i.ibb.co/NWdXJw2/Trayectoria.png)
