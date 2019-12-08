# Diseño de sistemas inteligentes - 10º A

##### Descripción del problema

> Una máquina **lanza una pelota** con un *ángulo* y una *velocidad inicial*
definidas por las variables de abajo.
```python
angulo_lanzamiento = 50  # grados
velocidad_incial = 5  # m/s
gravedad = 9.81 # m/s
```

> Un sensor observa el movimiento de la pelota y grafica su trayectoria con base en sus observaciones.

<!-- ##### Ecuaciones del movimiento parabólico
Cada observación se realiza en un instante de tiempo, por ende, en cada instante se puede descomponer la velocidad en sus dos componentes, **X**,**Y**.

<p align="center">
  <img width="410" height="200" draggable="false" src="https://i.ibb.co/GMdC0xR/Captura-de-Pantalla-2019-12-07-a-la-s-16-20-58.png">
</p>



Basándonos sobre la [imagen](https://ibb.co/pK14x0d), se obtienen las siguientes fórmulas:



1. Obtener el eje X:
	* H * cos &alpha; = A
2. Obtener el eje Y:
	* H * sin &alpha; = B

Para cada instante, se obtienen las dos componentes de la siguiente manera:

> V<sub>0x</sub> = V<sub>0 cos &alpha;</sub>

Como la velocidad es constante, se recurre a la ecuación del **movimiento rectilíneo uniforme**.

> x = V<sub>0x</sub> * t

Sustituyendo __V<sub>0x</sub>__ en la ecuación tenemos como resultado:

> x = ( V<sub>0</sub> cos &alpha; ) * t

Ahora vamos a calcular el componente Y, para ello recurrimos a la ecuación de caída libre, la cual es:

> y = V<sub>0y</sub>t + 1/2 * g * t^2 

Para calcular el componente Y, sabemos que debemos aplicar la fórmula:

> V<sub>0y</sub> = V<sub>0</sub> sin &alpha;

Reemplazando __V<sub>0y</sub>__ en la ecuación de caída libre obtenemos:

> y = ( V<sub>0</sub> sin &alpha; ) * t + 1/2 * g * t^2 

Para calcular la velocidad final en el eje __Y__:

> V<sub>ƒy</sub> = V<sub>0y</sub> + gt

Reemplazando __V<sub>0y</sub>__ en la ecuación _5_:

> V<sub>ƒy</sub> = ( V<sub>0</sub> sin &alpha; ) + gt

Para calcular el tiempo de subida:

> t<sub>s</sub> = ( V<sub>0</sub> sin &alpha; ) / g

Para calcular la distancia máxima que recorrerá la pelota usamos la siguiente ecuación:

> X<sub>max</sub> = ( V<sub>0</sub>^2 sin 2&alpha;) / g
 -->
### Dinámica

La dinámica del movimiento de las pelotas se realizará con las siguientes ecuaciones:

> x = ( V<sub>0</sub> cos &alpha; ) * t

> y = ( V<sub>0</sub> sin &alpha; ) * t + 1/2 * g * t^2 

<!-- > V<sub>ƒy</sub> = ( V<sub>0</sub> sin &alpha; ) + gt -->

### Observación

El sensor usará las siguientes ecuaciones para obtener la velocidad de la pelota en movimiento.

> V<sub>x</sub> = v<sub>0</sub> cos &theta;<sub>0</sub>

> V<sub>y</sub> = v<sub>0</sub> sin &theta;<sub>0</sub> - g * t