# Diseño de sistemas inteligentes - 10º A

##### Descripción del problema

> Una máquina **lanza una pelota** con un *ángulo* y una *velocidad inicial*
definidas por las variables de abajo.
```python
angulo_lanzamiento = 50  # grados
velocidad_incial = 5  # m/s
```

> Un sensor observa el movimiento de la pelota y grafica su trayectoria con base en sus observaciones.

##### Ecuaciones del movimiento parabólico
Cada observación se realiza en un instante de tiempo, por ende, en cada instante se puede descomponer la velocidad en sus dos componentes, **X**,**Y**.

<p align="center">
  <img width="410" height="200" draggable="false" src="https://i.ibb.co/GMdC0xR/Captura-de-Pantalla-2019-12-07-a-la-s-16-20-58.png">
</p>

<!-- ![Ver imagen de trigonometría](https://i.ibb.co/GMdC0xR/Captura-de-Pantalla-2019-12-07-a-la-s-16-20-58.png) -->

Basándonos sobre la [imagen](https://ibb.co/pK14x0d), se obtienen las siguientes fórmulas:

<!-- h<sub>&theta;</sub>(x) = &theta;<sub>o</sub> x + &theta;<sub>1</sub>x -->

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

### TL;DR

Para obtener los componentes __x__,__y__ de cada velocidad __V<sub>i</sub>__ se aplican las siguientes fórmulas:

> x = ( V<sub>0</sub> cos &alpha; ) * t

> y = ( V<sub>0</sub> sin &alpha; ) * t + 1/2 * g * t^2 

