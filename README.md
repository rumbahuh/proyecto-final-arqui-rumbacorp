# PFA_RESTAURANTE

# Overview

## Computation Graph
[Aquí deberíamos añadir el Computation Graph después de la descripción]

## BT
![Behavior Tree - pfa_restaurante](https://github.com/user-attachments/assets/de32e57e-c2a3-4e81-a0c0-c7a06d4b0fbc)

# Resumen
La idea principal es la implementación de un acomodador de un restaurante.

El kobuki irá desde su posición inicial al punto de entrada, donde se encontrará una cantidad
X de personas. Mediante hri, el kobuki preguntará cuántos son y mediante un input se especificará el número de personas a acomodar.
Dependiendo de la cantidad de personas (menos de 4 o entre 4 y 6) les llevará a la mesa adecuada, volviendo al punto donde se encuentran los clientes una vez acabada su acción.
Si la cantidad de personas es superior a 6 o no hay mesas libres, el kobuki dirá que no hay
mesas disponibles.

# Detalles Técnicos

## Documentación
Behavior Trees

Nav2

![HRI](https://github.com/rodperex/bt_nodes)


## Poses
Inicial

Recibidor

mesa BIG: 6

mesa SMALL: 4

## Lógica
Si la mesa de 4 personas está libre y llega un grupo de 6, el kobuki ha de 
detectar que el grupo es demasiado grande y que no hay sitio.

## Uso
**Disclaimer: Implementamos una copia de HRI en el repositorio, por lo que el paquete usado y modificado es el nuestro. Dará error con los duplicados.**

**Kobuki**
```
ros2 launch kobuki kobuki.launch.py lidar_a2:=true
```

**Navegation**.
```
ros2 launch kobuki navigation.launch.py
```
Gracias a este comando también se abre el rviz2 con el que se puede visualizar el mapa por el que se mueve el kobuki.
```
ros2 run tf2_ros tf2_echo map base_link
```
El proceso que seguimos fue señalizar con el rivz2 las coordenadas que queríamos guardar en el blackboard, haciendo este ros2 run para poder visualizar las coordenadas exactas de los lugares a los que queremos mandar el robot.

**Nodo**
```
ros2 run pfa_restaurante control_main
```


# Apuntes
Pese a que principalmente se quería implementar el uso del yolo para el recuento
de personas y la detección de la cantidad de gente en las mesas, por falta de tiempo
y el uso de hri, no ha sido posible, por eso, se marca la mesa como ocupada 
independientemente de cuantos sitios libres queden en la mesa, con el fin de 
no mezclar grupos en la misma mesa y la simplificación del código e implementación.


# IDEAS A IMPLEMENTAR
Si llegan clientes nuevos y no hay una mesa disponible, va a la mesa más óptima
y les pide que se vayan, la marca vacía (asumimos que los clientes han hecho caso)
y vuelve a donde se encuentran los nuevos clientes para llevarles a la nueva 
mesa vacía.


## Video de implementación básica
[![watch the video](https://github.com/rumbahuh/proyecto-final-arqui-rumbacorp/edit/main/p3.jpg)](https://www.youtube.com/watch?v=4J1Ffc5u-C8)


**FECHA:** 9/5/2024 

**AUTORES:**
  MarioC05 (Mario Casero)
  rumbahuh (Rebeca Castilla)
  inarem13/inarem-jpg (Henar Contreras)
  LoVeMi3 (Lorea Vera)
