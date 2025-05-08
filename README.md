# PFA_RESTAURANTE

## Overview

### Computation Graph
[Aquí deberíamos añadir el Computation Graph después de la descripción]

### BT
![Behavior Tree - pfa_restaurante](https://github.com/user-attachments/assets/de32e57e-c2a3-4e81-a0c0-c7a06d4b0fbc)

## Resumen
La idea principal es la implementación de un acomodador de un restaurante.

El kobuki irá desde su posición inicial al punto de entrada, donde se encontrará una cantidad
X de personas. Mediante hri, el kobuki preguntará cuántos son y mediante un input se especificará el número de personas a acomodar.
Dependiendo de la cantidad de personas (menos de 4 o entre 4 y 6) les llevará a la mesa adecuada, volviendo al punto donde se encuentran los clientes una vez acabada su acción.
Si la cantidad de personas es superior a 6 o no hay mesas libres, el kobuki dirá que no hay
mesas disponibles.

## Detalles Técnicos

### Documentación
Behavior Trees
Nav2
![HRI](https://github.com/rodperex/bt_nodes)

### Poses
Inicial
Recibidor
Mesa1: 4
Mesa2: 4
Mesa3: 6
Mesa4: 4

### Lógica
Si la mesa de 4 personas está libre y llega un grupo de 6, el kobuki ha de 
detectar que el grupo es demasiado grande y que no hay sitio.

### Uso
**Kobuki**
´´´
[AÑADIR COMANDOS]
´´´
**Navegation**
´´´
ros2 launch kobuki navigation.launch.py
´´´
**Nodo**
´´´
ros2 run pfa_restaurante control_main
´´´
## APUNTES
Pese a que principalmente se quería implementar el uso del yolo para el recuento
de personas y la detección de la cantidad de gente en las mesas, por falta de tiempo
y el uso de hri, no ha sido posible, por eso, se marca la mesa como ocupada 
independientemente de cuantos sitios libres queden en la mesa, con el fin de 
no mezclar grupos en la misma mesa y la simplificación del código e implementación.

## IDEAS A IMPLEMENTAR
Si llegan clientes nuevos y no hay una mesa disponible, va a la mesa más óptima
y les pide que se vayan, la marca vacía (asumimos que los clientes han hecho caso)
y vuelve a donde se encuentran los nuevos clientes para llevarles a la nueva 
mesa vacía.

**FECHA:** 9/5/2024 
**AUTORES:**
  MarioC05 (Mario Casero)
  rumbahuh (Rebeca Castilla)
  inarem13 (Henar Contreras)
  LoVeMi3 (Lorea Vera)
