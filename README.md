# pfa_restaurante

Poner fecha y autores (nuestros nombres y cual es el @ de cada uno,pq algunos
estamos usando la cuenta priv)

# Overview [borrador template]
[Aquí deberíamos añadir el Computation Graph después de la descripción]
#BT
subimos el Behaviour tree tmb?
# IDEAS A IMPLEMENTAR
si llegan clientes nuevos y no hay una mesa disponible, va a la mesa más óptima
y les pide que se vayan, la marca vacía (asumimos que los clientes han hecho caso)
y vuelve a donde se encuentran los nuevos clientes para llevarles a la nueva 
mesa vacía.

#RESUMEN
La idea principal es la implementación de un acomodador de un restaurante.

El kobuki irá desde su posición inicial a un punto donde se encontrará una cantidad
X de personas, mediante los hri, el kobuki preguntará cuántos son, y dependiendo
de la cantidad (menos de 4 o entre 4 y 6) les llevará a una mesa u otra, para
luego volver al punto donde se encuentran los clientes. Si la cantidad
de personas es superior a 6 o no hay mesas libres, el kobuki dirá que no hay
mesas disponibles.

#DETALLES TÉCNICOS
Si la mesa de 4 personas está libre y llega un grupo de 6, el kobuki ha de 
detectar que el grupo es demasiado grande y que no hay sitio 

Se ha implementado el uso de behaviour trees y hri PONER MÁS AQUÍ, 
explicar en profundidad

#APUNTES
Pese a que principalmente se quería implementar el suo del yolo para el recuento
de personas y la detección de la cantidad de gente en las mesas, por falta de tiempo
y el uso de hri, no ha sido posible, por eso, se marca la mesa como ocupada 
independientemente de cuantos sitios libres queden en la mesa, con el fin de 
no mezclar grupos en la misma mesa y la simplificación del código e implementación.


