# UsainBot

UsainBot es un robot velocista del tipo seguidor de líneas cuyo objetivo es servir de referencia y como plataforma para apoyar en la formación de los competidores. Está desarrollado en base a las tecnologías disponibles en un FabLab (impresión 3D, corte de PCBs, Arduino) y comprende las componentes más básicas para funcionar de acuerdo a las reglas de la competencia.

Un velocista seguidor de línea es un robot con ruedas que tiene la capacidad de distinguir franjas en una superficie de alto contraste (blanco y negro) y seguir su trayectoria de forma autónoma con la finalidad de completar un circuito lo más rápido posible.

# Diseño

## Antecedentes

Las funciones principales requeridas por el robot son:

- Reconocer franjas 
- Movimiento
- Energía 
- Procesamiento y control 

Para esto requiere una construcción óptima en cuanto al tamaño/peso y las prestaciones de sus componentes que cumplirán estas funcionalidades, las cuales pueden ser desempeñadas por diferentes tipos y arreglos de sensores, motores, baterías, etc. Otras funciones secundarias corresponden a la interacción con el usuario (encender/apagar, activar, indicar acciones), así como cualquier otra que se desee agregar (como comunicación inalámbrica para el debugging).

En el contexto de la competencia, existirán marcadores a los costados del circuito que indicarán hitos como curvas, cruces y fin. También se fija el tamaño máximo como 25x25x20 centímetros (largo, ancho y alto) y que las ruedas no pueden contener elementos adherentes que dañen la pista.

(imagen de los hitos)

## Diseño original

UsainBot se diseñó considerando este set mínimo de subsistemas, las tecnologías disponibles en el Fablab U de Chile, la disponibilidad de componentes comerciales en el país (Chile) y otros diseños populares de robots velocistas. Los dibujos del sistema se realizaron en AutodeskFusion 360 y se llegó al siguiente diseño:

(vistas del usainbot)

### Sensores

UsainBot cuenta con un arreglo de sensores infrarrojos modelo asdasdasdsad en el frente. Esta disposición permite detectar con cierto nivel de resolución la desviación del eje central del robot respecto a la franja del circuito:

(esquema)

También posee dos sensores simples del mismo tipo (modelo sadads) en cada lado para la detección de franjas. 

(imagen con zoom)

### Actuación

### Controlador

### Batería

### Chasis

### Otros

Con el objetivo de minimizar el peso y tamaño se diseña una PCB con doble propósito: ubicar los componentes en el mínimo espacio posible y como chasis del robot, considerando que la placa de sadklsdajlkasdjlk tiene rigidez suficiente.

(imagen de la PCB)

La PCB tiene conectores para intercambiar fácilmente los componentes en caso de que sufran algún desperfecto, así como orificios para el montaje de los motores.

# Lista de partes y materiales

## Partes comerciales

Item                     | Cantidad
 ---------------------------   | --------------
Motor DC con reducción 6V x rpm | 2
Driver asdasd | 1
Sensor QTR-8A | 1
Sensor QTR-1A | 2
Arduino Nano | 1
Lipo 7.4V | 1
Pulsador | 1
Switch | 1
LED | 1 
Buzzer | 1

Otros elementos necesarios para el desarrollo del robot pero no que son parte del mismo corresponden a: cargador de LiPo, alarma de carga de batería.

## Partes manufacturadas

Item                     | Cantidad
 ---------------------------   | ------------
Llantas | 2
Neumáticos | 2
Chasis PCB | 1

## Materiales

su weed
conectores

# Manufactura

## Herramientas necesarias

## Componentes

### Llantas

### Neumáticos

### PCB

## Ensamble

# Software

## Pista de pruebas

Para hacer pruebas de funcionamiento resulta muy útil tener una pequeña pista de pruebas, esta la fabricamos con un trozo de melamina delgado de color negro y cinta aislante de x mm de ancho de color blanco.

(imagen de la pista)

## Pruebas de subsistemas

Una vez que el sistema está ensamblado, los componentes deben ser probados separadamente antes de hacer la integración final. Para esto generamos códigos sencillos para ejecutar en el robot y observar tanto su funcionamiento como la información que entrega el Arduino a través del monitor serial.

- Prueba de motores.
- Prueba de sensores principal.
- Prueba de sensores laterales.

(links a los codigos)

## Integración



## Calibración del PID

El funcionamiento óptimo del robot implica maximizar la velocidad manteniendo el robot moviéndose dentro de la franja y colineal con esta. Para esto es necesario encontrar una configuración óptima de los parámetros del PID. 

### Calibración "manual"

De acuerdo al comportamiento observado en las pruebas, se pueden hacer las siguientes modificaciones:



# Posibles desarrollos

- Utilizar encoders en los motores, para tener un mayor control de la cinemática.





