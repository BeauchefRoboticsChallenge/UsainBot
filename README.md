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

UsainBot cuenta con un arreglo de sensores infrarrojos modelo asdasdasdsad en el frente. Esta disposición permite detectar con cierto nivel de resolución la desviación del eje central del robot respecto a la franja del circuito y tener un mejor control:

(esquema)

También posee dos sensores simples del mismo tipo (modelo sadads) en cada lado para la detección de franjas. 

(imagen con zoom)

### Actuación

El robot posee una configuración diferencial, dos ruedas controladas por motores DC idénticos modelo asdasdsad, de x rpm nominales y v Volts. 

La llanta de la rueda, impresa en 3D, posee una muesca donde se inserta el eje del motor (que tiene una muesca opuesta) a presión, mientras que el neumático es de silicona.

Los motores se apernan al chasis mediante soportes impresos en 3D.

Considerando la corriente que requieren estos motores elegimos utilizar como driver el puente H sadasdasdasd.

### Controlador

Decidimos utilizar un Arduino Nano por varias razones: Soporte de software, tamaño, lógica de 5V (compatible con el resto de componentes), disponibilidad, precio, facilidad de uso (cuenta con pines y puerto USB), entre otras.

### Batería

Considerando el voltaje necesario para el controlador y los motores, los cuales además suelen requerir altos niveles de corriente súbitamente es que elegimos utilizar una batería LiPo de 7.4 V modelo asdsdasad. Con tal capacidad permitirá realizar pruebas un par de horas antes de volver a cargarla sin conllevar mucho peso sobre el sistema.

### Chasis

Con el objetivo de minimizar el peso y tamaño se diseña una PCB con doble propósito: ubicar los componentes en el mínimo espacio posible y como chasis del robot, considerando que la placa de sadklsdajlkasdjlk tiene rigidez suficiente.

(imagen de la PCB)

La PCB tiene conectores para intercambiar fácilmente los componentes en caso de que sufran algún desperfecto, así como orificios para el montaje de los motores.

### Otros

Como tercer punto de apoyo se utiliza la parte cabeza de un LED redondo.

Utilizamos un buzzer standard de 5V para que el robot informe mediante alertas sonoras el estado en el que se encuentra.

Incluimos un switch para energizar los sistemas desde la batería y un pulsador para iniciar la rutina de carrera.

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

- pines, conectores,
- placa fr-4
- silicona epóxica 20-50 Shore A
- pernos m3x8 (4)
- tuerca m3 (4)

# Manufactura

## Componentes

### Llantas y neumáticos

Dado que tenemos acceso a una impresora de tipo SLA (Form2), decidimos fabricar las llantas con resina tenaz (Tough Resin) para soportar posibles golpes y el ajuste a presión sobre el eje, y moldes para los neumáticos con resina estándar translúcida (Clear Resin), pues de esta forma es fácil desprender la pieza moldeada del molde mismo y además permite observar la presencia de burbujas durante el procseo de curado. 

El neumático se fabrica directamente sobre la llanta, como se muestra en el diagrama:

Para el proceso es necesario mezclar la silicona en partes iguales, revolver y degasificar rapidamente antes de que la silicona empiece a solidificarse.

### Anclajes de motor

Estos anclajes afirman el motor utilizando pernos M3x8 y tuercas M3, son impresos en FDM con PLA.

### PCB

Para la manufactura de la PCB contamos con una máquina LPKF modelo sadasdsda. Las herramientas que utilizamos para el corte son:

- sdaasd
- asdsd
- sadsdasadsda

Antes de empezar a soldar los componentes es importante probar la integridad de las pistas midiendo la conductividad con un multímetro, para evitar corto circuitos y circuitos abiertos.

Luego soldamos los pines, buzzer, pulsador, switch y LED de acuerdo al esquema.

## Ensamble

1.- Montar las ruedas en cada motor
2.- Montar los motores con sus anclajes sobre la PCB.
3.- Conectar Arduino Nano
4.- Conectar sensores laterales
5.- Conectar arreglo de sensores frontal.
6.- Conectar driver.
7.- Montar batería con un elástico y conectar.

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





