# Feria Very Small Size Soccer
Este repositorio contiene un código para controlar robots de la categoría Very Small Size Soccer (VSSS) usando dos joysticks.

## Primeros pasos
Si nunca habías usado este programa antes, sigue los siguientes pasos:
### Dependencias
PIP

    sudo apt install python3-pip

pyserial

    pip install pyserial

### Permisos

    sudo usermod -a -G dialout your_user_name

Reiniciar

    sudo reboot now

## Comunicación

El nodo de communicación manda comandos a los robots

    rosrun feria_vsss communication.py

## Joysticks

El comando para conectar el joystick Xbox es:

    rosrun joy joy_node /joy _dev:="/dev/input/js0"

El comando para conectar el joystick Machenike es:

    rosrun joy joy_node __name:=joy_node2 /joy:=/joy2 _dev:="/dev/input/js1"

`__name:=joy_node2` es para que no se repita el nombre del nodo y `/joy:=/joy2` es para que no se repita el topic.

## Joysticks a comunicación

El comando para conectar el joystick Xbox con la comunicación es:

    rosrun feria_vsss joycom.py

El comando para conectar el joystick Machenike con la comunicación es:

    rosrun feria_vsss joycom2.py

