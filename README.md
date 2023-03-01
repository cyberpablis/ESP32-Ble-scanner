
# ESP32 GATT client modificado para enviar y recibir datos a traves de BLE

Version SDK: 5.0.1

remote_device_name[] deberá contener el nombre del dispositivo al que se conectará.

El equipo remoto debera tener las caracteristicas de TX y RX habilitadas.
- TX UUID "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
- RX UUID "6e400003-b5a3-f393-e0a9-e50e24dcca9e"

Al compilar, el fw intentará conectarse al dispositivo remoto y al hacerlo, enviará un string cada 2 segundos.
Todo lo que el dispositivo remoto envie, se mostrara por consola.

