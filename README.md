## Driver do sensor BMP280

Criei esse driver para fazer meu sensor BMP280 funcionar com uma placa ESP8266 da NodeMCU.

Para fazer com que seu sensor funcione, primeiro voce tera que realizar o processo de flash
do firmware MicroPython na sua ESP8266.

Eu usei esse [link](https://micropython.org/resources/firmware/ESP8266_GENERIC-20250809-v1.26.0.bin) para realizar o download do firmware.

Caso esteja usando Windows, recomendo que utilize esse [guia](https://embarcados.com.br/micropython-no-esp8266/) para realizar o processo de 
flash corretamente.

Eu utilizo a versao debian do Linux, entao nao ha necessidade de baixar nada alem da 
biblioteca esptool atraves do `pip install esptool`.

> Note que devido a possibilidade de quebra de dependencias, a maioria dos sistemas Linux
> proibem o uso do pip diretamente. O certo a se fazer e criar um [ambiente virtual](https://docs.python.org/3/library/venv.html) separado 
> para depois rodar o comando.

Apos a instalacao correta do `esptool`, procure a pasta em que voce realizou o download do
binario do MicroPython (use `cd` se estiver no Linux ate achar a pasta com o arquivo)
e execute o comando `esptool --port /dev/ttyUSB0 --baud 460800 write-flash --flash-size=detect 0 ESP8266_GENERIC-20250809-v1.26.0.bin`.

> Caso a versao do firmware baixada seja diferente, nao se esqueca de atualizar o nome do
> arquivo.

Se tudo der certo, voce agora tera o MicroPython instalado na sua placa ESP8266.

Para ter acesso a placa, voce simplesmente pode realizar o download do [Thonny IDE](https://thonny.org/). No 
Debian/Ubuntu (e sistemas derivados), voce pode usar `sudo apt install thonny` e apos
isso ele ja estara corretamente instalado no seu PC.

Neste [link](https://embarcados.com.br/micropython-programando-com-a-ide-thonny/) voce pode encontrar mais informacoes de como utilizar corretamente a Thonny IDE 
para controlar sua ESP8266.

---

Para utilizar corretamente este driver, realize o download do `bmp280.py`, salve-o na sua
placa atraves do Thonny IDE e pronto, voce ja estara com acesso seu sensor BMP280.

Um script de exemplo que voce pode rodar para verificar o correto funcionamento e o seguinte:

```python
import time 
from machine import Pin, I2C
from bmp280 import BMP280

# Initialize I2C bus on ESP8266
i2c = I2C(scl=Pin(5), sda=Pin(4))

# Create a BMP280 sensor object
bmp = BMP280(i2c)

# Loop to read values
while True:
    temperature, pressure = bmp.get_values()
    print(f"Temperature: {temperature:.2f} °C")
    print(f"Pressure: {pressure:.2f} hPa")
    time.sleep(2)
```

Apos isso, o console do Thonny deve te enviar a temperatura e a pressao medidas a cada 2 segundos.
