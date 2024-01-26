[![E+E_Logo](./images/epluse-logo.png)](https://www.epluse.com/en/)

# TEE301 I2C with Raspberry Pi 


![TEE301](./images/TEE301.png) 


<!--[![button1](./images/learn-more.png)](https://epluse.com/products/temperature-measurement/temperature-sensing-element/tee301/)   -->
[![button2](./images/data-sheet.png)](https://www.epluse.com/fileadmin/data/product/tee301/datasheet_TEE301.pdf) 



## QUICK START GUIDE  

### Components 
- TEE301
- Raspberry Pi 4
- Breadboard 
- Wire jumper cable <br>

| Step |                                                                                                                                                             |
|------|-------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 1    | Connect the TEE301 sensor module with Raspberry Pi according to the following scheme:<br>__Please note: For this connection scheme the I2C address is 0x4A.__ <br>  [<img src="images/TEE301_rpi.png" width="50%"/>](images/TEE301_rpi.png)|
| 2    | Download and install the operating system (https://www.raspberrypi.org/software/operating-systems/).                                                            |
| 3    | Boot Raspberry Pi and complete any first-time setup if necessary (find instructions online).                                                                |
| 4    | Activate I2C communication:https://github.com/fivdi/i2c-bus/blob/master/doc/raspberry-pi-i2c.md                     |
| 5    | Download and install the "smbus2" library on the Raspberry Pi. [Instruction](https://pypi.org/project/smbus2/#:~:text=Installation%20instructions)            |
| 6    | Clone the repository: ```git clone https://github.com/epluse/tee301_i2c_rpi.git```  |
| 7    | Open a command shell and type following command to receive measurement data – that’s it! |


### Example output

```shell
pi@raspberrypi:~ $ python3 tee301_i2c_single_shot.py
	temperature
	23.41 °C
```
<br>


### I2C address
The sensor‘s I2C base address is 0x4A (without R/W bit). Pins A1 and A2 overrule the I2C base address.(for more information check the data sheet) <br>
### No matching I2C address
You have two ways to match the I2C addresses. The first would be to call the library in the Python file with the correct address. Or you can change the connections to the pins A1 and A2.
<br> 
<br>

## License 
See [LICENSE](LICENSE).
