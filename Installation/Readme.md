* Install Arduino from the website  : https://docs.arduino.cc/software/ide-v1/tutorials/Linux
* Install rosserial 

```
sudo apt-get install ros-<distro>-rosserial
sudo apt-get install ros-<distro>-rosserial-arduino

```

* Do a dialout 

```
sudo usermod -a -G dialout YOUR_USER_NAME

```

* Give port access (777 gives a full read write access)

```
sudo chmod 777 /dev/ttyUSB*

```

* Intall rosserial for the arduino library
