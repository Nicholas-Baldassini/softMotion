# Install


Tested on python version 3.10.12
```
virtualenv softMotionEnv
source softMotionEnv/bin/activate
```
May have to run sudo apt install python3-virtualenv if this command is not found.

```
pip install -r requirements.txt
```



The robot parameters are defined in robotConfig.py, you can change the length, elastic properties and so much more.

do
```
python simulate.py
```
right now the robot can be controlled by arrow keys, which applies a constant force to in the corresponding direction each time frame.