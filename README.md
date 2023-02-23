# webots_simulation

simulation backup for the webots simulator

## Recompile

1. add path parameter python path

   ```bash
    export WEBOTS_HOME=/Applications/Webots.app
    export WEBOTS_HOME_PATH=$WEBOTS_HOME/Contents
    export DYLD_LIBRARY_PATH=${WEBOTS_HOME}/Contents/lib/controller:$DYLD_LIBRARY_PATH
    export PYTHONIOENCODING=UTF-8
    export PYTHONPATH=${WEBOTS_HOME}/Contents/lib/controller/python:$PYTHONPATH
    ```

2. Set the robot controller to `extern`
3. Run the python script from anywhere

## Changelog

* Init project
* Add new controller `move_pick.py`
* Finish the base movement // 2023.2.15
* Use local proto for the robot
* Add camera to the robot
* Add read urdf file function
* add ur10 robot //2023.2.23
* Make URDF reader a class
* Format joint info to a dict list