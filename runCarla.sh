/opt/carla-simulator/CarlaUE4.sh quality-level=Low -no-rendering &
sleep 5s;
python3 /opt/carla-simulator/PythonAPI/util/config.py --map Town06
python3 tickMaster.py