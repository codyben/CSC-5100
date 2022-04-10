/opt/carla-simulator/CarlaUE4.sh -fps=20 quality-level=Low --no-rendering --tag &
sleep 5s;
python3 /opt/carla-simulator/PythonAPI/util/config.py --map Town04
sleep 2s;
python3 tickMaster.py