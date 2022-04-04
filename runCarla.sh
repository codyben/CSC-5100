/opt/carla-simulator/CarlaUE4.sh -fps=5 -quality-level=Low -no-rendering &
sleep 3s;
python3 /opt/carla-simulator/PythonAPI/util/config.py --map Town06
python3 tickMaster.py