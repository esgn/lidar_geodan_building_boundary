# lidar_building_footprint

## How to use

```
python3 -m venv venv/footprint
source venv/footprint/bin/activate
python3 -m pip install -U pip
python3 -m pip install concave-hull/
python3 -m pip install pymintriangle/
python3 -m pip install building-boundary/
pip3 install -r requirements.txt
```
It might be necessary to install `libflann-dev` if not present on your system to build concave-hull (`sudo apt install libflann-dev`)