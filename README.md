# lidar_building_footprint

## How to use

```bash
python3 -m venv venv/footprint
source venv/footprint/bin/activate
python3 -m pip install -U pip
python3 -m pip install concave-hull/
python3 -m pip install pymintriangle/
python3 -m pip install building-boundary/
python3 -m pip install -r requirements.txt
```
It might be necessary to install `libflann-dev` if not present on your system to build concave-hull (`sudo apt install libflann-dev`)

### Step 1

Create buildings cluster using pdal with or without taking into account the 3rd dimension

```bash
pdal pipeline pdal_pipeline/cluster2D.json # 2D cluster
pdal pipeline pdal_pipeline/cluster3D.json # 3D cluster
```

### Step 2

Use the script `create_footprints_geodan.py` to create a geojson containing building footprints. 
Some options inside the script may be adapted.
