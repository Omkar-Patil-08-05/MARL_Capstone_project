from terrain import ground
from terrain import roads
from city import generate_city

def world():
    city_xml, metadata = generate_city()
    
    sdf = f"""<?xml version="1.0"?>

<sdf version="1.9">

<world name="realistic_sar">
<physics type="ode">
<max_step_size>0.004</max_step_size>
<real_time_factor>1.0</real_time_factor>
<real_time_update_rate>250</real_time_update_rate>
</physics>
<gravity>0 0 -9.8</gravity>
<atmosphere type="adiabatic"/>

<magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>

<spherical_coordinates>
  <surface_model>EARTH_WGS84</surface_model>
  <world_frame_orientation>ENU</world_frame_orientation>
  <latitude_deg>47.397971057728974</latitude_deg>
  <longitude_deg>8.546163739800146</longitude_deg>
  <elevation>0</elevation>
</spherical_coordinates>

{ground()}

{roads()}

{city_xml}

</world>

</sdf>
"""
    return sdf, metadata