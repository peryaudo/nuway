# Static per-town lane graph, published latched by map_server (02 §3.3).
std_msgs/Header header
Lane[] lanes
TrafficLightMapping[] traffic_lights
StopSign[] stop_signs
Crosswalk[] crosswalks
bool has_georeference             # from the <geoReference> header of map.xodr (M5 §2.4)
float64 geo_lat0                  # degrees
float64 geo_lon0                  # degrees
float64 geo_alt0                  # meters
