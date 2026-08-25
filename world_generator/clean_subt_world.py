#!/usr/bin/env python3

from pathlib import Path
import xml.etree.ElementTree as ET

INPUT = Path.home() / "capstone_project/external_worlds/subt/subt_ign/worlds/urban_circuit_practice_03.sdf"
OUTPUT = Path.home() / "capstone_project/generated_worlds/urban_clean.sdf"

tree = ET.parse(INPUT)
root = tree.getroot()

world = root.find("world")

removed_models = 0
removed_plugins = 0

# --------------------------------------------------
# Remove Gas includes
# --------------------------------------------------
for include in list(world.findall("include")):

    uri = include.find("uri")

    if uri is None:
        continue

    if "Gas" in uri.text:
        world.remove(include)
        removed_models += 1

# --------------------------------------------------
# Remove dummy plugin
# --------------------------------------------------
for plugin in list(world.findall("plugin")):

    filename = plugin.attrib.get("filename", "")

    if filename == "dummy":
        world.remove(plugin)
        removed_plugins += 1

# --------------------------------------------------
# Replace old Fuel URLs
# --------------------------------------------------
for uri in world.iter("uri"):

    if uri.text:

        uri.text = uri.text.replace(
            "https://fuel.ignitionrobotics.org",
            "https://fuel.gazebosim.org"
        )

tree.write(
    OUTPUT,
    encoding="utf-8",
    xml_declaration=True
)

print(f"Saved : {OUTPUT}")
print(f"Removed Gas models : {removed_models}")
print(f"Removed plugins : {removed_plugins}")
