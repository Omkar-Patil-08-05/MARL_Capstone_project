#!/usr/bin/env python3

import re
from pathlib import Path

SOURCE = Path("generated_worlds/urban_clean.sdf")
OUTDIR = Path("generated_worlds/incremental")

OUTDIR.mkdir(exist_ok=True)

text = SOURCE.read_text()

header_match = re.search(r'^(.*?)<include>', text, re.S)
footer_match = re.search(r'(<wind>.*?</sdf>)', text, re.S)

if not header_match:
    raise RuntimeError("Couldn't find header")

if not footer_match:
    raise RuntimeError("Couldn't find footer")

header = header_match.group(1)
footer = footer_match.group(1)

includes = re.findall(r'<include>.*?</include>', text, re.S)

print(f"Found {len(includes)} includes")

tests = [1, 3, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, len(includes)]

for n in tests:
    world = header
    world += "\n\n".join(includes[:n])
    world += "\n\n"
    world += footer

    outfile = OUTDIR / f"urban_{n}.sdf"
    outfile.write_text(world)

    print(f"Created {outfile}")
