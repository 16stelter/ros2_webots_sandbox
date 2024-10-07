#!/usr/bin/env python3

import numpy as np
from stl import mesh
import sys

if len(sys.argv) != 3:
    print(f"Usage: {sys.argv[0]} <infile> <outfile>")
    exit(1)

infile_path = sys.argv[1]
outfile_path = sys.argv[2]

map_mesh = mesh.Mesh.from_file(infile_path)
vertices = map_mesh.vectors.reshape(-1, 3)

xdim = len(np.unique(vertices[:, 0]))
ydim = len(np.unique(vertices[:, 1]))

xscale = np.mean(np.diff(np.sort(np.unique(vertices[:,0]))))
yscale = np.mean(np.diff(np.sort(np.unique(vertices[:,1]))))

unique_vertices = np.unique(vertices, axis=0)
sorted_vertices = unique_vertices[np.lexsort((unique_vertices[:, 1], unique_vertices[:, 0]))]

elevation = sorted_vertices[sorted_vertices[:, 2] != np.min(unique_vertices[:,2])][:, 2].tolist()

print("xdim: " + str(xdim) + " ydim: " + str(ydim) + " xscale: " + str(xscale) + " yscale: " + str(yscale))
print("Map size should be: " + str(xdim*ydim))
print("Map size is: " + str(len(elevation)))

with open(outfile_path, 'a') as f:
    f.write("#VRML_SIM R2023b utf8\n")
    f.write('PROTO terrainmap [\n]\n{\nSolid {\ncontactMaterial "floor"\n children [\nTransform {\nchildren [\n DEF WORLDGRID Shape {\n')
    f.write('appearance PBRAppearance { \nbaseColor 0.25 0.25 0.25\nroughness 0.8\nmetalness 0\nbaseColorMap ImageTexture {\nurl ["./textures/white_noise.png"]\nfiltering 5\n}\ntextureTransform TextureTransform { scale 200 200 }\n }')
    f.write("geometry ElevationGrid { \n")
    f.write("height " + str(elevation) + "\n")
    f.write("xDimension " + str(xdim) + "\n")
    f.write("xSpacing " + str(xscale) + "\n")
    f.write("yDimension " + str(ydim) + "\n")
    f.write("ySpacing " + str(yscale) + "\n")
    f.write("thickness 1 \n }\n}\n]\n}\n]\n")
    f.write("boundingObject USE WORLDGRID}\n}\n")
