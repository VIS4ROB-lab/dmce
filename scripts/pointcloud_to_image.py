#!/usr/bin/env python

# Warning: this is a very dumb script.
# A sane person would use laspy's built-in functionality to read the points in chunks and
# reduce memory usage. Also, there's probably meta-data somewhere to find min, max, etc.
# This script forgoes all that cleverness and instead throws more RAM at the problem.

import numpy as np
import laspy
from PIL import Image

resolution = 0.05 # m per pixel of final image
zrange = [0.1, 2] # [m] Range of z-values considered. Useful to isolate different floors of a building.
mark_zrange = True

with laspy.open('SubT_rawData/SubT_Urban_Circuit_Alpha_Course_Pointcloud.las') as fh:
    print('Points from Header:', fh.header.point_count)
    print("Reading point data...")
    las = fh.read()
    print('Points from data:', len(las.points))
    print("Calculating data range...")

    #  nPoints = 5_000_001
    nPoints = len(las.points)

    #  minx = np.ma.min(las.x)
    #  maxx = np.ma.max(las.x)
    #  miny = np.ma.min(las.y)
    #  maxy = np.ma.max(las.y)
    #  minz = np.ma.min(las.z)
    #  maxz = np.ma.max(las.z)
    minx = float('inf')
    maxx = float('-inf')
    miny = float('inf')
    maxy = float('-inf')
    minz = float('inf')
    maxz = float('-inf')

    for i in range(nPoints):
        minx = min(minx, las.x[i])
        maxx = max(maxx, las.x[i])
        miny = min(miny, las.y[i])
        maxy = max(maxy, las.y[i])
        minz = min(minz, las.z[i])
        maxz = max(maxz, las.z[i])

    print(f"Range: [{minx:.2f}; {maxx:.2f}] x [{miny:.2f}; {maxy:.2f}] x [{minz:.2f}; {maxz:.2f}]")
    lenx = np.absolute(maxx - minx)
    leny = np.absolute(maxy - miny)
    lenz = np.absolute(maxz - minz)
    print(f"Size: {lenx:.2f} x {leny:.2f} x {lenz:.2f}")
    sizex = int(np.ceil(lenx / resolution))
    sizey = int(np.ceil(leny / resolution))
    sizez = int(np.ceil(lenz / resolution))
    print(f"Image resolution: {sizex}x{sizey}x{sizez}")

    print("First pass: point count and mean")

    xzcount_map = np.zeros((sizez, sizex), np.int32)
    xysliceCount_map = np.zeros((sizey, sizex), np.int32)
    xycount_map = np.zeros((sizey, sizex), np.int32)
    #  zmean_map = np.zeros((sizey, sizex), np.float32)
    #  zvar_map  = np.zeros((sizey, sizex), np.float32)

    for i in range(nPoints):
        x = int(np.floor((las.x[i] - minx) / resolution))
        y = int(np.floor((las.y[i] - miny) / resolution))
        z = int(np.floor((las.z[i] - minz) / resolution))
        xzcount_map[z][x] += 1
        xycount_map[y][x] += 1

        if las.z[i] > zrange[0] and las.z[i] < zrange[1]:
            xysliceCount_map[y][x] += 1

        #  zmean_map[y][x] += (las.z[i] - minz) / lenz
        if i % 1e6 == 0:
            print(f"{i}/{len(las.points)} ({i/len(las.points)*100:.2f}%)")

    #  zmean_map /= xycount_map
    #  np.divide(zmean_map, xycount_map, zmean_map)
    #  print("Second pass: variance")

    #  for i in range(nPoints):
    #      x = int(np.floor((las.x[i] - minx) / resolution))
    #      y = int(np.floor((las.y[i] - miny) / resolution))
    #      #  z = int(np.floor((las.z[i] - minz) / resolution))
    #      zvar_map[y][x] += (las.z[i] - zmean_map[y][x]/xycount_map[y][x])**2 / xycount_map[y][x]
    #      if i % 1e6 == 0:
    #          print(f"{i}/{len(las.points)} ({i/len(las.points)*100:.2f}%)")

    if mark_zrange:
        zlow = int(np.floor((max(minz, zrange[0]) - minz) / resolution))
        zhigh = int(np.floor((min(maxz, zrange[1]) - minz) / resolution))
        for i in range(sizex):
            xzcount_map[zlow][i] = 2**16
            xzcount_map[zhigh][i] = 2**16

    #  xzcount_map = xzcount_map.flatten()
    #  xycount_map = xycount_map.flatten()
    #  zmean_map = zmean_map.flatten()
    #  zvar_map  = zvar_map.flatten()

    print("Exporting to images...")
    xysliceMaxCount = np.ma.max(xysliceCount_map)
    xymaxCount = np.ma.max(xycount_map)
    xzmaxCount = np.ma.max(xycount_map)
    print(f"xymaxCount = {xymaxCount}")

    img = Image.new('I', (sizex, sizez))
    img.putdata(xzcount_map.flatten(), scale=2**16/xzmaxCount)
    img.save("xzcount_map.png")

    img = Image.new('I', (sizex, sizey))
    img.putdata(xycount_map.flatten(), scale=2**16/xymaxCount)
    img.save("xycount_map.png")

    img = Image.new('I', (sizex, sizey))
    img.putdata(xysliceCount_map.flatten(), scale=2**16/xysliceMaxCount)
    img.save("xysliceCount_map.png")

    #  img2 = Image.new('I', (sizex, sizey))
    #  img2.putdata(zmean_map)
    #  img2.save("zmean_map.png")

    #  img3 = Image.new('I', (sizex, sizey))
    #  img3.putdata(zvar_map)
    #  img3.save("zvar_map.png")

