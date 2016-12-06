#!/usr/bin/env python2


import csv
import time
import numpy as np
import paraview.simple as par
import paraview.servermanager as srv

PATH_TO_FILE = '/home/dlandry/repos/pclem/build/output.csv'

class Ellipsoid:
    def __init__(self, axes=(0.,0.,0.), center=(0.,0.,0.), rotation=np.zeros((3,3)), opacity=1.0):
        self.axes = axes
        self.center = center
        self.rotation = rotation
        self.opacity = opacity

    def push_to_paraview(self):
        sphere = par.Sphere()
        t = par.Transform(sphere)
        t.Transform.Scale[0:3] = self.axes[0:3]

        display = par.Show()

        display.UserTransform = [self.rotation[0,0],self.rotation[0,1],self.rotation[0,2],self.center[0],
                                 self.rotation[1,0],self.rotation[1,1],self.rotation[1,2],self.center[1],
                                 self.rotation[2,0],self.rotation[2,1],self.rotation[2,2],self.center[2],
                                 0.0,0.0,0.0,1.0]

        display.Opacity = self.opacity * 1.5

    @classmethod
    def from_string_array(cls, strings):
        floats = map(lambda x: float(x), strings)

        axes = (floats[0], floats[1], floats[2])
        center = (floats[3], floats[4], floats[5])

        rotation = np.empty((3,3))
        for i in range(9):
            rotation[i / 3, i % 3] = floats[6+i]

        opacity = floats[15]

        return cls(axes, center, rotation)

with open(PATH_TO_FILE) as csv_file:
    csv_reader = csv.reader(csv_file, delimiter=',')
    csv_reader.next() # Skip the header

    for row in csv_reader:
        begin = time.time()
        e = Ellipsoid.from_string_array(row)
        print('From string array took %f' % (time.time() - begin))
        begin = time.time()
        e.push_to_paraview()
        print('Push to paraview took %f' % (time.time() - begin))


par.Render()
