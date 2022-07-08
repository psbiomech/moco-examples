# -*- coding: utf-8 -*-
"""
Create empty model with contact geometry

@author: Prasanna Sritharan
"""

import opensim as osim


# empty model
model = osim.Model()


# create contact mesh geometry
cgeom = osim.ContactMesh("plate.stl", osim.Vec3(0.0, 0.0, 0.0), osim.Vec3(0.0, 0.0, 0.0), model.updGround())

# appearance
# appear = osim.Appearance()
# appear.set_visible(True)
# appear.set_color(osim.Vec3(0.0, 0.0, 1.0))
# appear.set_opacity(1.0)
# cgeom.set_Appearance(appear)


hcforce = osim.HuntCrossleyForce()

# add the geometry
model.addContactGeometry(cgeom)
model.addForce(hcforce)

# print
model.finalizeConnections()
model.printToXML("plate.osim", )