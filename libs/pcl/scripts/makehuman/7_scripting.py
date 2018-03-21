#!/usr/bin/python2.7
# -*- coding: utf-8 -*-

"""
**Project Name:**      MakeHuman

**Product Home Page:** http://www.makehuman.org/

**Code Home Page:**    https://bitbucket.org/MakeHuman/makehuman/

**Authors:**           Joel Palmius

**Copyright(c):**      MakeHuman Team 2001-2017

**Licensing:**         AGPL3

    This file is part of MakeHuman (www.makehuman.org).

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU Affero General Public License as
    published by the Free Software Foundation, either version 3 of the
    License, or (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Affero General Public License for more details.

    You should have received a copy of the GNU Affero General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.


Abstract
--------

TODO
"""

# We need this for gui controls
import gui3d
import mh
import os
import io
import math
import material
import gui
import log
import os
from cStringIO import StringIO
from core import G
from codecs import open
import bvh
from export import ExportConfig
from datetime import datetime

import wavefront
import os
from progress import Progress
import numpy as np

# Bone used for determining the pose scaling (root bone translation scale)
COMPARE_BONE = "upperleg02.L"
AKIMBO01_BVH_PATH = "data/poses/akimbo01.bvh"

class ObjConfig(ExportConfig):

    def __init__(self):
        ExportConfig.__init__(self)
        self.useRelPaths = True
        self.useNormals = False
        self.hiddenGeom = False


def exportObj(filepath, config=None):
    human = config.human
    config.setupTexFolder(filepath)
    filename = os.path.basename(filepath)

    log.message("Collecting Objects" + str(datetime.now()))
    objects = human.getObjects(excludeZeroFaceObjs=not config.hiddenGeom)
    meshes = [o.mesh for o in objects]

    log.message("hiddenGeom" + str(config.hiddenGeom))

    if config.hiddenGeom:
        # Disable the face masking on copies of the input meshes
        meshes = [m.clone(filterMaskedVerts=False) for m in meshes]
        for m in meshes:
            # Would be faster if we could tell clone() to do this, but it would
            # make the interface more complex.
            # We could also let the wavefront module do this, but this would
            # introduce unwanted "magic" behaviour into the export function.
            face_mask = np.ones(m.face_mask.shape, dtype=bool)
            m.changeFaceMask(face_mask)
            m.calcNormals()
            m.updateIndexBuffer()

    log.message("Writing Objects" + str(datetime.now()))
    wavefront.writeObjFile(filepath, meshes, False, config, filterMaskedFaces=not config.hiddenGeom)
    log.message("OBJ Export finished. Output file: %s" % filepath + str(datetime.now()))


def writePCDFile(path, meshes, writeMTL=True, config=None, filterMaskedFaces=True):
    if not isinstance(meshes, list):
        meshes = [meshes]

    if isinstance(path, file):
        fp = path
    else:
        fp = open(path, 'w')

    fp.write(
        "# .PCD v0.7 - Point Cloud Data file format\n" +
        "VERSION 0.7\n" +
        "FIELDS x y z\n" +
        "SIZE 4 4 4\n" +
        "TYPE F F F\n" +
        "COUNT 1 1 1\n" +
        "WIDTH 14444\n" +
        "HEIGHT 1\n" +
        "VIEWPOINT 0 0 0 1 0 0 0\n" +
        "POINTS 14444\n" +
        "DATA ascii\n")

    scale = config.scale if config is not None else 1.0

    # Scale and filter out masked faces and unused verts
    if filterMaskedFaces:
        meshes = [m.clone(scale=scale, filterMaskedVerts=True) for m in meshes]
    else:
        # Unfiltered
        meshes = [m.clone(scale=scale, filterMaskedVerts=False) for m in meshes]

    if config and config.feetOnGround:
        offset = config.offset
    else:
        offset = [0,0,0]

    # Vertices
    for mesh in meshes:
        values = [tuple(co + offset) for co in mesh.coord]
        fp.write("".join(["%.4f %.4f %.4f\n" % (val[2], val[0], val[1]) for val in values]))

    fp.close()


def exportPCD(filepath, config=None):
    human = config.human
    config.setupTexFolder(filepath)

    log.message("Collecting Objects" + str(datetime.now()))
    objects = human.getObjects(excludeZeroFaceObjs=not config.hiddenGeom)
    meshes = [o.mesh for o in objects]

    log.message("Writing Objects" + str(datetime.now()))
    writePCDFile(filepath, meshes, False, config, filterMaskedFaces=not config.hiddenGeom)
    log.message("OBJ Export finished. Output file: %s" % filepath + str(datetime.now()))

# FROM 7_weight_estimation.py

def calculateSurface(mesh, vertGroups=None, faceMask=None):
    """
    Calculate surface area of a mesh. Specify vertGroups or faceMask to
    calculate area of a subset of the mesh and filter out other faces.
    """
    if vertGroups is not None:
        fvert = mesh.getFacesForGroups(vertGroups)
    elif faceMask is not None:
        f_idx = np.argwhere(faceMask)[...,0]
        fvert = mesh.fvert[f_idx]
    else:
        fvert = mesh.fvert

    if mesh.vertsPerPrimitive == 4:
        # Split quads in triangles (assumes clockwise ordering of verts)
        t1 = fvert[:,[0,1,2]]
        t2 = fvert[:,[2,3,0]]
        v1 = mesh.coord[t1]
        v2 = mesh.coord[t2]

        l1 = _sideLengthsFromTris(v1)
        l2 = _sideLengthsFromTris(v2)
        l = np.vstack([l1,l2])

        return _surfaceOfTris(l)
    elif mesh.vertsPerPrimitive == 3:
        v = mesh.coord[fvert]
        l = _sideLengthsFromTris(v)
        return _surfaceOfTris(l)
    else:
        raise RuntimeError("Only supports meshes with triangle or quad primitives.")

def _sideLengthsFromTris(triVects):
    """
    Calculate lengths of the sides of triangles specified by their vectors
    in clockwise fashion.
    triVects = [ [T1V1, T1V2, T1V3], [T2V1, T2V2, T2V3], ... ]
    with Ti a triangle, Vi a triange vector, defined in clockwise fashion
    and each vector (TiVi) an array [x, y, z] with vector coordinates

    Returns a list [ [T1L1, T1L2, T1L3], [T2L1, T2L2, T2L3], ...]
    with Ti a triangle (in the same order as in the input), and Li the length of
    side i (a float)
    """
    v = triVects
    s = np.zeros(v.shape, dtype=np.float32)

    # Get side vectors
    s[:,0] = v[:,1] - v[:,0]
    s[:,1] = v[:,2] - v[:,1]
    s[:,2] = v[:,0] - v[:,2]

    # Calculate lengths of sides
    l = s[:,:,0]*s[:,:,0] + s[:,:,1]*s[:,:,1] + s[:,:,2]*s[:,:,2]
    l = np.sqrt(l)

    return l

def _surfaceOfTris(triSideLengths):
    """
    Calculate total surface area of triangles with sides of specified lengths
    triSideLengths should be an array of layout
    [ [T1L1, T1L2, T1L3], [T2L1, T2L2, T2L3], ... ]
    with Ti a triangle, and Li the length of the ith side of the triangle
    TiLi should be a float.

    Returns a float representing the total surface area.
    """
    l = triSideLengths

    # Heron's formula
    o = ( l[:,0]  +l[:,1]  +l[:,2]) * \
        ( l[:,0]  +l[:,1]  -l[:,2]) * \
        (-l[:,0]  +l[:,1]  +l[:,2]) * \
        ( l[:,0]  -l[:,1]  +l[:,2])
    o = np.sqrt(o)/4

    return np.sum(o)

def calculateWeight(human):
    humanMesh = human.meshData

    height = human.getHeightCm()
    bsa = calculateSurface(humanMesh, faceMask=humanMesh.getFaceMask())/100

    #Shuter and Aslani, 2000
    weight_aslani = np.power(bsa /(np.power(height, 0.655) * 0.00949), 1/0.441)
    #DuBois and DuBois, 1916
    weight_DuBois_1916=np.power(bsa /(np.power(height, 0.725) * 0.007184), 1/0.425)
    #Reading and Freeman, 2005
    weight_Reading=np.power(bsa /(np.power(height, 0.5) /60), 1/0.5)
    #Wang and Hihara, 2004
    weight_Wang=np.power(bsa /(np.power(height, 0.5) * 0.0168), 1/0.5)
    #Livingston and Lee 2001
    weight_Livingston=np.power(bsa / 0.1173, 1/0.6466)
    #Mosteller, 1987
    weight_Mosteller=np.power(bsa /(np.power(height, 0.5) * 0.0167), 1/0.5)
    #Anderson et al., 1985
    weight_Anderson=np.power(bsa /(np.power(height, 0.417) * 0.0239), 1/0.517)
    #Haycock et al., 1978
    weight_Haycock=np.power(bsa /(np.power(height, 0.3964) * 0.024265), 1/0.5378)

    return [weight_aslani, weight_DuBois_1916, weight_Reading, weight_Wang, weight_Livingston, weight_Mosteller,
            weight_Anderson, weight_Haycock]

# END - FROM 7_weight_estimation.py

class ScriptingView(gui3d.TaskView):

    def __init__(self, category):
        gui3d.TaskView.__init__(self, category, 'Scripting')

        self.directory = os.getcwd()
        self.filename =  None

        box = self.addLeftWidget(gui.GroupBox('Script'))

        self.scriptText = self.addTopWidget(gui.DocumentEdit())
        self.scriptText.setText('');

        self.scriptText.setLineWrapMode(gui.DocumentEdit.NoWrap)

        self.loadButton = box.addWidget(gui.BrowseButton(mode='open'), 0, 0)
        self.loadButton.setLabel('Load ...')
        self.loadButton.directory = mh.getPath()
        self.saveButton = box.addWidget(gui.BrowseButton(mode='save'), 0, 1)
        self.saveButton.setLabel('Save ...')
        self.saveButton.directory = mh.getPath()

        @self.loadButton.mhEvent
        def onClicked(filename):
            if not filename:
                return

            if(os.path.exists(filename)):
                contents = io.open(filename, 'rU', encoding="utf-8").read()

                self.scriptText.setText(contents)
                dlg = gui.Dialog()
                dlg.prompt("Load script","File was loaded in an acceptable manner","OK")
                self.filename = filename
                self.directory = os.path.split(filename)[0]
            else:
                dlg = gui.Dialog()
                dlg.prompt("Load script","File %s does not exist","OK", fmtArgs=filename)

        @self.saveButton.mhEvent
        def onClicked(filename):
            if not filename:
                return

            with open(filename, "w", encoding="utf-8") as f:
                f.write(self.scriptText.getText())
            dlg = gui.Dialog()
            dlg.prompt("Save script","File was written in an acceptable manner","OK")
            self.filename = filename
            self.directory = os.path.split(filename)[0]

        box2 = self.addLeftWidget(gui.GroupBox('Examples'))

        self.insertLabel = box2.addWidget(gui.TextView('Append example to script'))
        self.listView = box2.addWidget(gui.ListView())
        self.listView.setSizePolicy(gui.SizePolicy.Ignored, gui.SizePolicy.Preferred)

        testlist = [ 
            'applyTarget()', 
            'incrementingFilename()',
            'getAgeYears()',
            'getHeightCm()',
            'getChestCm()',
            'getWaistCm()',
            'getHipsCm()',
            'getModelingParameters()',
            'getPositionX()',
            'getPositionY()',
            'getPositionZ()',
            'getRotationX()',
            'getRotationY()',
            'getRotationZ()',
            'getWeightInKg()',
            'getZoom()',
            'loadModel()',
            'modifyPositionX()',
            'modifyPositionY()',
            'modifyPositionZ()',
            'modifyRotationX()',
            'modifyRotationY()',
            'modifyRotationZ()',
            'modifyZoom()',
            'printCameraInfo()',
            'printDetailStack()',
            'printPositionInfo()',
            'printRotationInfo()',
            'reset()',
            'saveModel()',
            'screenShot()',
            'setAge()',
            'setAgeYears()',
            'setBodyProportions()',
            'setHeadSquareness()',
            'setGender()',
            'setHeight()',
            'setMaterial()',
            'setMuscle()',
            'setPositionX()',
            'setPositionY()',
            'setPositionZ()',
            'setRotationX()',
            'setRotationY()',
            'setRotationZ()',
            'setZoom()',
            'setWeight()',
            'updateModelingParameter()',
            'updateModelingParameters()',
            'saveObj()'
        ]

        self.listView.setData(testlist)

        self.insertButton = box2.addWidget(gui.Button('Append'))

        @self.insertButton.mhEvent
        def onClicked(event):
            item = self.listView.getSelectedItem()

            if(item == 'applyTarget()'):
                text = "# applyTarget(<target file name>, <power (from 0.0 to 1.0)>)\n"
                text = text + "#\n"
                text = text + "# This will apply the target on the model. If the target was already applied, the power will be updated\n"
                text = text + "# Note that targets are relative to the data/targets directory, and should not include the .target\n"
                text = text + "# extension, so a valid target name would be, for example, \"breast/breast-dist-max\"\n\n"
                text = text + "MHScript.applyTarget('aTargetName',1.0)\n\n"
                self.scriptText.addText(text)

            if(item == 'loadModel()'):
                text = "# loadModel(<model name>,[path])\n"
                text = text + "#\n"
                text = text + "# This will load a human model from an MHM file. The <model name> part should be a string without spaces\n"
                text = text + "# and without the .MHM extension. The [path] part defaults to the user's makehuman/models directory.\n\n"
                text = text + "MHScript.loadModel('myTestModel')\n\n"
                self.scriptText.addText(text)

            if(item == 'incrementingFilename()'):
                text = "# incrementingFilename(<file name base>, [file extension], [pad length])\n"
                text = text + "#\n"
                text = text + "# This will return a file name containing a numerical component which increases by one for each call.\n"
                text = text + "# The default file extension is \".png\". The default pad length is 4. For example, the following lines:\n";
                text = text + "#\n"
                text = text + "# print incrementingFilename(\"test\",\".target\",3) + \"\\n\"\n"
                text = text + "# print incrementingFilename(\"test\",\".target\",3) + \"\\n\"\n"
                text = text + "#\n"
                text = text + "# Will print:\n"
                text = text + "#\n"
                text = text + "# test001.target\n"
                text = text + "# test002.target\n"
                text = text + "#\n"
                text = text + "# The counter is reset each time the script is executed\n\n"
                text = text + "filename = MHScript.incrementingFilename('test')\n\n"
                self.scriptText.addText(text)

            if(item == 'printCameraInfo()'):
                text = "# printCameraInfo()\n"
                text = text + "#\n"
                text = text + "# This will print info about how the camera is targeted and focused .\n\n"
                text = text + "MHScript.printCameraInfo()\n\n"
                self.scriptText.addText(text)

            if(item == 'printDetailStack()'):
                text = "# printDetailStack()\n"
                text = text + "#\n"
                text = text + "# This will print a list of all applied targets (and their weights) to standard output.\n\n"
                text = text + "MHScript.printDetailStack()\n\n"
                self.scriptText.addText(text)

            if(item == 'printPositionInfo()'):
                text = "# printPositionInfo()\n"
                text = text + "#\n"
                text = text + "# This will print info about where the human object is currently located.\n\n"
                text = text + "MHScript.printPositionInfo()\n\n"
                self.scriptText.addText(text)

            if(item == 'printRotationInfo()'):
                text = "# printRotationInfo()\n"
                text = text + "#\n"
                text = text + "# This will print info about how the human object is currently rotated.\n\n"
                text = text + "MHScript.printRotationInfo()\n\n"
                self.scriptText.addText(text)
            if(item == 'reset()'):
                text = "# reset()\n"
                text = text + "#\n"
                text = text + "# This will reset the human model to default.\n\n"
                text = text + "MHScript.reset()\n\n"
                self.scriptText.addText(text)
            if(item == 'saveModel()'):
                text = "# saveModel(<model name>,[path])\n"
                text = text + "#\n"
                text = text + "# This will save the human model to an MHM file. The <model name> part should be a string without spaces\n"
                text = text + "# and without the .MHM extension. The [path] part defaults to the user's makehuman/models directory.\n"
                text = text + "# Note that this will not save any thumbnail.\n\n"
                text = text + "MHScript.saveModel('myTestModel')\n\n"
                self.scriptText.addText(text)

            if(item == 'saveObj()'):
                text = "# saveObj(<model name>,[path])\n"
                text = text + "#\n"
                text = text + "# This will save the human model to a wavefront .OBJ file. The <model name> part should be a string without spaces\n"
                text = text + "# and without the .obj extension. The [path] part defaults to the user's makehuman/exports directory.\n"
                text = text + "MHScript.saveObj('myOBJExport')\n\n"
                self.scriptText.addText(text)

            if(item == 'screenShot()'):
                text = "# screenShot(<png file name>)\n"
                text = text + "#\n"
                text = text + "# This will save a png file of how the model currently looks.\n\n"
                text = text + "MHScript.screenShot('screenshot.png')\n\n"
                self.scriptText.addText(text)

            if(item == 'setAge()'):
                text = "# setAge(age)\n"
                text = text + "#\n"
                text = text + "# Sets the age of the model. The age parameter is a float between 0 and 1, where 0 is 1 year old, 0.18 is 10 years old, 0.5 is 25 years and 1 equals 90 years old.\n\n"
                text = text + "MHScript.setAge(0.5)\n\n"
                self.scriptText.addText(text)

            if(item == 'getAgeYears()'):
                text = "# getAgeYears()\n"
                text = text + "#\n"
                text = text + "# Get the age of the model in years\n"
                text = text + "MHScript.getAgeYears()\n\n"
                self.scriptText.addText(text)

            if(item == 'setAgeYears()'):
                text = "# setAgeYears(age)\n"
                text = text + "#\n"
                text = text + "# Sets the age of the model. The age parameter is a float between 0 and 90, in years.\n\n"
                text = text + "MHScript.setAgeYears(25)\n\n"
                self.scriptText.addText(text)

            if(item == 'setWeight()'):
                text = "# setWeight(weight)\n"
                text = text + "#\n"
                text = text + "# Sets the weight of the model. The weight parameter is a float between 0 and 1, where 0 is starved and\n"
                text = text + "# 1 is severely overweight\n\n"
                text = text + "MHScript.setWeight(0.5)\n\n"
                self.scriptText.addText(text)

            if(item == 'setGender()'):
                text = "# setGender(value)\n"
                text = text + "#\n"
                text = text + "# Sets the gender of the model. The gender parameter is a float between 0 and 1, where 0 is female and\n"
                text = text + "# 1 is male\n\n"
                text = text + "MHScript.setGender(0.5)\n\n"
                self.scriptText.addText(text)

            if(item == 'setHeight()'):
                text = "# setHeight(height)\n"
                text = text + "#\n"
                text = text + "# Sets the height of the model. The height parameter is a float between 0 and 1, where 0 is starved and\n"
                text = text + "# 1 is severely overheight\n\n"
                text = text + "MHScript.setHeight(0.5)\n\n"
                self.scriptText.addText(text)

            if(item == 'setBodyProportions()'):
                text = "# setBodyProportions(value)\n"
                text = text + "#\n"
                text = text + "MHScript.setBodyProportions(0.5)\n\n"
                self.scriptText.addText(text)

            if(item == 'setMuscle()'):
                text = "# setMuscle(value)\n"
                text = text + "#\n"
                text = text + "MHScript.setMuscle(0.5)\n\n"
                self.scriptText.addText(text)

            if(item == 'setHeadSquareness()'):
                text = "# setHeadSquareness(squareness)\n"
                text = text + "#\n"
                text = text + "# Sets the squaredness of the model's head. The squareness parameter is a float between 0 and 1, where 0 is not square and\n"
                text = text + "# 1 is very square shaped\n\n"
                text = text + "MHScript.setHeadSquareness(0.5)\n\n"
                self.scriptText.addText(text)

            if(item == 'setMaterial()'):
                text = "# setMaterial(mhmat_filename)\n"
                text = text + "#\n"
                text = text + "# Sets the skin material of the 3D model\n"
                text = text + "# The filename must be realtive to the App Resources directory\n\n"
                text = text + "MHScript.setMaterial('data/skins/young_caucasian_female/young_caucasian_female.mhmat')\n\n"
                self.scriptText.addText(text)

            if(item == 'getHeightCm()'):
                text = "# getHeightCm()\n"
                text = text + "#\n"
                text = text + "# Gets the current height of the model, in cm.\n\n"
                text = text + "height = MHScript.getHeightCm()\n"
                text = text + "print('height='+str(height))\n\n"
                self.scriptText.addText(text)

            if(item == 'getChestCm()'):
                text = "# getChestCm()\n"
                text = text + "#\n"
                text = text + "# Gets the current chest circumference of the model, in cm.\n\n"
                text = text + "chest = MHScript.getChestCm()\n"
                text = text + "print('chest ='+str(chest))\n\n"
                self.scriptText.addText(text)

            if(item == 'getWaistCm()'):
                text = "# getWaistCm()\n"
                text = text + "#\n"
                text = text + "# Gets the current waist circumference of the model, in cm.\n\n"
                text = text + "waist = MHScript.getWaistCm()\n"
                text = text + "print('waist ='+str(waist))\n\n"
                self.scriptText.addText(text)

            if(item == 'getHipsCm()'):
                text = "# getHipsCm()\n"
                text = text + "#\n"
                text = text + "# Gets the current hips circumference of the model, in cm.\n\n"
                text = text + "hips = MHScript.getHipsCm()\n"
                text = text + "print('hips ='+str(hips))\n\n"
                self.scriptText.addText(text)

            if(item == 'getModelingParameters()'):
                text = "# getModelingParameters()\n"
                text = text + "#\n"
                text = text + "# Prints the names of all modeling aspects that can be modified on the human model.\n"
                text = text + "MHScript.getModelingParameters()\n\n"
                self.scriptText.addText(text)

            if(item == 'updateModelingParameter()'):
                text = "# updateModelingParameter(parameterName, value)\n"
                text = text + "#\n"
                text = text + "# Sets the modeling parameter with specified name of the model to the specified value.\n"
                text = text + "# The value is a float between 0 and 1, where 0 means nothing at all or minimal, and 1 is the maximum value.\n\n"
                text = text + "MHScript.updateModelingParameter('macrodetails/Age', 0.7)\n\n"
                self.scriptText.addText(text)

            if(item == 'updateModelingParameters()'):
                text = "# updateModelingParameters(dictOfParameterNameAndValue)\n"
                text = text + "#\n"
                text = text + "# Sets more modeling parameters with specified names of the model to the specified values.\n"
                text = text + "# Faster than setting parameters one by one because the 3D mesh is updated only once.\n"
                text = text + "# The values are a float between 0 and 1, where 0 means nothing at all or minimal, and 1 is the maximum value.\n\n"
                text = text + "MHScript.updateModelingParameters({'macrodetails/Caucasian': 1.000,'macrodetails/Gender': 1.000,'macrodetails/Age': 0.250})\n\n"
                self.scriptText.addText(text)

            if(item == 'setPositionX()'):
                text = "# setPositionX(xpos)\n"
                text = text + "#\n"
                text = text + "# Sets the X position of the model of the model in 3d space, where 0.0 is centered.\n\n"
                text = text + "MHScript.setPositionX(2.0)\n\n"
                self.scriptText.addText(text)

            if(item == 'getPositionX()'):
                text = "# getPositionX()\n"
                text = text + "#\n"
                text = text + "# Returns the current X position of the model of the model in 3d space.\n\n"
                text = text + "MHScript.getPositionX()\n\n"
                self.scriptText.addText(text)

            if(item == 'modifyPositionX()'):
                text = "# modifyPositionX(xmod)\n"
                text = text + "#\n"
                text = text + "# Modifies X position of the model of the model in 3d space.\n\n"
                text = text + "MHScript.modifyPositionX(-0.1)\n\n"
                self.scriptText.addText(text)

            if(item == 'setPositionZ()'):
                text = "# setPositionZ(zpos)\n"
                text = text + "#\n"
                text = text + "# Sets the Z position of the model of the model in 3d space, where 0.0 is centered.\n\n"
                text = text + "MHScript.setPositionZ(2.0)\n\n"
                self.scriptText.addText(text)

            if(item == 'getPositionZ()'):
                text = "# getPositionZ()\n"
                text = text + "#\n"
                text = text + "# Returns the current Z position of the model of the model in 3d space.\n\n"
                text = text + "MHScript.getPositionZ()\n\n"
                self.scriptText.addText(text)

            if(item == 'modifyPositionZ()'):
                text = "# modifyPositionZ(zmod)\n"
                text = text + "#\n"
                text = text + "# Modifies Z position of the model of the model in 3d space.\n\n"
                text = text + "MHScript.modifyPositionZ(-0.1)\n\n"
                self.scriptText.addText(text)

            if(item == 'setPositionY()'):
                text = "# setPositionY(ypos)\n"
                text = text + "#\n"
                text = text + "# Sets the Y position of the model of the model in 3d space, where 0.0 is centered.\n"
                text = text + "# Note that the depth of the scene is clipped, so if you move the model too far back\n"
                text = text + "# it will disappear. You will most likely want to use zoom instead of Y position.\n\n";
                text = text + "MHScript.setPositionY(2.0)\n\n"
                self.scriptText.addText(text)

            if(item == 'getPositionY()'):
                text = "# getPositionY()\n"
                text = text + "#\n"
                text = text + "# Returns the current Y position of the model of the model in 3d space.\n\n"
                text = text + "MHScript.getPositionY()\n\n"
                self.scriptText.addText(text)

            if(item == 'modifyPositionY()'):
                text = "# modifyPositionY(ymod)\n"
                text = text + "#\n"
                text = text + "# Modifies Y position of the model of the model in 3d space.\n"
                text = text + "# Note that the depth of the scene is clipped, so if you move the model too far back\n"
                text = text + "# it will disappear. You will most likely want to use zoom instead of Y position.\n\n";
                text = text + "MHScript.modifyPositionY(-0.1)\n\n"
                self.scriptText.addText(text)

            if(item == 'setRotationX()'):
                text = "# setRotationX(xrot)\n"
                text = text + "#\n"
                text = text + "# Sets the rotation around the X axis for the model, where 0.0 is frontal projection.\n"
                text = text + "# Rotation is set in degrees from -180.0 to +180.0 (these two extremes are equal)\n\n"
                text = text + "MHScript.setRotationX(90.0)\n\n"
                self.scriptText.addText(text)

            if(item == 'getRotationX()'):
                text = "# getRotationX()\n"
                text = text + "#\n"
                text = text + "# Returns the current rotatation around the X axis of the model.\n\n"
                text = text + "MHScript.getRotationX()\n\n"
                self.scriptText.addText(text)

            if(item == 'modifyRotationX()'):
                text = "# modifyRotationX(xmod)\n"
                text = text + "#\n"
                text = text + "# Modifies the rotation around the X axis for the model, where 0.0 is frontal projection.\n"
                text = text + "# Rotation is set in degrees from -180.0 to +180.0 (these two extremes are equal)\n\n"
                text = text + "MHScript.modifyRotationX(-5.0)\n\n"
                self.scriptText.addText(text)

            if(item == 'setRotationZ()'):
                text = "# setRotationZ(zrot)\n"
                text = text + "#\n"
                text = text + "# Sets the rotation around the Z axis for the model, where 0.0 is frontal projection.\n"
                text = text + "# Rotation is set in degrees from -180.0 to +180.0 (these two extremes are equal)\n\n"
                text = text + "MHScript.setRotationZ(90.0)\n\n"
                self.scriptText.addText(text)

            if(item == 'getRotationZ()'):
                text = "# getRotationZ()\n"
                text = text + "#\n"
                text = text + "# Returns the current rotatation around the Z axis of the model.\n\n"
                text = text + "MHScript.getRotationZ()\n\n"
                self.scriptText.addText(text)

            if(item == 'modifyRotationZ()'):
                text = "# modifyRotationZ(zmod)\n"
                text = text + "#\n"
                text = text + "# Modifies the rotation around the Z axis for the model, where 0.0 is frontal projection.\n"
                text = text + "# Rotation is set in degrees from -180.0 to +180.0 (these two extremes are equal)\n\n"
                text = text + "MHScript.modifyRotationZ(-5.0)\n\n"
                self.scriptText.addText(text)

            if(item == 'setRotationY()'):
                text = "# setRotationY(yrot)\n"
                text = text + "#\n"
                text = text + "# Sets the rotation around the Y axis for the model, where 0.0 is upright projection.\n"
                text = text + "# Rotation is set in degrees from -180.0 to +180.0 (these two extremes are equal)\n\n"
                text = text + "MHScript.setRotationY(90.0)\n\n"
                self.scriptText.addText(text)

            if(item == 'getRotationY()'):
                text = "# getRotationY()\n"
                text = text + "#\n"
                text = text + "# Returns the current rotatation around the Y axis of the model.\n\n"
                text = text + "MHScript.getRotationY()\n\n"
                self.scriptText.addText(text)

            if(item == 'modifyRotationY()'):
                text = "# modifyRotationY(ymod)\n"
                text = text + "#\n"
                text = text + "# Modifies the rotation around the Y axis for the model, where 0.0 is upright projection.\n"
                text = text + "# Rotation is set in degrees from -180.0 to +180.0 (these two extremes are equal)\n\n"
                text = text + "MHScript.modifyRotationY(-5.0)\n\n"
                self.scriptText.addText(text)

            if(item == 'setZoom()'):
                text = "# setZoom(zoom)\n"
                text = text + "#\n"
                text = text + "# Sets current camera zoom. In practise this moves the camera closer or further from the.\n"
                text = text + "# the model. The zoom factor is reversed ans goes from 100.0 which is far away from the\n"
                text = text + "# the model as possible (if you move further away, the model will be clipped and disappear)\n"
                text = text + "# and 0.0 is inside the model. A zoom factor of 10.0 is what is used for the face\n"
                text = text + "# projection, and is in most cases as zoomed in as is functional.\n\n"
                text = text + "MHScript.setZoom(70.0)\n\n"
                self.scriptText.addText(text)

            if(item == 'modifyZoom()'):
                text = "# modifyZoom(zmod)\n"
                text = text + "#\n"
                text = text + "# Modifies current camera zoom. In practise this moves the camera closer or further from the.\n"
                text = text + "# the model. The zoom factor is reversed ans goes from 100.0 which is far away from the\n"
                text = text + "# the model as possible (if you move further away, the model will be clipped and disappear)\n"
                text = text + "# and 0.0 is inside the model. A zoom factor of 10.0 is what is used for the face\n"
                text = text + "# projection, and is in most cases as zoomed in as is functional.\n\n"
                text = text + "MHScript.modifyZoom(1.0)\n\n"
                self.scriptText.addText(text)

            if(item == 'getWeightInKg()'):
                text = "# getWeightInKg()\n"
                text = text + "#\n"
                text = text + "# Returns an array of weight in kilograms from different algorithms.\n\n"
                text = text + "weights_arr = MHScript.getWeightInKg()\n\n"
                self.scriptText.addText(text)

            if(item == 'getZoom()'):
                text = "# getZoom()\n"
                text = text + "#\n"
                text = text + "# Returns the current camera zoom factor.\n\n"
                text = text + "MHScript.getZoom()\n\n"
                self.scriptText.addText(text)
            if (item == 'setPose()'):
                text = "# setPose(posePath)\n"
                text = text + "MHScript.setPose('data/poses/akimbo1')\n\n"
                self.scriptText.addText(text)
             
    def onShow(self, event):
        gui3d.app.statusPersist('This is a rough scripting module')

    def onHide(self, event):
        gui3d.app.statusPersist('')


class ScriptingExecuteTab(gui3d.TaskView):
    def __init__(self, category):
        gui3d.TaskView.__init__(self, category, 'Execute')

        box = self.addLeftWidget(gui.GroupBox('Execute'))

        self.poseButton = box.addWidget(gui.Button('Pose'))
        self.execButton = box.addWidget(gui.Button('Execute'))
        self.resetButton = box.addWidget(gui.Button('Reset'))

        self.randomButton = box.addWidget(gui.Button('Random generator'))
        self.aProgressBar = box.addWidget(gui.ProgressBar())

        self.randomLocationTextEdit = box.addWidget(gui.TextEdit(text='D:\\mhmodels\\'))

        @self.poseButton.mhEvent
        def onClicked(event):
            global MHScript
            MHScript = Scripting()
            MHScript.reset()
            MHScript.setPose(AKIMBO01_BVH_PATH)

        @self.resetButton.mhEvent
        def onClicked(event):
            global MHScript
            MHScript = Scripting()
            MHScript.reset()

        @self.randomButton.mhEvent
        def onClicked(event):
            global MHScript
            MHScript = Scripting()
            root_dir = self.randomLocationTextEdit.text
            log.message("Random location = " + root_dir)

            # 1. Check if random location exists
            if not os.path.exists(root_dir):
                os.mkdir(root_dir)
                log.message("Create directory " + root_dir)
            model_dir = os.path.join(root_dir, "models")
            if not os.path.exists(model_dir):
                os.mkdir(model_dir)
                log.message("Create directory " + model_dir)
            dataset_file_path = os.path.join(root_dir, "dataset.txt")
            dataset_file = open(dataset_file_path, "w")

            def generate_data():
                # for gender in [0, 1]:
                #     for age in [x * 0.01 for x in range(70, 25, -20)]:
                #         for muscle in [x * 0.01 for x in range(25, 100, 20)]:
                #             for proportion in [0.2, 0.5, 0.8]:
                #                 for height in [x * 0.01 for x in range(25, 100, 20)]:
                #                     for weight in [x * 0.01 for x in range(25, 100, 15)]:
                #                         yield gender, height, weight, age, muscle, proportion
                for gender in [1]:
                    for age in [x * 0.01 for x in range(70, 25, -20)]:
                        for proportion in [x * 0.01 for x in range(0, 100, 10)]:
                            for height in [x * 0.01 for x in range(30, 70, 4)]:
                                for muscle in [x * 0.01 for x in range(0, 100, 10)]:
                                    for weight in [x * 0.01 for x in range(20, 100, 10)]:
                                        yield gender, height, weight, age, muscle, proportion

            total = 0
            for gender, height, weight, age, muscle, proportion in generate_data():
                total += 1
            log.message("Total 1 = " + str(total))

            count = 0
            dataset_file.write("Height Weight Age Gender Muscle Proportion Chest Waist Hips Path\n")
            from datetime import datetime
            for gender, height, weight, age, muscle, proportion in generate_data():
                log.message("start : " + str(datetime.now()))
                count += 1
                MHScript.reset()
                MHScript.setGender(gender)
                MHScript.setHeight(height)
                MHScript.setWeight(weight)
                MHScript.setAge(age)
                MHScript.setMuscle(muscle)
                MHScript.setBodyProportions(proportion)
                MHScript.setPose(AKIMBO01_BVH_PATH)

                heightCm = MHScript.getHeightCm()
                ageYears = MHScript.getAgeYears()
                weight_arr = MHScript.getWeightInKg()
                weightKg = sum(weight_arr) / len(weight_arr)
                waistCm = MHScript.getWaistCm()
                hipsCm = MHScript.getHipsCm()
                chestCm = MHScript.getChestCm()

                info_str = "{}-{}-{}-{}".format(int(heightCm), int(weightKg), int(ageYears), int(muscle * 100))

                file_path = os.path.join(model_dir, "{}_{}.pcd".format(count, info_str))

                MHScript.savePCD(file_path)
                # png_file_path = os.path.join(model_dir, "{}_{}.png".format(count, info_str))
                # MHScript.screenShot(png_file_path)

                log.message("end : " + str(datetime.now()))
                log.message("count/total = {} / {}".format(count, total))
                line = "{} {} {} {} {} {} {} {} {} {}\n".format(heightCm, weightKg, ageYears, gender, muscle, proportion, chestCm, waistCm, hipsCm, file_path)
                dataset_file.write(line)

                self.aProgressBar.setProgress(count * 1.0 / total)

                # if count >= 10:
                #     break

            dataset_file.close()

        @self.execButton.mhEvent
        def onClicked(event):
            #width = G.windowWidth;
            #height = G.windowHeight;

            global MHScript
            global scriptingView

            MHScript = Scripting()
            executeScript(str(scriptingView.scriptText.toPlainText()))

        box2 = self.addLeftWidget(gui.GroupBox('Fixed canvas size'))

        self.widthLabel = box2.addWidget(gui.TextView('Width'))
        self.widthEdit = box2.addWidget(gui.TextEdit(text='0'))
        self.heightLabel = box2.addWidget(gui.TextView('Height'))
        self.heightEdit = box2.addWidget(gui.TextEdit(text='0'))
        self.getButton = box2.addWidget(gui.Button('Get'))
        self.setButton = box2.addWidget(gui.Button('Set'))

        @self.getButton.mhEvent
        def onClicked(event):
            width = G.windowWidth;
            height = G.windowHeight;
            self.widthEdit.setText(str(width))
            self.heightEdit.setText(str(height))

        @self.setButton.mhEvent
        def onClicked(event):
            dlg = gui.Dialog()

            desiredWidth = self.widthEdit.getText()
            if(desiredWidth == None or not desiredWidth.isdigit()):
                dlg.prompt("Input error","Width and height must be valid integers","OK")
                return

            desiredHeight = self.heightEdit.getText()
            if(desiredHeight == None or not desiredHeight.isdigit()):
                dlg.prompt("Input error","Width and height must be valid integers","OK")
                return

            desiredWidth = int(desiredWidth)
            desiredHeight = int(desiredHeight)

            if(desiredHeight < 100 or desiredWidth < 100):
                dlg.prompt("Input error","Width and height must be at least 100 pixels each","OK")
                return

            # This is because we're excluding a passepartout when doing screenshots.
            desiredWidth = desiredWidth + 3
            desiredHeight = desiredHeight + 3

            qmainwin = G.app.mainwin
            central = qmainwin.centralWidget() 
            cWidth = central.frameSize().width()
            cHeight = central.frameSize().height()
            width = G.windowWidth;
            height = G.windowHeight;

            xdiff = desiredWidth - width;
            ydiff = desiredHeight - height;

            cWidth = cWidth + xdiff
            cHeight = cHeight + ydiff

            central.setFixedSize(cWidth,cHeight)
            qmainwin.adjustSize()

class Scripting():
    def __init__(self):
        self.human = gui3d.app.selectedHuman
        self.fileIncrement = 0;
        self.modelPath = mh.getPath('models')
        self.cam = G.app.modelCamera

        self.ruler = Ruler()

        if(not os.path.exists(self.modelPath)):
            os.makedirs(self.modelPath)

    def applyTarget(self,targetName,power):
        log.message("SCRIPT: applyTarget(" + targetName + ", " + str(power) + ")")
        self.human.setDetail(mh.getSysDataPath("targets/" + targetName + ".target"), power)
        self.human.applyAllTargets()
        mh.redraw()

    def saveModel(self,name,path = mh.getPath('models')):
        log.message("SCRIPT: saveModel(" + name + "," + path + ")")
        filename = os.path.join(path,name + ".mhm")
        self.human.save(filename,name)

    def loadModel(self,name,path = mh.getPath('models')):
        log.message("SCRIPT: loadModel(" + name + "," + path + ")")
        filename = os.path.join(path,name + ".mhm")
        self.human.load(filename, True)

    def saveObj(self, name, path = mh.getPath('exports')):
        log.message("SCRIPT: saveObj(" + name + "," + path + ")")
        filename = os.path.join(path, name + ".obj")

        cfg = ObjConfig()
        # cfg.unit = "m" # Not work
        cfg.scale = 0.1
        cfg.feetOnGround = True
        cfg.setHuman(self.human)

        exportObj(filename, cfg)

    def savePCD(self, filename):
        log.message("SCRIPT: savePCD(" + filename + ")")

        cfg = ObjConfig()
        # cfg.unit = "m" # Not work
        cfg.scale = 0.1
        cfg.feetOnGround = True
        cfg.setHuman(self.human)

        exportPCD(filename, cfg)

    def screenShot(self,fileName):
        log.message("SCRIPT: screenShot(" + fileName + ")")
        width = G.windowWidth;
        height = G.windowHeight;
        width = width - 3;
        height = height - 3;
        mh.grabScreen(1,1,width,height,fileName)

    def incrementingFilename(self,basename,suffix=".png",width=4):
        fn = basename;
        i = width - 1;
        self.fileIncrement += 1
        while(i > 0):       
            power = 10**i;
            if(self.fileIncrement < power):
                fn = fn + "0";
            i -= 1
        fn = fn + str(self.fileIncrement) + suffix
        return fn        

    def printDetailStack(self):
        log.message("SCRIPT: printDetailStack()")
        for target in self.human.targetsDetailStack.keys():
            print str(self.human.targetsDetailStack[target]) + "\t" + target

    def reset(self):
        gui3d.app.resetHuman()

    def loadBvh(self, filepath, convertFromZUp="auto"):
        bvh_file = bvh.load(filepath, convertFromZUp)
        anim = bvh_file.createAnimationTrack(self.human.getBaseSkeleton())
        if "root" in bvh_file.joints:
            posedata = anim.getAtFramePos(0, noBake=True)
            root_bone_idx = 0
            self.bvh_root_translation = posedata[root_bone_idx, :3, 3].copy()
        else:
            self.bvh_root_translation = np.asarray(3*[0.0], dtype=np.float32)
        self.bvh_bone_length = self.calculateBvhBoneLength(bvh_file)
        self.autoScaleAnim(anim)
        anim.license = None
        return anim

    def calculateBvhBoneLength(self, bvh_file):
        import numpy.linalg as la
        if COMPARE_BONE not in bvh_file.joints:
            raise RuntimeError('Failed to auto scale BVH file %s, it does not contain a joint for "%s"' % (
            bvh_file.name, COMPARE_BONE))

        bvh_joint = bvh_file.joints[COMPARE_BONE]
        joint_length = la.norm(bvh_joint.children[0].position - bvh_joint.position)
        return joint_length

    def autoScaleAnim(self, anim):
        """
        Auto scale BVH translations by comparing upper leg length to make the
        human stand on the ground plane, independent of body length.
        """
        import numpy.linalg as la
        bone = self.human.getBaseSkeleton().getBone(COMPARE_BONE)
        scale_factor = float(bone.length) / self.bvh_bone_length
        trans = scale_factor * self.bvh_root_translation
        log.message("Scaling animation %s with factor %s" % (anim.name, scale_factor))
        # It's possible to use anim.scale() as well, but by repeated scaling we accumulate error
        # It's easier to simply set the translation, as poses only have a translation on
        # root joint

        # Set pose root bone translation
        root_bone_idx = 0
        posedata = anim.getAtFramePos(0, noBake=True)
        posedata[root_bone_idx, :3, 3] = trans
        anim.resetBaked()

    def setPose(self, posePath):
        log.message("SCRIPT: setPose " + str(posePath))
        anim = self.loadBvh(posePath, convertFromZUp="auto")
        self.human.resetToRestPose()
        self.human.addAnimation(anim)
        self.human.setActiveAnimation(anim.name)
        self.human.setToFrame(0, update=False)
        self.human.setPosed(True)

    def setMuscle(self, value):
        log.message("SCRIPT: setMuscle(" + str(value) + ")")
        self.human.setMuscle(value)
        # mh.redraw()

    def setBodyProportions(self, value):
        log.message("SCRIPT: setBodyProportions(" + str(value) + ")")
        self.human.setBodyProportions(value)
        # mh.redraw()

    def setAge(self,age):
        log.message("SCRIPT: setAge(" + str(age) + ")")
        self.human.setAge(age)
        # mh.redraw()

    def setAgeYears(self, age):
        log.message("SCRIPT: setAgeYears(" + str(age) + ")")
        self.human.setAgeYears(age)
        # mh.redraw()

    def getAgeYears(self):
        log.message("SCRIPT: getAgeYears()")
        age = gui3d.app.selectedHuman.getAgeYears()
        return age

    def setWeight(self,weight):
        log.message("SCRIPT: setWeight(" + str(weight) + ")")
        self.human.setWeight(weight)
        # mh.redraw()

    def setGender(self, gender):
        log.message("SCRIPT: setGender(" + str(gender) + ")")
        self.human.setGender(gender)
        # mh.redraw()

    def setHeight(self, height):
        log.message("SCRIPT: setHeight(" + str(height) + ")")
        self.human.setHeight(height)
        # mh.redraw()

    def setMaterial(self, mhmat_filename):
        log.message("SCRIPT: setMaterial(" + mhmat_filename + ")")
        # The file must be realtive to the App Resources directory,
        # e.g.: 'data/skins/young_caucasian_female/young_caucasian_female.mhmat'
        mat = material.fromFile(mhmat_filename)
        self.human.material = mat

    def getHeightCm(self):
        value = gui3d.app.selectedHuman.getHeightCm()
        log.message("SCRIPT: getHeightCm(" + str(value) + ")")
        return value

    def getChestCm(self):
        return self.getMeasure('measure/measure-bust-circ-decr|incr')

    def getWaistCm(self):
        return self.getMeasure('measure/measure-waist-circ-decr|incr')

    def getHipsCm(self):
        return self.getMeasure('measure/measure-hips-circ-decr|incr')

    def getMeasure(self, name):
        return self.ruler.getMeasure(gui3d.app.selectedHuman, name, G.app.getSetting('units'))

    def getModelingParameters(self):
        log.message("SCRIPT: getModelingParameters()")
        modifierNamesList = sorted( self.human.modifierNames )
        print "Modifier names:"
        print "\n".join( modifierNamesList )

    def updateModelingParameter(self, parameterName, value):
        log.message("SCRIPT: updateModelingParameter(parameterName, value)")
        modifier = self.human.getModifier(parameterName)
        modifier.setValue(value)
        self.human.applyAllTargets()
        # mh.redraw()

    def updateModelingParameters(self, dictOfParameterNameAndValue):
        log.message("SCRIPT: updateModelingParameters("+str(dictOfParameterNameAndValue)+")")
        for key, value in dictOfParameterNameAndValue.iteritems():
            modifier = self.human.getModifier(key)
            modifier.setValue(value)
        self.human.applyAllTargets()
        # mh.redraw()

    def setHeadSquareness(self, squareness):
        log.message("SCRIPT: setHeadSquareness(" + str(squareness) + ")")
        modifier = self.human.getModifier('head/head-square')
        modifier.setValue(squareness)
        self.human.applyAllTargets()
        # mh.redraw()

    def setPositionX(self,xpos):
        log.message("SCRIPT: setPositionX(" + str(xpos) + ")")
        pos = self.human.getPosition()
        pos[0] = xpos
        self.human.setPosition(pos)
        # mh.redraw()

    def getPositionX(self):
        log.message("SCRIPT: getPositionX()")
        pos = self.human.getPosition()
        return pos[0]

    def modifyPositionX(self,xmod):
        log.message("SCRIPT: modifyPositionX(" + str(xmod) + ")")
        pos = self.human.getPosition()
        pos[0] = pos[0] + xmod
        self.human.setPosition(pos)
        # mh.redraw()

    def setPositionZ(self,zpos):
        log.message("SCRIPT: setPositionZ(" + str(zpos) + ")")
        pos = self.human.getPosition()
        pos[1] = zpos
        self.human.setPosition(pos)
        # mh.redraw()

    def getPositionZ(self):
        log.message("SCRIPT: getPositionZ()")
        pos = self.human.getPosition()
        return pos[1]

    def modifyPositionZ(self,zmod):
        log.message("SCRIPT: modifyPositionZ(" + str(zmod) + ")")
        pos = self.human.getPosition()
        pos[1] = pos[1] + zmod
        self.human.setPosition(pos)
        mh.redraw()

    def setPositionY(self,ypos):
        log.message("SCRIPT: setPositionY(" + str(ypos) + ")")
        pos = self.human.getPosition()
        pos[2] = ypos
        self.human.setPosition(pos)
        mh.redraw()

    def getPositionY(self):
        log.message("SCRIPT: getPositionY()")
        pos = self.human.getPosition()
        return pos[2]

    def modifyPositionY(self,ymod):
        log.message("SCRIPT: modifyPositionY(" + str(ymod) + ")")
        pos = self.human.getPosition()
        pos[2] = pos[2] + ymod
        self.human.setPosition(pos)
        mh.redraw()

    def setRotationX(self,xrot):
        log.message("SCRIPT: setRotationX(" + str(xrot) + ")")
        rot = self.human.getRotation()
        rot[0] = xrot
        self.human.setRotation(rot)
        mh.redraw()

    def getRotationX(self):
        log.message("SCRIPT: getRotationX()")
        rot = self.human.getRotation()
        return rot[0]

    def modifyRotationX(self,xmod):
        log.message("SCRIPT: modifyRotationX(" + str(xmod) + ")")
        rot = self.human.getRotation()
        rot[0] = rot[0] + xmod
        self.human.setRotation(rot)
        mh.redraw()

    def setRotationZ(self,zrot):
        log.message("SCRIPT: setRotationZ(" + str(zrot) + ")")
        rot = self.human.getRotation()
        rot[1] = zrot
        self.human.setRotation(rot)
        mh.redraw()

    def getRotationZ(self):
        log.message("SCRIPT: getRotationZ()")
        rot = self.human.getRotation()
        return rot[1]

    def modifyRotationZ(self,zmod):
        log.message("SCRIPT: modifyRotationZ(" + str(zmod) + ")")
        rot = self.human.getRotation()
        rot[1] = rot[1] + zmod
        self.human.setRotation(rot)
        mh.redraw()

    def setRotationY(self,yrot):
        log.message("SCRIPT: setRotationY(" + str(yrot) + ")")
        rot = self.human.getRotation()
        rot[2] = yrot
        self.human.setRotation(rot)
        mh.redraw()

    def getRotationY(self):
        log.message("SCRIPT: getRotationY()")
        rot = self.human.getRotation()
        return rot[2]

    def modifyRotationY(self,ymod):
        log.message("SCRIPT: modifyRotationY(" + str(ymod) + ")")
        rot = self.human.getRotation()
        rot[2] = rot[2] + ymod
        self.human.setRotation(rot)
        mh.redraw()

    def printCameraInfo(self):
        log.message("SCRIPT: printCameraInfo()")

        # TODO update to new camera
        print "eyeX:\t" + str(self.cam.eyeX)
        print "eyeY:\t" + str(self.cam.eyeY)
        print "eyeZ:\t" + str(self.cam.eyeZ)
        print "focusX:\t" + str(self.cam.focusX)
        print "focusY:\t" + str(self.cam.focusY)
        print "focusZ:\t" + str(self.cam.focusZ)
        print "upX:\t" + str(self.cam.upX)
        print "upY:\t" + str(self.cam.upY)
        print "upZ:\t" + str(self.cam.upZ)

    def printPositionInfo(self):
        log.message("SCRIPT: printPositionInfo()")

        pos = self.human.getPosition();

        print "posX:\t" + str(pos[0])
        print "posY:\t" + str(pos[2])
        print "posZ:\t" + str(pos[1])

    def printRotationInfo(self):
        log.message("SCRIPT: printRotationInfo()")

        rot = self.human.getRotation();

        print "rotX:\t" + str(rot[0])
        print "rotY:\t" + str(rot[2])
        print "rotZ:\t" + str(rot[1])

    def getWeightInKg(self):
        log.message("SCRIPT: getWeightInKg()")
        weights = calculateWeight(self.human)
        for w in weights:
            log.debug(w)
        return weights

    def getZoom(self):
        log.message("SCRIPT: getZoom()")
        return self.cam.zoomFactor

    def setZoom(self, zoom):
        log.message("SCRIPT: setZoom(" + str(zoom) + ")")
        self.cam.zoomFactor = zoom
        mh.redraw()

    def modifyZoom(self, zmod):
        log.message("SCRIPT: modifyZoom(" + str(zmod) + ")")
        self.cam.addZoom(zmod)
        mh.redraw()


class Ruler:
    def __init__(self):

        # these are tables of vertex indices for each body measurement of interest
        # TODO define in data file?

        self.Measures = {}
        self.Measures['measure/measure-neck-circ-decr|incr'] = [7514, 10358, 7631, 7496, 7488, 7489, 7474, 7475, 7531,
                                                                7537, 7543, 7549, 7555, 7561, 7743, 7722, 856, 1030,
                                                                1051, 850, 844, 838, 832, 826, 820, 756, 755, 770, 769,
                                                                777, 929, 3690, 804, 800, 808, 801, 799, 803, 7513,
                                                                7515, 7521, 7514]
        self.Measures['measure/measure-neck-height-decr|incr'] = [853, 854, 855, 856, 857, 858, 1496, 1491]

        self.Measures['measure/measure-upperarm-circ-decr|incr'] = [8383, 8393, 8392, 8391, 8390, 8394, 8395, 8399,
                                                                    10455, 10516, 8396, 8397, 8398, 8388, 8387, 8386,
                                                                    10431, 8385, 8384, 8389]
        self.Measures['measure/measure-upperarm-length-decr|incr'] = [8274, 10037]

        self.Measures['measure/measure-lowerarm-length-decr|incr'] = [10040, 10548]
        self.Measures['measure/measure-wrist-circ-decr|incr'] = [10208, 10211, 10212, 10216, 10471, 10533, 10213, 10214,
                                                                 10215, 10205, 10204, 10203, 10437, 10202, 10201, 10206,
                                                                 10200, 10210, 10209, 10208]

        self.Measures['measure/measure-frontchest-dist-decr|incr'] = [1437, 8125]
        self.Measures['measure/measure-bust-circ-decr|incr'] = [8439, 8455, 8462, 8446, 8478, 8494, 8557, 8510, 8526,
                                                                8542, 10720, 10601, 10603, 10602, 10612, 10611, 10610,
                                                                10613, 10604, 10605, 10606, 3942, 3941, 3940, 3950,
                                                                3947, 3948, 3949, 3938, 3939, 3937, 4065, 1870, 1854,
                                                                1838, 1885, 1822, 1806, 1774, 1790, 1783, 1767, 1799,
                                                                8471]
        self.Measures['measure/measure-underbust-circ-decr|incr'] = [10750, 10744, 10724, 10725, 10748, 10722, 10640,
                                                                     10642, 10641, 10651, 10650, 10649, 10652, 10643,
                                                                     10644, 10645, 10646, 10647, 10648, 3988, 3987,
                                                                     3986, 3985, 3984, 3983, 3982, 3992, 3989, 3990,
                                                                     3991, 3980, 3981, 3979, 4067, 4098, 4073, 4072,
                                                                     4094, 4100, 4082, 4088, 4088]
        self.Measures['measure/measure-waist-circ-decr|incr'] = [4121, 10760, 10757, 10777, 10776, 10779, 10780, 10778,
                                                                 10781, 10771, 10773, 10772, 10775, 10774, 10814, 10834,
                                                                 10816, 10817, 10818, 10819, 10820, 10821, 4181, 4180,
                                                                 4179, 4178, 4177, 4176, 4175, 4196, 4173, 4131, 4132,
                                                                 4129, 4130, 4128, 4138, 4135, 4137, 4136, 4133, 4134,
                                                                 4108, 4113, 4118, 4121]
        self.Measures['measure/measure-napetowaist-dist-decr|incr'] = [1491, 4181]
        self.Measures['measure/measure-waisttohip-dist-decr|incr'] = [4121, 4341]
        self.Measures['measure/measure-shoulder-dist-decr|incr'] = [7478, 8274]

        self.Measures['measure/measure-hips-circ-decr|incr'] = [4341, 10968, 10969, 10971, 10970, 10967, 10928, 10927,
                                                                10925, 10926, 10923, 10924, 10868, 10875, 10861, 10862,
                                                                4228, 4227, 4226, 4242, 4234, 4294, 4293, 4296, 4295,
                                                                4297, 4298, 4342, 4345, 4346, 4344, 4343, 4361, 4341]

        self.Measures['measure/measure-upperleg-height-decr|incr'] = [10970, 11230]
        self.Measures['measure/measure-thigh-circ-decr|incr'] = [11071, 11080, 11081, 11086, 11076, 11077, 11074, 11075,
                                                                 11072, 11073, 11069, 11070, 11087, 11085, 11084, 12994,
                                                                 11083, 11082, 11079, 11071]

        self.Measures['measure/measure-lowerleg-height-decr|incr'] = [11225, 12820]
        self.Measures['measure/measure-calf-circ-decr|incr'] = [11339, 11336, 11353, 11351, 11350, 13008, 11349, 11348,
                                                                11345, 11337, 11344, 11346, 11347, 11352, 11342, 11343,
                                                                11340, 11341, 11338, 11339]

        self.Measures['measure/measure-ankle-circ-decr|incr'] = [11460, 11464, 11458, 11459, 11419, 11418, 12958, 12965,
                                                                 12960, 12963, 12961, 12962, 12964, 12927, 13028, 12957,
                                                                 11463, 11461, 11457, 11460]
        self.Measures['measure/measure-knee-circ-decr|incr'] = [11223, 11230, 11232, 11233, 11238, 11228, 11229, 11226,
                                                                11227, 11224, 11225, 11221, 11222, 11239, 11237, 11236,
                                                                13002, 11235, 11234, 11223]

        self._validate()

    def _validate(self):
        """
        Verify currectness of ruler specification
        """
        names = []
        for n, v in self.Measures.items():
            if len(v) % 2 != 0:
                names.append(n)
        if len(names) > 0:
            raise RuntimeError(
                "One or more measurement rulers contain an uneven number of vertex indices. It's required that they are pairs indicating the begin and end point of every line to draw. Rulers with uneven index count: %s" % ", ".join(
                    names))

    def getMeasure(self, human, measurementname, mode):
        measure = 0
        vindex1 = self.Measures[measurementname][0]
        log.message("Coord length = " + str(len(human.meshData.coord)))
        log.message("fvert = " + str(len(human.meshData.fvert)))
        log.message("fnorm = " + str(len(human.meshData.fnorm)))
        log.message("measurementname" + str(measurementname))
        log.message("{}".format(human.meshData.coord[vindex1]))
        for vindex2 in self.Measures[measurementname]:
            log.message("vindex 1 = " + str(vindex1) + " vindex 2 = " + str(vindex2))
            vec = human.meshData.coord[vindex1] - human.meshData.coord[vindex2]
            measure += math.sqrt(vec.dot(vec))
            vindex1 = vindex2

        if mode == 'metric':
            return 10.0 * measure
        else:
            return 10.0 * measure * 0.393700787


MHScript = None

def executeScript(scriptSource):
    print scriptSource
    try:
        exec(scriptSource)
        dlg = gui.Dialog()
        dlg.prompt("Good job!","The script was executed without problems.","OK")
    except Exception as e:
        log.error(e, exc_info = True)
        dlg = gui.Dialog()
        dlg.prompt("Crappy script","The script produced an exception: " + format(str(e)),"OK")

scriptingView = None

def load(app):

    global scriptingView

    category = app.getCategory('Utilities')
    scriptingView = ScriptingView(category)
    executeView = ScriptingExecuteTab(category)
    taskview = category.addTask(scriptingView)
    taskview1 = category.addTask(executeView)

def unload(app):
    pass


