import vtk
import math
import numpy as np
import pyvista as pv
from vtkmodules.vtkCommonColor import vtkNamedColors
from vtkmodules.vtkFiltersSources import vtkSphereSource
from vtkmodules.vtkIOImage import vtkPNGWriter
from vtkmodules.vtkRenderingCore import (
    vtkActor,
    vtkCamera,
    vtkPolyDataMapper,
    vtkRenderWindow,
    vtkRenderWindowInteractor,
    vtkRenderer, vtkWindowToImageFilter,
)
import open3d as o3d


def trans_to_matrix(trans):
    """ Convert a numpy.ndarray to a vtk.vtkMatrix4x4 """
    matrix = vtk.vtkMatrix4x4()
    for i in range(trans.shape[0]):
        for j in range(trans.shape[1]):
            matrix.SetElement(i, j, trans[i, j])
    return matrix

def main():

    w = 1080
    h = 1349

    intrinsic = np.array([[543.2,   0.0, 1003.7],
                          [  0.0, 667.3,  965.2],
                          [  0.0,   0.0,    1.0]])

    extrinsic = np.array([[  0.8433,  0.1371, -0.5197, -122.7617],
                          [ -0.2888,  0.9311, -0.2230,  -72.0352],
                          [ -0.4533, -0.3381, -0.8247,  356.9042],
                          [   0.0,    0.0,     0.0,       1.0000]])



    reader = vtk.vtkPNGReader()
    reader.SetFileName("C:\d\HandEyeARVisualization\PythonVisualization\Brain_case1.png")
    reader.Update()

    # create camera
    camera = vtkCamera()

    #
    # intrinsics
    #

    cx = intrinsic[0, 2]
    cy = intrinsic[1, 2]
    f = intrinsic[0, 0]

    # convert the principal point to window center (normalized coordinate system) and set it
    wcx = -2 * (cx - float(w) / 2) / w
    wcy = 2 * (cy - float(h) / 2) / h
    camera.SetWindowCenter(wcx, wcy)

    # convert the focal length to view angle and set it
    view_angle = 180 / math.pi * (2.0 * math.atan2(h / 2.0, f))
    camera.SetViewAngle(view_angle)

    #
    # extrinsics
    #

    # apply the transform to scene objects
    camera.SetModelTransformMatrix(trans_to_matrix(extrinsic))

    # the camera can stay at the origin because we are transforming the scene objects
    camera.SetPosition(0, 0, 0)

    # look in the +Z direction of the camera coordinate system
    camera.SetFocalPoint(0, 0, 1)

    # the camera Y axis points down
    camera.SetViewUp(0, -1, 0)

    # Create a renderer, render window, and interactor
    renderer = vtkRenderer()
    renderer.SetActiveCamera(camera)
    renderer.ResetCameraClippingRange()

    renderWindow = vtkRenderWindow()
    renderWindow.AddRenderer(renderer)
    renderWindow.SetWindowName('Camera')

    renderWindowInteractor = vtkRenderWindowInteractor()
    renderWindowInteractor.SetRenderWindow(renderWindow)

    # set background to image
    texture = vtk.vtkTexture()
    texture.SetInputConnection(reader.GetOutputPort())
    renderer.SetTexturedBackground(True)
    renderer.SetBackgroundTexture(texture)

    colors = vtkNamedColors()

    # Create a sphere
    filename = "Brain_mesh_crop.stl"
    reader = vtk.vtkSTLReader()
    print(reader.GetLocator())
    reader.SetFileName(filename)

    # Create a mapper and actor
    mapper = vtk.vtkPolyDataMapper()
    if vtk.VTK_MAJOR_VERSION <= 5:
        mapper.SetInput(reader.GetOutput())
    else:
        mapper.SetInputConnection(reader.GetOutputPort())

    actor = vtk.vtkActor()
    actor.SetMapper(mapper)

    # Render and interact
    renderer.AddActor(actor)
    renderWindowInteractor.Initialize()
    renderWindow.Render()
    renderWindowInteractor.UpdateSize(640, 480)

    # Save Image to File
    im = vtkWindowToImageFilter()
    writer = vtkPNGWriter()
    im.SetInput(renderWindow)
    im.Update()
    writer.SetInputConnection(im.GetOutputPort())
    writer.SetFileName("BrainOut.png")
    writer.Write()

    # renderWindowInteractor.Start()

if __name__ == '__main__':
    main()
