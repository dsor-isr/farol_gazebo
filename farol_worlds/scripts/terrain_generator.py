#!/usr/bin/python3

"""
Convert .mat files with that into the corresponding .npy and .stl files
@author: Marcelo Fialho Jacinto
@email: marcelo.jacinto@tecnico.ulisboa.pt
@date: 06/03/2021
"""

import numpy as np
import matplotlib.tri as mtri
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from opensimplex import OpenSimplex
from stl import mesh, Mode
from scipy.io import loadmat

import sys


def main():
    
    try:
        # Load the map into memory
        map = loadmat(sys.argv[1])

        x_mesh = map['X']
        y_mesh = map['Y']
        z_mesh = map['Z']
    except:
        print("ðŸ¥´The file passed as argument does not contain matrices with names 'X', 'Y' or 'Z'.\n Be carefull and use upper-case letters!")
        exit(1)
    
    # Name of the output files
    numpy_file_name = sys.argv[1].replace('.mat', '.npy')
    stl_file_name = sys.argv[1].replace('.mat', '.stl')

    # Generate the corresponding numpy file
    np.save(numpy_file_name, (x_mesh, y_mesh, z_mesh))
    print("ðŸº Generated numpy file successfully!")

    try:

        # Flatten the data for interpolation
        output = np.zeros(shape=(x_mesh.size, 3))
        output[:, 0] = x_mesh.flatten()
        output[:, 1] = y_mesh.flatten()
        output[:, 2] = z_mesh.flatten()
    
        # Triangulation of the interpolated data
        tri = mtri.Triangulation(output[:, 0], output[:, 1])
        print("ðŸº Generated triangulation of the data!")

        plt.imshow(z_mesh)
        plt.colorbar()
        plt.show()

        # Create the mesh object
        seabed_mesh = mesh.Mesh(np.zeros(tri.triangles.shape[0], dtype=mesh.Mesh.dtype))
        print("ðŸº Generated a 3D mesh!")

        # Set the vectors
        for i, f in enumerate(tri.triangles):
            for j in range(3):
                seabed_mesh.vectors[i][j] = output[f[j]]

        # Store the seabed as a STL file
        seabed_mesh.save(stl_file_name)
        print("ðŸº Generated the stl file!")
    
    except Exception as e:
        print('âŒ')
        print(e)
    
    
if __name__ == "__main__":

    # Validate the arguments of the program
    if len(sys.argv) != 2 or sys.argv[1] == '-h' or sys.argv[1] == '-help' or sys.argv[1] == 'h' or sys.argv[1] == 'help':
        print("Execution mode: python <program_name> <file.mat>\n")
        print("The file <file.mat> should contain 3 matrices of the same size named 'X', 'Y' and 'Z'")
        print("NOTE: Use upper case letters in the name of the variables!")
        exit(1)

    main()