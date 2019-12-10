import matplotlib.pyplot as plt
import numpy as np
import sys

def plot_contact_forces(name,datafile):
    data = np.loadtxt(datafile)
    print(name)
    print(np.shape(data))
    plt.figure()
    plt.subplot(3,1,1)
    plt.title(name + ' Contact Forces During Cylinder Grip')
    plt.plot(data[:,0])
    plt.ylabel('X (Grip Direction) (N)')
    plt.subplot(3,1,2)
    plt.plot(data[:,1])
    plt.ylabel('Y (N)')
    plt.subplot(3,1,3)
    plt.plot(data[:,2])
    plt.xlabel('Sample Number')
    plt.ylabel('Z (N)')
    plt.show()
	
		
if __name__ == '__main__':
    plot_contact_forces(sys.argv[1],sys.argv[2]) # Name of plot and file passed via CLAs