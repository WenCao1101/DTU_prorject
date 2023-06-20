import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from array import array
# Set up the grid of points on the surface of the hemisphere
#n_points = 10
#u = np.linspace(0, 2 * np.pi, n_points)
#v = np.linspace(0, np.pi / 2, n_points)
#theta, phi = np.meshgrid(u, v)

#brief partition the unit hemisphere into smaller patches with equal area
#Ref: A general rule for disk and hemisphere partition into equal-area cells
r_i1=2*(np.sin(0.25*np.pi))
theta_i1=0.5*np.pi
k_i1=19
theta=0.01 
k=0
r=0
number_ring=int(0)
section_center=np.zeros((k_i1,2))
section_center[0,:]=np.array([0,0])
rad_ang=float(360/(2*np.pi))
accumulated_ZenithAngle=[]
accumulated_azimuthAngle=[[]]
accumulated_ZenithAngle_repeat=[[]]
while theta>0:
   accumulated_ZenithAngle.append(theta_i1)
   theta=theta_i1-2*np.sin(0.5*theta_i1)*np.sqrt(np.pi/k_i1)
   r=2*np.sin(0.5*theta)
   #compute a new r and theta
   k=int(k_i1*(r/r_i1)*(r/float(r_i1)))
   theta=2*np.arcsin(0.5*r)
   NumOfDirectionInSector=int(k_i1-k)
   aziInterval=2*np.pi/float(NumOfDirectionInSector)
   azi_each_sector=[]
   accumulated_ZenithAngle_temp=[]
   for i in range(0, NumOfDirectionInSector, 1):
         azimutStart = float(i * aziInterval)
         azimuEnd = float((i + 1)*aziInterval)
         solidAngle =np.pi * (r_i1*r_i1 - r * r)/float(NumOfDirectionInSector)
         azi_each_sector.append(azimuEnd)
         section_center[k_i1-i-1,:]=np.array([0.5*(azimutStart+azimuEnd),0.5*(theta_i1+theta)])
         accumulated_ZenithAngle_temp.append(theta_i1)
   accumulated_azimuthAngle.append(azi_each_sector)
   accumulated_ZenithAngle_repeat.append(accumulated_ZenithAngle_temp)
   k_i1 = k
   r_i1 = r
   theta_i1 = theta

#theta, phi = np.meshgrid(accumulated_azimuthAngle, accumulated_ZenithAngle)   

#accumulated_ZenithAngle = [ele for ele in accumulated_ZenithAngle if ele != []]   
accumulated_azimuthAngle = [ele for ele in accumulated_azimuthAngle if ele != []] 
accumulated_ZenithAngle_repeat = [ele for ele in accumulated_ZenithAngle_repeat if ele != []] 


phi_array = array("f", accumulated_ZenithAngle)
theta_array = array("f", accumulated_azimuthAngle[0])
theta_1, phi_1 = np.meshgrid(theta_array, phi_array)
# Convert radians to  degrees
theta_ang = np.degrees(theta_array)
phi_ang = np.degrees(phi_array)
section_center_ang=np.degrees(section_center)

# Convert spherical coordinates to Cartesian coordinates
x = np.sin(phi_1) * np.cos(theta_1)
y = np.sin(phi_1) * np.sin(theta_1)
z = np.cos(phi_1)


fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(x, y, z, s=60)
ax.set_aspect('equal')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
plt.show()