# Machine Vision: RGB-D-RANGE-SENSOR-SIMULATOR
This is a simulation for RGB-D Range Sensor using Matlab
# Part 1: Textured point cloud
The virtual plane and the corresponding HSV color space were created as shwon in the figure belwo:
![image](https://user-images.githubusercontent.com/89004966/162563729-50ced937-f292-4b3c-839d-490e2a37145c.png)

More information provided in the report.

The generated 3D point cloud is distributed in 3D model corresponding to actual shape of virtual plan as shown below:
![image](https://user-images.githubusercontent.com/89004966/162563734-cd9b9432-2276-4578-a7d0-2c5dd5135f3d.png)


# Part 2: Cartesian depth map
The generated depth map image represents the front view of our virtual scene as shown in figure below. So, we can see the crests and troughs of the scene according to the gray level variation. The crest is darker corresponding to decreasing in depth where trough is lighter gray corresponding to increasing in depth.
![Output_Type_2_Cartesian_Depth_Map](https://user-images.githubusercontent.com/89004966/162563824-9cbecc77-9eb5-4e8b-b6fa-bbe5cf070ec3.jpg)

# Part 3: Radial depth map
The generated radial depth map image represents the front view of our virtual scene as shown in the figure below.

![Output_Type_3_Radial_Depth_Map](https://user-images.githubusercontent.com/89004966/162563919-f5ee7b6a-662f-4fb8-a6e8-8621fdc4a9a3.jpg)


For both cartesian depth map and radial depth map, the generated maps contain gray tones that vary in accordance with the measured cartesian and radial depth, where dark gray corresponds to surfaces closer to the range sensor (crest of sinusoidal surface) and light gray corresponds to surfaces farther away from the sensor (trough of sinusoidal surface). However, there are variations in the gray shading between the Cartesian and radial representations. 
In radial depth map, the gray tones distributed over the surface according to the depth. So, the gray level distribution along the vertical axis looks equal, but inversely proportional to the depth when moving horizontally – resulted with gray level changes (the shorter depth, the darker gray and the longer depth, the lighter gray). This equal distribution along vertical axis and different in the horizontal axis forms a smooth vertical stripes seen clearly when zooming in over the image. This is expected since the cartesian depth was estimated based on distance measured parallel to the sensor’s optical axis (Zcam) with respect to the depth reference plane whose surface is parallel and perfectly aligned with the origin of the sensor’s image plane, that is Zcam = 0.
In radial depth map, the gray tones distributed over the surface according to the radial depth. So, the distribution along the vertical and horizontal axes looks not equal, but still inversely proportional to the depth when moving horizontally (the shorter depth, the darker gray and the longer depth, the lighter gray). This gray level changes in the vertical and horizontal axes forms a smooth circular stripes seen clearly when zooming in over the image. This is also expected since the radial depth estimated based on the measured distance from the projection center that changes according to the change in azimuth and elevation angles. Also, there is a difference in gray intensity values, the radial depth map has higher values compared to cartesian depth map, so the radial depth map looks darker than cartesian depth map.
