close all; 
%clear all; clc;
%% ------------------------------------------------------------------------
%% Virtual plane parameters
% X = Y = -1000:1000, taking 1 step, the total points = 2000 in both X & Y
x_total = 2000;
x_min = -x_total/2;
x_max = x_total/2;
%% ------------------------------------------------------------------------
%% Image plan parameters
image_width = 12.7; image_height = 12.7;
raw_pixel = 480; column_pixel = 640;
% Resolution = physical size / pixel dimensions
resolution_raw = image_width/raw_pixel;
resolution_column = image_height/column_pixel;

%% ------------------------------------------------------------------------                          
%%                    Create The Virtual Plane
xyz_plane = zeros(x_total,x_total,6); %(x,y,z,H,S,V)
i = 1; j = 1;
for x = x_min:(x_max-1)
    for y = x_min:(x_max-1)
        xyz_plane(i,j,1) = x;
        xyz_plane(i,j,2) = y;
        xyz_plane(i,j,3) = 100*sin(x/150);
        xyz_plane(i,j,4) = 0.5*((x/1000)+1);
        xyz_plane(i,j,5) = 1;
        xyz_plane(i,j,6) = 0.5*((-y/1000)+1);
        j = j+1;
    end
    i = i+1; j=1;
end
% Extract xyz points and hsv color space points:
xyzPoints = xyz_plane(:,:,1:3); % Extract the xyz points of the virtual plane.
HSV = xyz_plane(:,:,4:6);       % Extract the hsv color space
RGB = hsv2rgb(HSV);             % Convert HSV into RGB color space

% % Plot the virtual plane using RGB color space
ptCloud_RGB = pointCloud(xyzPoints, 'Color', RGB);
figure,pcshow(ptCloud_RGB), title('Virtual Plane in RGB'), xlabel('X'), ylabel('Y'),zlabel('Z');
set(gca,'linewidth', 1.5,'fontsize',14,'fontname','Times New Roman','Color','none')

%% Part I (Output type 1 - Textured point cloud)
tic
% The homogeneous transformation:
Q_Robj1_Rcam = [1 0 0 0; 0 1 0 0; 0 0 1 1000; 0 0 0 1];
w = ones(x_total, x_total);   % w = 1 as this point is in real 3D space (full sccale).
xyzw_TPC = cat(3, xyzPoints, w);
xyz_TPC = zeros(x_total,x_total,4); % Textured point cloud (TPC)
for i = 1:x_total
    for j = 1:x_total
        xyz_TPC(i,j,:) = Q_Robj1_Rcam * squeeze(xyzw_TPC(i,j,:));
    end
end

XYZ_TPC = xyz_TPC(:,:,1:3); % textured 3D point cloud

ptCloud_TPC = pointCloud(XYZ_TPC, 'Color', RGB);
figure,pcshow(ptCloud_TPC), title('Output Type 1 - Textured Point Cloud'), xlabel('X'), ylabel('Y'),zlabel('Z');
set(gca,'linewidth', 1.5,'fontsize',14,'fontname','Times New Roman','Color','none')

toc

%% Part II (Output type 2 - Cartesian depth map (CDM)
tic
f = 5; % Focal lenth
P = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 1/f 0]; % Perspective Projection Matrix 
%%
w = ones(x_total, x_total);   % w = 1 as this point is in real 3D space (full sccale).
xyzw = cat(3, xyzPoints, w);
xyz_DM = zeros(x_total,x_total,4); % Depth Map points (DM)
for i = 1:x_total
    for j = 1:x_total
        xyz_DM(i,j,:) = P * Q_Robj1_Rcam * squeeze(xyzw(i,j,:));
    end
end
%%                          Perspective Projection Equations
%                              [x';y';z';w'] = [x; y; z; z/f]
% Retrieve the perspective projection equations by deviding x',y',z' by the scaling factor (w'):
%                        [x';y';z'] = [x/w'; y/w'; z/w'] = [f*x/z; f*y/z; f']
xyz_DM(:,:,1:2) = xyz_DM(:,:,1:2)./xyz_DM(:,:,4);
%xyz_camera(:,:,3) = f;
%%                           Pixel Coordinates Calculation
% Calculate the pixel coordinates of every projection over the pixel map by considering the image
% plane physical dimension, the number of pixels per row and colomn in the
% output image.
xyz_DM(:,:,1) = round(xyz_DM(:,:,1)./resolution_column)+(column_pixel/2);
xyz_DM(:,:,2) = round(xyz_DM(:,:,2)./resolution_raw)+(raw_pixel/2);

%% Mapping the depth to the equivalant coordinations of the image plane
% Initializing the background of the output image map with black
CDM = zeros(column_pixel,raw_pixel); % Cartesian depth map (CDM)
% Retrieve the transformed coordination in image plane range: 
% - Columns: between 0 and 641
% - Raws: between 0 and 480
for i = 1:x_total
    for j = 1:x_total
        if (xyz_DM(i,j,1)>0) && (xyz_DM(i,j,2)>0) &&...
                (xyz_DM(i,j,1) <= column_pixel) && (xyz_DM(i,j,2) <= raw_pixel)
            if CDM(xyz_DM(i,j,1), xyz_DM(i,j,2)) == 0
                CDM(xyz_DM(i,j,1), xyz_DM(i,j,2)) = xyz_DM(i,j,3);
            end
        end
    end
end
CDM = permute(CDM,[2 1 3]);

CDM = CDM/max(CDM(:));
figure
imshow(CDM,[min(CDM(:)) max(CDM(:))])
title('Output Type 2 - Cartesian Depth Map with P');
toc
imwrite(CDM,'Output_Type_2_Cartesian_Depth_Map.jpg')

%% Part III (Output type 3 - Radial depth map)
tic
% Transformation coordinates:
% C a r t e s i a n   ( x , y , z )   →   S p h e r i c a l   ( r , θ , ϕ )
% r = sqrt(x.^2 + y.^2 + z.^2)
% θ = atan(y/x)
% ϕ = atan(sqrt(x.^2 + y.^2))./z)
r = sqrt((xyz_DM(:,:,1)).^2 + (xyz_DM(:,:,2)).^2 + (xyz_DM(:,:,3)).^2);
%theta = atan(xyz_DM(:,:,2)./xyz_DM(:,:,1));
%phi = atan(sqrt((xyz_DM(:,:,1)).^2 + (xyz_DM(:,:,2)).^2)./xyz_DM(:,:,3));

%[X,Y,Z]=sph2cart(theta,phi,r);
%surf(r);

RDM = zeros(column_pixel,raw_pixel); % Radial depth map (RDM)
for i = 1:x_total
    for j = 1:x_total
        if (xyz_DM(i,j,1)>0) && (xyz_DM(i,j,2)>0) &&...
                (xyz_DM(i,j,1) <= column_pixel) && (xyz_DM(i,j,2) <= raw_pixel)
            if (RDM(xyz_DM(i,j,1), xyz_DM(i,j,2)) == 0)
                RDM(xyz_DM(i,j,1), xyz_DM(i,j,2)) = r(i,j);
            end
        end
    end
end

RDM = permute(RDM,[2 1 3]);
RDM = RDM/max(RDM(:));
figure
imshow(RDM,[min(RDM(:)) max(RDM(:))])
title('Output Type 3 - Radial Depth Map with P');
toc
imwrite(RDM,'Output_Type_3_Radial_Depth_Map.jpg')



%% This is just to try the output of radial depth map when calculating the 
% radial depth based on the 3D points of tectured point cloud only.

r1 = sqrt((XYZ_TPC(:,:,1)).^2 + (XYZ_TPC(:,:,2)).^2 + (XYZ_TPC(:,:,3)).^2);


RDM1 = zeros(column_pixel,raw_pixel,3); % Radial depth map (RDM)
for i = 1:x_total
    for j = 1:x_total
        if (xyz_DM(i,j,1)>0) && (xyz_DM(i,j,2)>0) &&...
                (xyz_DM(i,j,1) <= column_pixel) && (xyz_DM(i,j,2) <= raw_pixel)
            if (RDM1(xyz_DM(i,j,1), xyz_DM(i,j,2),:) == 0)
                RDM1(xyz_DM(i,j,1), xyz_DM(i,j,2),:) = r1(i,j);
            end
        end
    end
end

RDM1 = permute(RDM1,[2 1 3]);
RDM1 = RDM1/max(RDM1(:));
figure
imshow(RDM1,[min(RDM1(:)) max(RDM1(:))])
title('Output Type 3 - Radial Depth Map without P');
%imwrite(RDM1,'Output_Type_3_Radial_Depth_Map1.jpg')
