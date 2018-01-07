function [ mprim, res, num_angles] = loadmprim(filename)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
NUMOFDIRS = 8;
NUMOFPRIMS = 5;
NUMOFINTERSTATES = 10;
NUMOFDIM = 3;

fileID = fopen(filename, 'r');

 % resolution
fscanf(fileID,'%s',1); 
res = fscanf(fileID,'%f',1);
 % number of angles
fscanf(fileID,'%s',1);
num_angles = fscanf(fileID,'%f',1);
% number of motion primitives
fscanf(fileID,'%s',1);
num_prims = fscanf(fileID,'%f',1);

%% Motion Primitives
mprim = zeros(NUMOFDIRS,NUMOFPRIMS,NUMOFINTERSTATES,NUMOFDIM);

% populate primitives
for i = 1 : NUMOFDIRS
    for j = 1 : NUMOFPRIMS
        fscanf(fileID,'%s',2);  % skip line
        for k = 1 : NUMOFINTERSTATES
%             poses = textscan(fileID,'%f%f%f','HeaderLines',2,'CollectOutput',1);
            mprim(i,j,k,:) = fscanf(fileID,'%f',3);
        end
    end
end

end

