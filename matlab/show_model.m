clc;
clear;
close all;
addpath(genpath('../'));

figure;
biorob_path = "../biorob_bounding_fwd/model/biorob.urdf";
biorob = importrobot(biorob_path);
show(biorob);
grid off;
% smimport(biorob_path);

figure;
solo_path = "../solo_bounding_fwd/model/solo.urdf";
solo = importrobot(solo_path);
show(solo);
grid off;
% smimport(solo_path);
