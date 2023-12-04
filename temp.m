clear all;
close all;
lat_error_1 = 0.3;
lat_error_2 = 0.6;
L1 = 0:0.05:1;
l = L1*lat_error_1 +(1-L1) * lat_error_2;

plot(L1,l);
xlabel("L1");
ylabel("lat\_error\_cost")
