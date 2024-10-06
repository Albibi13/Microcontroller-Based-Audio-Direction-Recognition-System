clear all; close all; clc;

N=200;
b_rate=115200;
s=serialport("COM7",b_rate);

tic
num=read(s,N,"uint8");
toc
t=linspace(1,toc,N/2);

left=zeros(1,N/2);
right=zeros(1,N/2);

for i=1:N
    if rem(i,2)==1
        right((i+1)/2)=num(i);
    else
        left(i/2)=num(i);
    end
end

plot(t,left)
hold on
plot(t,right)
grid on, legend("left","right")