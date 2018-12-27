clear
clc 
d1 = textread('ax.txt');
d2 = textread('ay.txt');
d3 = textread('az.txt');
d4 = textread('wx.txt');
d5 = textread('wy.txt');
d6 = textread('wz.txt');
d7 = textread('roll.txt');
d8 = textread('pitch.txt');
d9 = textread('yaw.txt');
d10 = textread('time.txt');
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      
n = length(d1);
ax = d1(1:n-1);
ay = d2(1:n-1);
az = d3(1:n-1);
wx = d4(1:n-1);
wy = d5(1:n-1);
wz = d6(1:n-1);
roll = d7(1:n-1);
pitch = d8(1:n-1);
yaw = d9(1:n-1);
t = d10(1:n-1);
n=n-1;
plot(t,ax);

% for i=1:n-1
%     sum = ax(i) * 0.005;
% end