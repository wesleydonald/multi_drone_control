%The purpose is to initializing dynamic models of robot nodes and the
%payload
clear all

load('pp.mat');

n0 = 5;
n = n0 + 2; 
pp0_ = [];

breaktime = 5;
psi = 2*pi*(rand(n0,1) - 0.5);
theta = pi/4 + pi/4*rand(n0,1);
tt = pi/6 + pi*2/3*rand(n0,1);
l = 2.02;
pp(:,n0+1) = [0;0;-8.5858]; 

mass = pp(:,n0+1);
r = 2.02*ones(10,1) + 0*2.02*(rand(10,1) - 0.5); 
rr = 2.02;
h0 = -11;

for i = 1:n0
    pp(:,i) = pp(:,n0+1) + [r(i)*cos(theta(i))*cos(psi(i)); r(i)*cos(theta(i))*sin(psi(i)); -r(i)*sin(theta(i))];
    r0 = (pp(:,i) - mass)/norm(pp(:,i)-mass,2); 
    pp(:,i) = pp(:,n0+1) + r(i)/1*r0;
    l2 = norm(pp(:,i)-mass,2);
    h1 = abs(h0 - pp(3,i));
    h2 = abs(pp(3,i) - mass(3));
    l1 = h1/h2*l2;
    pp0_(:,i) = pp(:,i) + l1*r0;
end

p1 = [mass(1);mass(2);h0];
pp0_ = [p1, pp0_, mass];
pp_ = reshape(pp0_([1,2],:),[2*n,1]);
pp_ = [pp_; h0; mass(3)];
m = [2,1,1,1,1,1,5]';

g = 9.8;
k = 20;
c = 10;
v0 = [zeros(1,2+2*(n0+2))]'; 

p00 = [10,0,h0];
R = 5;
t1 = 20;
t2 = 0;
step = 0.01;
time1 = 0:0.01:t1;
pd1 = [time1', kron(p00,ones(size(time1,2),1))];
time2 = t1+step:0.01:t2;
theta = 2*pi/(length(time2)):2*pi/(length(time2)):2*pi;
theta = theta';
pd2 = p00 + R*[cos(theta),sin(theta),zeros(size(theta,1),1)];
pd2 = [time2',pd2];
pd = pd1;
pd_hor = pd(:,1:3); 
pd_ver = pd(:,[1,4]);

M = diag(m);
W = ones(n,n);
for i = 1:n
    for j = 1:n
        if i == j
            W(i,j) = 0;
        else if (i==1 && j==n)|| (i==n && j==1)
                W(i,j) = 0;
        end
        end
    end
end

W_b = W;
W_b([5,6],:) = zeros(2,n);
W_b(:,[5,6]) = zeros(n,2);
M_b = M;
M_b([5,6],:) = zeros(2,n);
M_b(:,[5,6]) = zeros(n,2);

D = eye(n);
for i = 1:n
    D(i,i) = sum(W(i,:));
end
L = D-W;

distance = inf;
D_ = distance*W(1:n-1,1:n-1);
delta_l = rand(n,n);

L0 = ones(n,n);
delta_l = rand(n,n);
for i = 1:n
    for j = 1:n
        if i==1 || j==1 || i==j
            L0(i,j) = 0;
        else if i == n || j == n
              L0(i,j) = norm(pp0_(:,i)-pp0_(:,j),2);
        else
              L0(i,j) = 2.02; 
        end
        end
    end
end

K0 = 2*W;
K = K0;
delta_k = rand(n,n);
for i = 1:n
    for j = 1:n
        if (i==n || j==n) && (i~=1) && (j~=1) && (i~=n || j~=n)
            K(i,j) = 5*K0(i,j) + 0*0.5*(delta_k(i,j) + delta_k(j,i));
        else
            K(i,j) = 5*K0(i,j); 
        end
    end
end

am = [0 inf*9*ones(1,n0) + 0*3.6*(rand(1,n0)-0.5) 0];
open('decentralized.slx');
sim('decentralized.slx');
caldata;
demonstration;