%The purpose is to calculate the supposed position and speed of the flying robots
[m n] = size(W.data);
t1 = size(p.time,1);
t = t1;
axis([-5 5 -5 5]);
syms sx sy sz
d = r;
point_ = [];
velo_ = [];
point1_ = [];
velo1_ = [];
a = squeeze(p.data);
a = a';

for i = 1:10001
    position = p.data(:,i); 
    V = v.data(:,i);
    pp = zeros(3,m);
    vv = zeros(3,m);
    for j = 1:m
        % Position of the formation center and flying robots 
        pp(:,j) = [position(2*j-1);position(2*j);position(end-1)]; 
        vv(j) = norm([V(2*j-1);V(2*j);V(end-1)],2);
        if j == m
        pp(:,j) = [position(2*j-1);position(2*j);position(end)]; 
        % Position of the Payload
        vv(j) = norm([V(2*j-1);V(2*j);V(end)],2);
        mass = pp(:,j);
        end
        if j == 1 
            a = 1;
        else if j == m % Greed dot in the simulation is the payload
                point = [];
                velo = [];
                point1 = [];
                velo1 = [];
            for k = 2:m
                vpoint = [pp(1,k);pp(2,k);pp(3,k)]; 
                if k ~= m
                    ll = (vpoint-mass)/norm(vpoint-mass,2); 
                    uav_vector = mass + d(k-1)*ll; 
                    point = [point;uav_vector];
                    velo = [velo;vv(k)*d(k-1)/norm(vpoint-mass,2)];
                    point1 = [point1;vpoint];
                    velo1 = [velo1;vv(k)];
                else
                    point = [vpoint;point];
                    velo = [vv(k);velo];
                    point1 = [vpoint;point1];
                    velo1 = [vv(k);velo1];
                end
            end
        end
        end
    end
                point_ = [point_, point];
                velo_ = [velo_,velo];
                point1_ = [point1_, point1];
                velo1_ = [velo1_,velo1];
end
position = point_;
velo_norm_ = velo_;
position1 = point1_;
velo_norm1_ = velo1_;