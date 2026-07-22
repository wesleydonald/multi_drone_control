%The purpose is to calculate the supposed position and speed of the flying robots

[m n] = size(W.data);
m = n0+2;
t1 = size(p.time,1);
t = t1;

% mass = line('Color',[0.67 0 1],'Marker','.','MarkerSize',20,'EraseMode','xor');
axis([-5 5 -5 5]);

syms sx sy sz

d = r;
point_ = [];
velo_ = [];
point1_ = [];
velo1_ = [];

for i = 1:size(tout,1)
    position = p.data(:,i); 
    V = v.data(:,i);
    pp = zeros(3,m);
    vv = zeros(3,m);
    for j = 1:m
        pp(:,j) = [position(2*j-1);position(2*j);position(end-1)];
        vv(j) = norm([V(2*j-1);V(2*j);V(end-1)],2);
        if j == m
        pp(:,j) = [position(2*j-1);position(2*j);position(end)]; 
        vv(j) = norm([V(2*j-1);V(2*j);V(end)],2);
        mass = pp(:,j);
        end
        if j == 1
            a = 1;
        else if j == m 
                point = [];
                velo = [];
                point1 = [];
                velo1 = [];
            for k = 2:m
                vpoint = [pp(1,k);pp(2,k);pp(3,k)]; 
                if k ~= m
                    ll = (vpoint-mass)/norm(vpoint-mass,2); 
                    if all(W.data(k,:,i) == 0)
                        uav_vector = point_([3*k-2,3*k-1,3*k],i-1);
                    else
                        uav_vector = mass + d(k-1)*ll; 
                    end
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