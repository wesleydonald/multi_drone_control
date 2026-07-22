% Visualization results
close all
figure(1)
xlim([-1, 12]);
ylim([-2, 2]);
zlim([8, 12]);
axis([-3 3 -2 12 7 14]); 
tttime = p.time;
t = size(p.time,1);
save('time.mat','t');
w = W.data;

m = 11; 
pp = [];
pp1 = [];

for i = 1:50:10001
        cla;

        axis([-3 3 -2 12 7 14]); 
        pp_ = position(:,i); 
        ww_ = w(:,:,i);
        pp1_ = position1(:,i);
        p_poly = [];
        p_poly1 = [];
        x = [];
        y = [];
        z = [];
    for j = 1:m 
        p_poly(:,j) = [pp_(3*j-2); pp_(3*j-1); pp_(3*j)]; 
        p_poly1(:,j) = [pp1_(3*j-2); pp1_(3*j-1); pp1_(3*j)];
          if j ~= 1 
             h = plot3(p_poly(2,j),p_poly(1,j),-p_poly(3,j),'Color',[255/255 90/255 33/255],'Marker','.','MarkerSize',10,'EraseMode','xor');
             axis([-3 3 -2 12 7 14]); 
             hold on
             h = plot3(p_poly1(2,j),p_poly1(1,j),-p_poly1(3,j),'Color',[0.2 0.2 0.5],'Marker','.','MarkerSize',7,'EraseMode','xor');
             line([p_poly(2,1),p_poly(2,j)],[p_poly(1,1),p_poly(1,j)],-[p_poly(3,1),p_poly(3,j)],'Color',[128/255, 39/255, 3/255],'LineStyle','-','LineWidth',0.7);
             string = line([p_poly(2,j),p_poly1(2,j)],[p_poly(1,j),p_poly1(1,j)],-[p_poly(3,j),p_poly1(3,j)],'Color',[0.5 0.2 0.5],'LineStyle','-.','LineWidth',0.7);
             string.Color(4) = 0.3; 
             if j ~= m
                for k = j+1:m
                    neighbor = [pp1_(3*k-2); pp1_(3*k-1); pp1_(3*k)];
                    if norm(p_poly1(:,j)-neighbor, 2) < distance
                        string1 = line([neighbor(2),p_poly1(2,j)],[neighbor(1),p_poly1(1,j)],-[neighbor(3),p_poly1(3,j)],'Color',[0.5 0.2 0.5],'LineStyle','-.','LineWidth',0.7);
                        string1.Color(4) = 0.3;
                    end
                end
             end
             axis([-3 3 -2 12 7 14]); 
             hold on
          else 
             h = plot3(p_poly(2,j),p_poly(1,j),-p_poly(3,j),'Color',[145/255 213/255 66/255],'Marker','.','MarkerSize',30,'EraseMode','xor');
             axis([-3 3 -2 12 7 14]); 
             hold on
             grid on
          end
    end
    pause(0.1);
    axis([-3 3 -2 12 7 14]); 
    axis equal
end

for j = 1:m
        pp = [position(3*j-2,:); position(3*j-1,:); position(3*j,:)]; 
        pp1 = [position1(3*j-2,:); position1(3*j-1,:); position1(3*j,:)];
        v_norm_ = velo_norm_(j,:);
        v_norm1_ = velo_norm1_(j,:);

        pp(:,end) = NaN;
        pp1(:,end) = NaN;

        if j == 1
            g = patch(pp(2,:),pp(1,:),-pp(3,:), v_norm_,'EdgeColor','interp','LineWidth',2);
            axis([-3 3 -2 12 7 16]); 
            colorbar

            hold on
        end
end
