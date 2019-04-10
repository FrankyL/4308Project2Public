%Scan obs.txt and get coordinates of all obstacles
fileID = fileread('obstacle.txt');
data = textscan( fileID, '%f %f') 
fclose all;
obsnum = size(data{1})
allxobs = data{1};
allyobs = data{2};

%Set up waypoint coordinates
waypt=[-1.5 1.5 ; 2 0.1 ; 2 -0.125; -1.5 -1.5]


% Array setup and create an array with resolution stated in gridres
gridres = 180;
actgrid = 6;
grid_X = gridres;
grid_Y = gridres;
val = 10;
grid_map = 2*(ones(grid_X,grid_Y));

%set up plotting map to illustrate A*star navigation
j=0;
x_val = 1;
y_val = 1;
axis([0 grid_X 0 grid_Y])
% axis([-grid_X/2-(.5*5) grid_X/2+(.5*5) -grid_Y/2-(.5*5) grid_Y/2+(.5*5)])
grid on;
hold on;
n=0;


%read path.txt and start writing into it the take off sequence
fileIDPath = fopen('path2.txt','w');
%take off

fprintf(fileIDPath, 'Taking off sequence\r\n');
height = 0;
tAllot = 3;
nDataP = tAllot * 20; %number of data points = time allocated * 20 Hz
dPDataP = 1/nDataP; % distance per data point =  1m / number of data point
height = 0:dPDataP:1;
fprintf(fileIDPath, '-1.500 1.500 %4.3f 0 0 0 0 0 0 0 0\r\n',height);


%setup destination coordinate on grid
xDestTxt=waypt(2,1)
yDestTxt=waypt(2,2)
xval = round(-yDestTxt* gridres/actgrid + gridres/2)
yval = round(xDestTxt* gridres/actgrid + gridres/2)
xTarget = xval;
yTarget = yval;
grid_map(xval,yval) =0;
plot(xval+0.5,yval+0.5,'gd');
text(xval+1,yval+.5,'Target')

%set up obstacles with 60 cm area using obs.txt
for m=1:obsnum(1,1)
    xobs = allxobs(m)
    yobs = allyobs(m)
    xval = (-yobs* gridres/actgrid + gridres/2);
    yval = (xobs* gridres/actgrid + gridres/2);
    actperi = 0.6;

    circperi = 0;
    j=0;
    while(circperi < actperi * gridres/actgrid)
        i=0;
        while(circperi < actperi * gridres/actgrid)
            grid_map(round(xval+i),round(yval+j))=-1;
            plot(xval+i+.5,yval+j+.5,'ro');
            i=i+1;
            circperi=sqrt(i*i+j*j);
        end
        i=0;
        circperi=sqrt(i*i+j*j);

        i=0;
        while(circperi < actperi * gridres/actgrid)
            grid_map(round(xval+i),round(yval+j))=-1;
            plot(xval+i+.5,yval+j+.5,'ro');
            i=i-1;
            circperi=sqrt(i*i+j*j);
        end
        i=0;
        circperi=sqrt(i*i+j*j);

        j=j+1;
    end
    circperi = 0;
    j=0;
    while(circperi < actperi * gridres/actgrid)
        i=0;
        while(circperi < actperi * gridres/actgrid)
            grid_map(round(xval+i),round(yval+j))=-1;
            plot(xval+i+.5,yval+j+.5,'ro');
            i=i+1;
            circperi=sqrt(i*i+j*j);
        end
        i=0;
        circperi=sqrt(i*i+j*j);

        i=0;
        while(circperi < actperi * gridres/actgrid)
            grid_map(round(xval+i),round(yval+j))=-1;
            plot(xval+i+.5,yval+j+.5,'ro');
            i=i-1;
            circperi=sqrt(i*i+j*j);
        end
        i=0;
        circperi=sqrt(i*i+j*j);

        j=j-1;
    end
end
  


% Setup Starting waypoint
xStartTxt=waypt(1,1)
yStartTxt=waypt(1,2)
xval = round(-yStartTxt* gridres/actgrid + gridres/2)
yval = round(xStartTxt* gridres/actgrid + gridres/2)
xStart = xval
yStart = yval
grid_map(xStart,yStart)=1;
plot(xval+.5,yval+.5,'bo');



OPEN=[];
CLOSED=[];
%Put all obstacles on the Closed list
k=1;%Dummy counter
for i=1:grid_X
    for j=1:grid_Y
        if(grid_map(i,j) == -1)
            CLOSED(k,1)=i; 
            CLOSED(k,2)=j; 
            k=k+1;
        end
    end
end
CLOSED_COUNT=size(CLOSED,1);

xNode=xval;
yNode=yval;
OPEN_COUNT=1;
path_cost=0;
goal_distance=distance(xNode,yNode,xTarget,yTarget);
OPEN(OPEN_COUNT,:)=insert_open(xNode,yNode,xNode,yNode,path_cost,goal_distance,goal_distance);
OPEN(OPEN_COUNT,1)=0;
CLOSED_COUNT=CLOSED_COUNT+1;
CLOSED(CLOSED_COUNT,1)=xNode;
CLOSED(CLOSED_COUNT,2)=yNode;
NoPath=1;

while((xNode ~= xTarget || yNode ~= yTarget) && NoPath == 1)
%  plot(xNode+.5,yNode+.5,'go');
 exp_array=expand_array(xNode,yNode,path_cost,xTarget,yTarget,CLOSED,grid_X,grid_Y);
 exp_count=size(exp_array,1);
 %UPDATE LIST OPEN WITH THE SUCCESSOR NODES
 %OPEN LIST FORMAT
 %--------------------------------------------------------------------------
 %IS ON LIST 1/0 |X val |Y val |Parent X val |Parent Y val |h(n) |g(n)|f(n)|
 %--------------------------------------------------------------------------
 %EXPANDED ARRAY FORMAT
 %--------------------------------
 %|X val |Y val ||h(n) |g(n)|f(n)|
 %--------------------------------
 for i=1:exp_count
    flag=0;
    for j=1:OPEN_COUNT
        if(exp_array(i,1) == OPEN(j,2) && exp_array(i,2) == OPEN(j,3) )
            OPEN(j,8)=min(OPEN(j,8),exp_array(i,5)); %#ok<*SAGROW>
            if OPEN(j,8)== exp_array(i,5)
                %UPDATE PARENTS,gn,hn
                OPEN(j,4)=xNode;
                OPEN(j,5)=yNode;
                OPEN(j,6)=exp_array(i,3);
                OPEN(j,7)=exp_array(i,4);
            end;%End of minimum fn check
            flag=1;
        end;%End of node check
%         if flag == 1
%             break;
    end;%End of j for
    if flag == 0
        OPEN_COUNT = OPEN_COUNT+1;
        OPEN(OPEN_COUNT,:)=insert_open(exp_array(i,1),exp_array(i,2),xNode,yNode,exp_array(i,3),exp_array(i,4),exp_array(i,5));
     end;%End of insert new element into the OPEN list
 end;%End of i for
 %Find out the node with the smallest fn 
  index_min_node = min_fn(OPEN,OPEN_COUNT,xTarget,yTarget);
  if (index_min_node ~= -1)    
   %Set xNode and yNode to the node with minimum fn
   xNode=OPEN(index_min_node,2);
   yNode=OPEN(index_min_node,3);
   path_cost=OPEN(index_min_node,6);%Update the cost of reaching the parent node
  %Move the Node to list CLOSED
  CLOSED_COUNT=CLOSED_COUNT+1;
  CLOSED(CLOSED_COUNT,1)=xNode;
  CLOSED(CLOSED_COUNT,2)=yNode;
  OPEN(index_min_node,1)=0;
  else
      %No path exists to the Target!!
      NoPath=0;%Exits the loop!
  end;%End of index_min_node check
end;%End of While Looph

i=size(CLOSED,1);
Optimal_path=[];
xval=CLOSED(i,1);
yval=CLOSED(i,2);
i=1;
Optimal_path(i,1)=xval;
Optimal_path(i,2)=yval;
i=i+1;

if ( (xval == xTarget) && (yval == yTarget))
    inode=0;
   %Traverse OPEN and determine the parent nodes
   parent_x=OPEN(node_index(OPEN,xval,yval),4);%node_index returns the index of the node
   parent_y=OPEN(node_index(OPEN,xval,yval),5);
   while( parent_x ~= xStart || parent_y ~= yStart)
           Optimal_path(i,1) = parent_x;
           Optimal_path(i,2) = parent_y;
           inode=node_index(OPEN,parent_x,parent_y);
           parent_x=OPEN(inode,4);%node_index returns the index of the node
           parent_y=OPEN(inode,5);
           i=i+1;
    end;
 j=size(Optimal_path,1);
 p=plot(Optimal_path(j,1)+.5,Optimal_path(j,2)+.5,'bo');
 j=j-1;
 for i=j:-1:1
%   pause(.25);
  set(p,'XData',Optimal_path(i,1)+.5,'YData',Optimal_path(i,2)+.5);
 drawnow ;
 end;
 plot(Optimal_path(:,1)+.5,Optimal_path(:,2)+.5);
else
 pause(1);
 h=msgbox('Sorry, No path exists to the Target!','warn');
 uiwait(h,5);
end



% get the path used by algorithm and convert it from grid's coordinate to
% gazebo's coordinate
first_path = flipud(Optimal_path);
first_path_xgrid = first_path(:,1); 
first_path_ygrid = first_path(:,2); 
first_path_xgaz = (first_path_ygrid - gridres/2)/(gridres/actgrid);
first_path_ygaz = (gridres/2-first_path_xgrid)/(gridres/actgrid);



%get robot to settle to the grid's equivalent coordinate

% fprintf(fileIDPath, '%4.3f %4.3f 1.000 0 0 0 0 0 0 0 0\r\n',A);
% coordN = size(first_path_xgaz);
% tAllot = 0.4;
% nDataP = tAllot * 20;
% dPDataPx = (first_path_xgaz(1)- (-1.5))/nDataP; 
% dPDataPy = (first_path_ygaz(1)- 1.5)/nDataP;
% if (-1.5<first_path_xgaz(1)) 
%     dPDataPx = abs(dPDataPx);
% end
% if (1.5<first_path_ygaz(1)) 
%     dPDataPy = abs(dPDataPy);
% end 
% % if (first_path_xgaz(1) )
% settlex = -1.5:dPDataPx:first_path_xgaz(1);
% settley = 1.5:dPDataPy:first_path_ygaz(1);
% A = [settlex;settley];
% fprintf(fileIDPath, '%4.3f %4.3f 1.000 0 0 0 0 0 0 0 0\r\n',A);


fprintf(fileIDPath, '===\r\n');
% for i=2:coordN
%     gap = (first_path_xgaz(i+4*(i-2))-first_path_xgaz(i+4*(i-2)-1) )/5;
%     first_path_xgaz( i+4*(i-1) : coordN+4*(i-1) ) = first_path_xgaz( i+4*(i-2) : coordN+4*(i-2) );
%     for j=0:3
%         first_path_xgaz( i+4*(i-2)+j ) = first_path_xgaz( i+4*(i-2)-1 ) + gap*(j+1); 
%     end
% end
% for i=2:coordN
%     gap = (first_path_ygaz(i+4*(i-2))-first_path_ygaz(i+4*(i-2)-1) )/5;
%     first_path_ygaz( i+4*(i-1) : coordN+4*(i-1) ) = first_path_ygaz( i+4*(i-2) : coordN+4*(i-2) );
%     for j=0:3
%         first_path_ygaz( i+4*(i-2)+j ) = first_path_ygaz( i+4*(i-2)-1 ) + gap*(j+1); 
%     end
% end
fprintf(fileIDPath, 'Travel from A to B\r\n');
first_path_xgaz=transpose(first_path_xgaz);
first_path_ygaz=transpose(first_path_ygaz);
A = [first_path_xgaz ; first_path_ygaz];
fprintf(fileIDPath, '%4.3f %4.3f 1.000 0 0 0 0 0 0 0 0\r\n',A);
fprintf(fileIDPath, '===\r\n');








fprintf(fileIDPath, 'Horizontal Flight through point B\r\n');
height = 0;
tAllot = 0.75;
nDataP = tAllot * 20; %number of data points = time allocated * 20 Hz
dPDataP = - (waypt(2,2) - waypt(3,2))/nDataP; % distance per data point =  1m / number of data point
height = waypt(2,2):dPDataP:waypt(3,2);
fprintf(fileIDPath, '2.000 %4.3f 1.000 0 0 0 0 0 0 0 0\r\n',height);

%BEGIN MOVING FROM WAYPOINT B TO C#########################################




xDestTxt=waypt(4,1)
yDestTxt=waypt(4,2)
xval = round(-yDestTxt*gridres/actgrid + gridres/2)
yval = round(xDestTxt*gridres/actgrid + gridres/2)
xTarget = xval;
yTarget = yval;
grid_map(xval,yval) =0;
plot(xval+0.5,yval+0.5,'gd');
text(xval+1,yval+.5,'Target')

for m=1:obsnum(1,1)
    xobs = allxobs(m)
    yobs = allyobs(m)
    xval = (-yobs*gridres/actgrid + gridres/2);
    yval = (xobs*gridres/actgrid + gridres/2);
    actperi = 0.6;

    
    
    circperi = 0;
    j=0;
    while(circperi < actperi * gridres/actgrid)
        i=0;
        while(circperi < actperi * gridres/actgrid)
            grid_map(round(xval+i),round(yval+j))=-1;
            plot(xval+i+.5,yval+j+.5,'ro');
            i=i+1;
            circperi=sqrt(i*i+j*j);
        end
        i=0;
        circperi=sqrt(i*i+j*j);

        i=0;
        while(circperi < actperi * gridres/actgrid)
            grid_map(round(xval+i),round(yval+j))=-1;
            plot(xval+i+.5,yval+j+.5,'ro');
            i=i-1;
            circperi=sqrt(i*i+j*j);
        end
        i=0;
        circperi=sqrt(i*i+j*j);

        j=j+1;
    end
    circperi = 0;
    j=0;
    while(circperi < actperi * gridres/actgrid)
        i=0;
        while(circperi < actperi * gridres/actgrid)
            grid_map(round(xval+i),round(yval+j))=-1;
            plot(xval+i+.5,yval+j+.5,'ro');
            i=i+1;
            circperi=sqrt(i*i+j*j);
        end
        i=0;
        circperi=sqrt(i*i+j*j);

        i=0;
        while(circperi < actperi * gridres/actgrid)
            grid_map(round(xval+i),round(yval+j))=-1;
            plot(xval+i+.5,yval+j+.5,'ro');
            i=i-1;
            circperi=sqrt(i*i+j*j);
        end
        i=0;
        circperi=sqrt(i*i+j*j);

        j=j-1;
    end
end
  



% Setup Starting waypoint
xStartTxt=waypt(3,1)
yStartTxt=waypt(3,2)
xval = round(-yStartTxt*gridres/actgrid + gridres/2)
yval = round(xStartTxt*gridres/actgrid + gridres/2)
xStart = xval
yStart = yval
grid_map(xStart,yStart)=1;
plot(xval+.5,yval+.5,'bo');





% 
% for z = 2:3
% 
% xDestTxt=waypt(z,1)
% yDestTxt=waypt(z,2)
% xval = round(-yDestTxt*5+15)
% yval = round(xDestTxt*5+15)
% xTarget = xval;
% yTarget = yval;
% grid_map(xval,yval) =0;
% plot(xval+0.5,yval+0.5,'gd');
% text(xval+1,yval+.5,'Target')

OPEN=[];
CLOSED=[];
%Put all obstacles on the Closed list
k=1;%Dummy counter
for i=1:grid_X
    for j=1:grid_Y
        if(grid_map(i,j) == -1)
            CLOSED(k,1)=i; 
            CLOSED(k,2)=j; 
            k=k+1;
        end
    end
end
CLOSED_COUNT=size(CLOSED,1);

xNode=xval;
yNode=yval;
OPEN_COUNT=1;
path_cost=0;
goal_distance=distance(xNode,yNode,xTarget,yTarget);
OPEN(OPEN_COUNT,:)=insert_open(xNode,yNode,xNode,yNode,path_cost,goal_distance,goal_distance);
OPEN(OPEN_COUNT,1)=0;
CLOSED_COUNT=CLOSED_COUNT+1;
CLOSED(CLOSED_COUNT,1)=xNode;
CLOSED(CLOSED_COUNT,2)=yNode;
NoPath=1;

while((xNode ~= xTarget || yNode ~= yTarget) && NoPath == 1)
%  plot(xNode+.5,yNode+.5,'go');
 exp_array=expand_array(xNode,yNode,path_cost,xTarget,yTarget,CLOSED,grid_X,grid_Y);
 exp_count=size(exp_array,1);
 %UPDATE LIST OPEN WITH THE SUCCESSOR NODES
 %OPEN LIST FORMAT
 %--------------------------------------------------------------------------
 %IS ON LIST 1/0 |X val |Y val |Parent X val |Parent Y val |h(n) |g(n)|f(n)|
 %--------------------------------------------------------------------------
 %EXPANDED ARRAY FORMAT
 %--------------------------------
 %|X val |Y val ||h(n) |g(n)|f(n)|
 %--------------------------------
 for i=1:exp_count
    flag=0;
    for j=1:OPEN_COUNT
        if(exp_array(i,1) == OPEN(j,2) && exp_array(i,2) == OPEN(j,3) )
            OPEN(j,8)=min(OPEN(j,8),exp_array(i,5)); %#ok<*SAGROW>
            if OPEN(j,8)== exp_array(i,5)
                %UPDATE PARENTS,gn,hn
                OPEN(j,4)=xNode;
                OPEN(j,5)=yNode;
                OPEN(j,6)=exp_array(i,3);
                OPEN(j,7)=exp_array(i,4);
            end;%End of minimum fn check
            flag=1;
        end;%End of node check
%         if flag == 1
%             break;
    end;%End of j for
    if flag == 0
        OPEN_COUNT = OPEN_COUNT+1;
        OPEN(OPEN_COUNT,:)=insert_open(exp_array(i,1),exp_array(i,2),xNode,yNode,exp_array(i,3),exp_array(i,4),exp_array(i,5));
     end;%End of insert new element into the OPEN list
 end;%End of i for
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 %END OF WHILE LOOP
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 %Find out the node with the smallest fn 
  index_min_node = min_fn(OPEN,OPEN_COUNT,xTarget,yTarget);
  if (index_min_node ~= -1)    
   %Set xNode and yNode to the node with minimum fn
   xNode=OPEN(index_min_node,2);
   yNode=OPEN(index_min_node,3);
   path_cost=OPEN(index_min_node,6);%Update the cost of reaching the parent node
  %Move the Node to list CLOSED
  CLOSED_COUNT=CLOSED_COUNT+1;
  CLOSED(CLOSED_COUNT,1)=xNode;
  CLOSED(CLOSED_COUNT,2)=yNode;
  OPEN(index_min_node,1)=0;
  else
      %No path exists to the Target!!
      NoPath=0;%Exits the loop!
  end;%End of index_min_node check
end;%End of While Loop
%Once algorithm has run The optimal path is generated by starting of at the
%last node(if it is the target node) and then identifying its parent node
%until it reaches the start node.This is the optimal path

i=size(CLOSED,1);
Optimal_path=[];
xval=CLOSED(i,1);
yval=CLOSED(i,2);
i=1;
Optimal_path(i,1)=xval;
Optimal_path(i,2)=yval;
i=i+1;




if ( (xval == xTarget) && (yval == yTarget))
    inode=0;
   %Traverse OPEN and determine the parent nodes
   parent_x=OPEN(node_index(OPEN,xval,yval),4);%node_index returns the index of the node
   parent_y=OPEN(node_index(OPEN,xval,yval),5);
   
   while( parent_x ~= xStart || parent_y ~= yStart)
           Optimal_path(i,1) = parent_x;
           Optimal_path(i,2) = parent_y;
           %Get the grandparents:-)
           inode=node_index(OPEN,parent_x,parent_y);
           parent_x=OPEN(inode,4);%node_index returns the index of the node
           parent_y=OPEN(inode,5);
           i=i+1;
    end;
 j=size(Optimal_path,1);
 %Plot the Optimal Path!
 p=plot(Optimal_path(j,1)+.5,Optimal_path(j,2)+.5,'bo');
 j=j-1;
 for i=j:-1:1
%   pause(.25);
  set(p,'XData',Optimal_path(i,1)+.5,'YData',Optimal_path(i,2)+.5);
 drawnow ;
 end;
 plot(Optimal_path(:,1)+.5,Optimal_path(:,2)+.5);
else
 pause(1);
 h=msgbox('Sorry, No path exists to the Target!','warn');
 uiwait(h,5);
end

% second_path=flipud(Optimal_path);
% get the path used by algorithm and convert it from grid's coordinate to
% gazebo's coordinate
second_path = flipud(Optimal_path);
second_path_xgrid = second_path(:,1); 
second_path_ygrid = second_path(:,2); 
second_path_xgaz = (second_path_ygrid - gridres/2)/(gridres/actgrid);
second_path_ygaz = ((gridres/2) - second_path_xgrid)/(gridres/actgrid);



% 
% %get robot to settle to the grid's equivalent coordinate
% coordN = size(second_path_xgaz);
% tAllot = 0.4;
% nDataP = tAllot * 20;
% dPDataPx = (second_path_xgaz(1)- 2)/nDataP; 
% dPDataPy = (second_path_ygaz(1)- 0)/nDataP;
% if (2<second_path_xgaz(1)) 
%     dPDataPx = abs(dPDataPx);
% end
% if (0<second_path_ygaz(1)) 
%     dPDataPy = abs(dPDataPy);
% end 
% settlex = 2:dPDataPx:second_path_xgaz(1);
% settley = 0:dPDataPy:second_path_ygaz(1);
% if (second_path_xgaz(1) == 2  && second_path_ygaz(1) ~= 0)
%     sizeofy = size(settley);
%     settlex = zeros(1, sizeofy(2));
% end 
% if (second_path_xgaz(1) ~= 2  && second_path_ygaz(1) == 0)
%     sizeofx = size(settlex);
%     settley = zeros(1, sizeofx(2));
% end 
% A = [settlex;settley];
% fprintf(fileIDPath, '%4.3f %4.3f 1.000 0 0 0 0 0 0 0 0\r\n',A);
% fprintf(fileIDPath, '===\r\n');


% for i=2:coordN
%     gap = (second_path_xgaz(i+4*(i-2))-second_path_xgaz(i+4*(i-2)-1) )/5;
%     second_path_xgaz( i+4*(i-1) : coordN+4*(i-1) ) = second_path_xgaz( i+4*(i-2) : coordN+4*(i-2) );
%     for j=0:3
%         second_path_xgaz( i+4*(i-2)+j ) = second_path_xgaz( i+4*(i-2)-1 ) + gap*(j+1); 
%     end
% end
% for i=2:coordN
%     gap = (second_path_ygaz(i+4*(i-2))-second_path_ygaz(i+4*(i-2)-1) )/5;
%     second_path_ygaz( i+4*(i-1) : coordN+4*(i-1) ) = second_path_ygaz( i+4*(i-2) : coordN+4*(i-2) );
%     for j=0:3
%         second_path_ygaz( i+4*(i-2)+j ) = second_path_ygaz( i+4*(i-2)-1 ) + gap*(j+1); 
%     end
% end
fprintf(fileIDPath, 'Travel from B to C\r\n');
second_path_xgaz=transpose(second_path_xgaz);
second_path_ygaz=transpose(second_path_ygaz);
A = [second_path_xgaz ; second_path_ygaz];
fprintf(fileIDPath, '%4.3f %4.3f 1.000 0 0 0 0 0 0 0 0\r\n',A);
fprintf(fileIDPath, '===\r\n');




fprintf(fileIDPath, 'Landing sequence\r\n');
height = 0;
tAllot = 3;
nDataP = tAllot * 20; %number of data points = time allocated * 20 Hz
dPDataP = -1/nDataP; % distance per data point =  1m / number of data point
height = 1:dPDataP:0;
fprintf(fileIDPath, '-1.500 -1.500 %4.3f 0 0 0 0 0 0 0 0\r\n',height);



% 
% final_destx = waypt(3,1);
% final_desty = waypt(3,2);
% % second_path=flipud(Optimal_path);
% % get the path used by algorithm and convert it from grid's coordinate to
% % gazebo's coordinate
% second_path = flipud(Optimal_path);
% second_path_xgrid = second_path(:,1); 
% second_path_ygrid = second_path(:,2); 
% second_path_xgaz = (second_path_ygrid - 15)/5;
% second_path_ygaz = (15-second_path_xgrid)/5;
% 
% %get robot to settle to the grid's equivalent coordinate
% coordN = size(second_path_xgaz);
% tAllot = 0.4;
% nDataP = tAllot * 20;
% dPDataPx = (final_destx- 2)/nDataP; 
% dPDataPy = (final_destx- 0)/nDataP;
% if (2<second_path_xgaz(1)) 
%     dPDataPx = abs(dPDataPx);
% end
% if (0<second_path_ygaz(1)) 
%     dPDataPy = abs(dPDataPy);
% end 
% settlex = 2:dPDataPx:second_path_xgaz(1);
% settley = 0:dPDataPy:second_path_ygaz(1);
% if (second_path_xgaz(1) == 2  && second_path_ygaz(1) ~= 0)
%     sizeofy = size(settley);
%     settlex = zeros(1, sizeofy(2));
% end 
% if (second_path_xgaz(1) ~= 2  && second_path_ygaz(1) == 0)
%     sizeofx = size(settlex);
%     settley = zeros(1, sizeofx(2));
% end 
% A = [settlex;settley];
% fprintf(fileIDPath, '%4.3f %4.3f 1.000 0 0 0 0 0 0 0 0\r\n',A);
% fprintf(fileIDPath, '===\r\n');
