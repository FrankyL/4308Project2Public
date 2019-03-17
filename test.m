fileID = fileread('obstacle.txt');
data = textscan( fileID, '%f %f') 
% fprintf(fileID,'%d %4.4f\n',y);
fclose all;
obsnum = size(data{1})
allxobs = data{1};
allyobs = data{2};

waypt=[-1.5 1.5 ; 2 0; -1.5 -1.5]


% Array setup
grid_X = 30;
grid_Y = 30;
val = 10;

grid_map = 2*(ones(grid_X,grid_Y));

j=0;
x_val = 1;
y_val = 1;
axis([0 grid_X 0 grid_Y])
% axis([-grid_X/2-(.5*5) grid_X/2+(.5*5) -grid_Y/2-(.5*5) grid_Y/2+(.5*5)])
grid on;
hold on;
n=0;


% % pause(1);
% h=msgbox('Please Select the Target using the Left Mouse button');
% uiwait(h,5);
% if ishandle(h) == 1
%     delete(h);
% end
% but = 0;
% while (but ~= 1)
%     [xval,yval,but]=ginput(1);
% end
xDestTxt=waypt(3,1)
yDestTxt=waypt(3,2)
xval = round(-yDestTxt*5+15)
yval = round(xDestTxt*5+15)
xTarget = xval;
yTarget = yval;
grid_map(xval,yval) =0;
plot(xval+0.5,yval+0.5,'gd');
text(xval+1,yval+.5,'Target')

for m=1:obsnum(1,1)
    xobs = allxobs(m)
    yobs = allyobs(m)
    xval = (-yobs*5+15);
    yval = (xobs*5+15);
    actperi = 0.6;

    
    
    circperi = 0;
    j=0;
    while(circperi < actperi * 5)
        i=0;
        while(circperi < actperi * 5)
            grid_map(round(xval+i),round(yval+j))=-1;
            plot(xval+i+.5,yval+j+.5,'ro');
            i=i+1;
            circperi=sqrt(i*i+j*j);
        end
        i=0;
        circperi=sqrt(i*i+j*j);

        i=0;
        while(circperi < actperi * 5)
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
    while(circperi < actperi * 5)
        i=0;
        while(circperi < actperi * 5)
            grid_map(round(xval+i),round(yval+j))=-1;
            plot(xval+i+.5,yval+j+.5,'ro');
            i=i+1;
            circperi=sqrt(i*i+j*j);
        end
        i=0;
        circperi=sqrt(i*i+j*j);

        i=0;
        while(circperi < actperi * 5)
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
xStartTxt=waypt(2,1)
yStartTxt=waypt(2,2)
xval = round(-yStartTxt*5+15)
yval = round(xStartTxt*5+15)
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
  pause(.25);
  set(p,'XData',Optimal_path(i,1)+.5,'YData',Optimal_path(i,2)+.5);
 drawnow ;
 end;
 plot(Optimal_path(:,1)+.5,Optimal_path(:,2)+.5);
else
 pause(1);
 h=msgbox('Sorry, No path exists to the Target!','warn');
 uiwait(h,5);
end

% end




% 
% 
% 
% 
% % Setup Starting waypoint
% xStartTxt=waypt(2,1)
% yStartTxt=waypt(2,2)
% xval = round(-yStartTxt*5+15)
% yval = round(xStartTxt*5+15)
% xStart = xval
% yStart = yval
% grid_map(xStart,yStart)=1;
% plot(xval+.5,yval+.5,'bo');
% 
% 
% xDestTxt=waypt(3,1)
% yDestTxt=waypt(3,2)
% xval = round(-yDestTxt*5+15)
% yval = round(xDestTxt*5+15)
% xTarget = xval;
% yTarget = yval;
% grid_map(xval,yval) =0;
% plot(xval+0.5,yval+0.5,'gd');
% text(xval+1,yval+.5,'Target')
% 
% 
% 
% OPEN=[];
% CLOSED=[];
% %Put all obstacles on the Closed list
% k=1;%Dummy counter
% for i=1:grid_X
%     for j=1:grid_Y
%         if(grid_map(i,j) == -1)
%             CLOSED(k,1)=i; 
%             CLOSED(k,2)=j; 
%             k=k+1;
%         end
%     end
% end
% CLOSED_COUNT=size(CLOSED,1);
% 
% 
% 
% xNode=xval;
% yNode=yval;
% OPEN_COUNT=1;
% path_cost=0;
% goal_distance=distance(xNode,yNode,xTarget,yTarget);
% OPEN(OPEN_COUNT,:)=insert_open(xNode,yNode,xNode,yNode,path_cost,goal_distance,goal_distance);
% OPEN(OPEN_COUNT,1)=0;
% CLOSED_COUNT=CLOSED_COUNT+1;
% CLOSED(CLOSED_COUNT,1)=xNode;
% CLOSED(CLOSED_COUNT,2)=yNode;
% NoPath=1;
% 
% fprintf('xNodeVal executed');
% 
% while((xNode ~= xTarget || yNode ~= yTarget) && NoPath == 1)
% %  plot(xNode+.5,yNode+.5,'go');
%  exp_array=expand_array(xNode,yNode,path_cost,xTarget,yTarget,CLOSED,grid_X,grid_Y);
%  exp_count=size(exp_array,1);
%  %UPDATE LIST OPEN WITH THE SUCCESSOR NODES
%  %OPEN LIST FORMAT
%  %--------------------------------------------------------------------------
%  %IS ON LIST 1/0 |X val |Y val |Parent X val |Parent Y val |h(n) |g(n)|f(n)|
%  %--------------------------------------------------------------------------
%  %EXPANDED ARRAY FORMAT
%  %--------------------------------
%  %|X val |Y val ||h(n) |g(n)|f(n)|
%  %--------------------------------
%  for i=1:exp_count
%     flag=0;
%     for j=1:OPEN_COUNT
%         if(exp_array(i,1) == OPEN(j,2) && exp_array(i,2) == OPEN(j,3) )
%             OPEN(j,8)=min(OPEN(j,8),exp_array(i,5)); %#ok<*SAGROW>
%             if OPEN(j,8)== exp_array(i,5)
%                 %UPDATE PARENTS,gn,hn
%                 OPEN(j,4)=xNode;
%                 OPEN(j,5)=yNode;
%                 OPEN(j,6)=exp_array(i,3);
%                 OPEN(j,7)=exp_array(i,4);
%             end;%End of minimum fn check
%             flag=1;
%         end;%End of node check
% %         if flag == 1
% %             break;
%     end;%End of j for
%     if flag == 0
%         OPEN_COUNT = OPEN_COUNT+1;
%         OPEN(OPEN_COUNT,:)=insert_open(exp_array(i,1),exp_array(i,2),xNode,yNode,exp_array(i,3),exp_array(i,4),exp_array(i,5));
%      end;%End of insert new element into the OPEN list
%  end;%End of i for
%   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  %END OF WHILE LOOP
%  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  %Find out the node with the smallest fn 
%   index_min_node = min_fn(OPEN,OPEN_COUNT,xTarget,yTarget);
%   if (index_min_node ~= -1)    
%    %Set xNode and yNode to the node with minimum fn
%    xNode=OPEN(index_min_node,2);
%    yNode=OPEN(index_min_node,3);
%    path_cost=OPEN(index_min_node,6);%Update the cost of reaching the parent node
%   %Move the Node to list CLOSED
%   CLOSED_COUNT=CLOSED_COUNT+1;
%   CLOSED(CLOSED_COUNT,1)=xNode;
%   CLOSED(CLOSED_COUNT,2)=yNode;
%   OPEN(index_min_node,1)=0;
%   else
%       %No path exists to the Target!!
%       NoPath=0;%Exits the loop!
%   end;%End of index_min_node check
% end;%End of While Loop
% %Once algorithm has run The optimal path is generated by starting of at the
% %last node(if it is the target node) and then identifying its parent node
% %until it reaches the start node.This is the optimal path
% 
% 
% 
% 
% i=size(CLOSED,1);
% Optimal_path=[];
% xval=CLOSED(i,1);
% yval=CLOSED(i,2);
% i=1;
% Optimal_path(i,1)=xval;
% Optimal_path(i,2)=yval;
% i=i+1;
% 
% 
% 
% 
% if ( (xval == xTarget) && (yval == yTarget))
%     inode=0;
%    %Traverse OPEN and determine the parent nodes
%    parent_x=OPEN(node_index(OPEN,xval,yval),4);%node_index returns the index of the node
%    parent_y=OPEN(node_index(OPEN,xval,yval),5);
%    
%    while( parent_x ~= xStart || parent_y ~= yStart)
%            Optimal_path(i,1) = parent_x;
%            Optimal_path(i,2) = parent_y;
%            %Get the grandparents:-)
%            inode=node_index(OPEN,parent_x,parent_y);
%            parent_x=OPEN(inode,4);%node_index returns the index of the node
%            parent_y=OPEN(inode,5);
%            i=i+1;
%     end;
%  j=size(Optimal_path,1);
%  %Plot the Optimal Path!
%  p=plot(Optimal_path(j,1)+.5,Optimal_path(j,2)+.5,'bo');
%  j=j-1;
%  for i=j:-1:1
%   pause(.25);
%   set(p,'XData',Optimal_path(i,1)+.5,'YData',Optimal_path(i,2)+.5);
%  drawnow ;
%  end;
%  plot(Optimal_path(:,1)+.5,Optimal_path(:,2)+.5);
% else
%  pause(1);
%  h=msgbox('Sorry, No path exists to the Target!','warn');
%  uiwait(h,5);
% end

