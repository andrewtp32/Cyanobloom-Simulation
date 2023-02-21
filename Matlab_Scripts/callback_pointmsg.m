function callback_pointmsg(src,msg)
    global points;
    points=struct('cornerX',[],'cornerY',[],'measX',[],'measY',[],'priorX',[],'priorY',[],'postX',[],'postY',[],'postW',[]); 
    PointCd=msg;
    
    framedetails=PointCd.Header.FrameId;% meas_ends=6,prior=12,post=18
    meas_ends=str2double( extractBetween(framedetails,"meas_ends=",",p"));
    prior_est_ends=str2double( extractBetween(framedetails,"prior=",",post"));
    est_size=size(msg.Points);
    for i=1:size(msg.Points)
        if i<5 %get the four edges
            
            points.cornerX(end+1)=msg.Points(i).X;
            points.cornerY(end+1)=msg.Points(i).Y;
%             scatter(msg.Points(i).X,msg.Points(i).Y,'Marker','*');
%             hold on
        
        elseif (i>4 && i<(meas_ends+1))%gets the measurements
            points.measX(end+1)=msg.Points(i).X;
            points.measY(end+1)=msg.Points(i).Y;
%             scatter(msg.Points(i).X,msg.Points(i).Y,'Marker','*');
        end
        if (i>meas_ends && i<(prior_est_ends+1))%gets the prior estimates
            points.priorX(end+1)=msg.Points(i).X;
            points.priorY(end+1)=msg.Points(i).Y;
            
        elseif i>prior_est_ends %gets the final estimates
            points.postX(end+1)=msg.Points(i).X;
            points.postY(end+1)=msg.Points(i).Y;
            points.postW(end+1)=msg.Points(i).Z;
%             msg.Points(i).Z/.001
%             msg.Points(i).Z
            
        end
    end
    
    %swapping 3 and 4 so it could be plotted like a square
            swaptempx=points.cornerX(3);
            swaptempy=points.cornerY(3);
            
            points.cornerX(3)=points.cornerX(4);
            points.cornerY(3)=points.cornerY(4);
            
            points.cornerX(4)=swaptempx;
            points.cornerY(4)=swaptempy;
            
    %adding first point to the 5th so square could be completed
            points.cornerX(5)=points.cornerX(1);
            points.cornerY(5)=points.cornerY(1);
            
            
    %points
    plot(points.cornerX',points.cornerY','Marker','*');
    hold on
    scatter(points.measX',points.measY','Marker','x');
    %scatter(points.priorX',points.priorY','Marker','+');
    %points.postW;
    scatter(points.postX',points.postY',points.postW','Marker','o');%,points.postW'
    hold off

    %plot(points.x',points.y')
    %showdetails(data)
    pause(2)
end