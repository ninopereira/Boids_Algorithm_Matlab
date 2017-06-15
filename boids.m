%% Boids Algorithm
% This matlab version was implemented following the instructions from:
% http://www.kfish.org/boids/pseudocode.html
% Original algorithm by Reynolds [http://www.red3d.com/cwr/]
%
% By Nino Pereira 2016

function boids(numBoids)
    
    global boids
    global config
    if nargin<1
        config.numBoids = 0;
    else
        config.numBoids = numBoids;
    end
    config.area = [0 32 0 18];
    config.area = [0 16 0 9];
    config.max_speed = 0.3;
    config.rule1 = 1;
    config.rule2 = 1;
    config.rule3 = 1;
    config.invertRule1 = 0;
    config.getTogether = 100 % rule 1 factor
    config.minDistance = 0.5; % rule 2 factor
    config.averageSpeedFactor = 200; % rule 3 factor the higher the smoother
    config.execute = 1;
    fig_handle = figure(1);
    axis equal;
    set(figure(fig_handle),'KeyPressFcn',@Key_Up); % interrupt when clicking fig
    
    initialise_positions();
    
	while(config.execute)
        clf(1);
        axis equal;
        axis([config.area(1)-0.1 config.area(2)+0.1 config.area(3)-0.1 config.area(4)+0.1]);
        hold on;
 		draw_boids();
		move_all_boids_to_new_positions();
%         pause(0.2);
        drawnow();
    end
		
function initialise_positions()
    global boids
    global config
    boids = [];
    numBoids = config.numBoids;
    if numBoids == 0
        numBoids =input('How many initial boids?');
    end
    for it = 1:numBoids
        xy = [config.area(1)+(config.area(2)-config.area(1))*rand()...
              config.area(1)+(config.area(2)-config.area(1))*rand()];
        u = [0.2*rand()-0.1 0.2*rand()-0.1];
        theta = atan2(u(2),u(1));
        boids(it).pose = [xy theta u]';
        boids(it).color = rand(1,3);
    end
    
function c = rule2(boidId)
    global boids
    global config
    c = 0;
    for it=1:size(boids,2)
        if it~=boidId
            if pdist([boids(it).pose(1:2)'; boids(boidId).pose(1:2)'],...
                    'euclidean')<config.minDistance
                c = c-(boids(it).pose(1:2)-boids(boidId).pose(1:2));
            end
        end
    end
    
function v3 = rule3(boidId)
    global boids
    global config
    pvJ = [0 0]';
    for it = 1:size(boids,2)
        if it ~=boidId
            pvJ = pvJ + boids(it).pose(4:5);
        end
    end
    v3 = (pvJ-boids(boidId).pose(4:5))/config.averageSpeedFactor;
    
function move_all_boids_to_new_positions()
    global boids
    global config
    area = config.area;
%     dt = 0.5;
    sumPos = sumAllPoses();
    numBoids = size(boids,2);
    for i = 1:numBoids
        %%keepInsideArea
        if boids(i).pose(1)>area(2)
%             boids(i).pose(4) = -boids(i).pose(4);
            boids(i).pose(1)=area(2);
        end
        if boids(i).pose(1)<area(1)
%             boids(i).pose(4) = -boids(i).pose(4);
            boids(i).pose(1)=area(1);
        end
        if boids(i).pose(2)>area(4)
%             boids(i).pose(5) = -boids(i).pose(5);
            boids(i).pose(2)=area(4);
        end
        if  boids(i).pose(2)<area(3)
%             boids(i).pose(5) = -boids(i).pose(5);
            boids(i).pose(2)=area(3);
        end
        
        if config.rule1 && numBoids>2
            v1 = ((sumPos-boids(i).pose(1:2))/(numBoids-1) - boids(i).pose(1:2))/config.getTogether;
        else
            v1 =[0 0]';
        end
        if config.invertRule1
            v1 = -v1;
        end
        if config.rule2 && numBoids>2
            v2 = rule2(i);
        else
            v2 =[0 0]';
        end
        if config.rule3 && numBoids>2
            v3 = rule3(i);
        else
            v3 =[0 0]';
        end   
        u = boids(i).pose(4:5) + v1 + v2 + v3;
        if norm(u,2)>config.max_speed
            u = u/norm(u,2)*config.max_speed;
        end
%         u = v1;
        % limit speeds
%         if u(1)>config.max_speed
%             u(1) = config.max_speed;
%         end
%         if u(1)<-config.max_speed
%             u(1) = -config.max_speed;
%         end
%         if u(2)>config.max_speed
%             u(2) = config.max_speed;
%         end
%         if u(2)<-config.max_speed
%             u(2) = -config.max_speed;
%         end
%         boids(i).pose = GetPoseAt(boids(i).pose,u, dt);
        new_xy = boids(i).pose(1:2) + u;
        theta = atan2(u(2),u(1));
        boids(i).pose = [new_xy; theta; u];
%         boids(i).pose(1) = mod(boids(i).pose(1) + area(2)-area(1),area(2)-area(1));
%         boids(i).pose(2) = mod(boids(i).pose(2) + area(4)-area(3),area(4)-area(3));

    end

function draw_boids()
    global boids

    for it= 1:size(boids,2)
        x = boids(it).pose(1);
        y = boids(it).pose(2);
        theta = boids(it).pose(3);
        XY = [-0.2  -0.1;...
              -0.2   0.1;...
              0.2   0; ...
              -0.2  -0.1];

        % rotate at (0,0)
        rotationArray = [cos(theta), -sin(theta);...
                         sin(theta), cos(theta)];
        XY=rotationArray*XY';

        % translate to desired center point
        XY(1,:) = XY(1,:) + [x x x x];
        XY(2,:) = XY(2,:) + [y y y y];
        line(XY(1,:),XY(2,:),'Color',boids(it).color,'LineWidth',2);
%         fill(XY(1,:),XY(2,:),boids(it).color);
    end
    
function sumPos = sumAllPoses()        
    global boids
    poses = [];
    poses = [boids.pose];
	sumPos = [sum(poses(1:2,:),2)];

function [] = Key_Up(src,evnt)
    global config
    key_value = evnt.Key;
    disp('Press "q" to quit simulation');
    switch key_value
        case 'q'
        config.execute = 0; %stop execution of the while loop
        case '1'
            config.rule1 = 1-config.rule1;
            disp(strcat('rule1 = ',num2str(config.rule1)));
        case '2'
            config.rule2 = 1-config.rule2;
            disp(strcat('rule2 = ',num2str(config.rule2)));
        case '3'
            config.rule3 = 1-config.rule3;
            disp(strcat('rule3 = ',num2str(config.rule3)));
        case '4'
            config.invertRule1 = 1- config.invertRule1;
            disp(strcat('InvertRule1 = ',num2str(config.invertRule1)));
        otherwise
        createBoid();
        
    end
    
function createBoid()
    global boids
    global config
    % create a new boid
    newBoidPose = [ginput(1) 0]';
    newBoidVel = [2*config.max_speed*rand()-config.max_speed 2*config.max_speed*rand()-config.max_speed]';
    k = size(boids,2);
    boids(k+1).pose = [newBoidPose; newBoidVel] ;
    boids(k+1).color = rand(1,3);