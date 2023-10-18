%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%  Code modified by Francisco Vasconcelos from
%%%%
%%%%  Drone class, feel free to add functionality as you see fit
%%%%  Author: Daniel Butters
%%%%  Date: 16/11/17
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

classdef Drone_2b < handle
    properties (Constant)
        %width, length, height offset between centre and rotors
        % body = [0.6 0.6 0];
        body = [1.6 1.6 0.0];
        
        %time interval for simulation (seconds)
        time_interval = 0.02;
        %time_interval = 0.005;
        
        % size of floating window that follows drone
        axis_size = 2.;
        
        %colours of each component of drone model
        colours = [[.8 .3 .1];[.2 .2 .5];[.8 .1 .3];[.9 .6 .8];[.9 .2 .4]];
        
        %Follows the drone within the figure
        %Don't use if you are simulating more than one drone!
        %Switch to false to see the overall world view
        drone_follow = true;
    end
    properties
        %axis to draw on
        axis
        
        %length of one side of the flight arena
        spaceDim
        
        %limits of flight arena
        spaceLimits
        
        %drone position
        pos
        
        %drone rotation matrix
        R
        
        
        %Simulation time
        time
        
        %parameter to start drone in random position
        pos_offset
        
        %number of drones
        num_drones
        
        m
        
        I
        
        g
        
        kd
        
        k
        
        L
        
        b
        
        x

        A

        B

        C

        D

        y
    end
    methods
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %INSTANTIATION OF CLASS
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function obj = Drone_2b(axis, spaceDim, num_drones)
            if nargin > 1
                load('disc_matrices.mat');

                obj.axis = axis;
                
                obj.spaceDim = spaceDim;
                
                obj.spaceLimits = [(-spaceDim/2)+10 (spaceDim/2)-10 (-spaceDim/2)+10 (spaceDim/2)-10 10 spaceDim-10];
                
                obj.pos = [0;0;5];
                
                %obj.pos_offset = [5.*(rand - 0.5),5.*(rand - 0.5),2.5.*(rand)];
                
                obj.R = [1,0,0;0,1,0;0,0,1];
                
                obj.time = 0;
                
                obj.num_drones = num_drones;
                
                % fixed parameters
                obj.m = 0.2;
                obj.I = [1 0 0;
                    0 1 0;
                    0 0 0.5];
                obj.g = 9.8;
                obj.kd = 0.1;
                obj.k = 1;
                obj.L = 0.2;
                obj.b = 0.1;
                obj.x = [0;0;5;0;0;0;0;0;0;0;0;0];
                obj.y = [0;0;5;0;0;0;0;0;0;0;0;0];
                %f-A*x-B*u
                obj.A = sys_d.A;
                obj.B = sys_d.B;
                obj.C = sys_d.C;
                obj.D = sys_d.D;
            else
                error('Drone not initialised correctly')
            end
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %DRAWING OF DRONE TO FIGURE
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function draw(obj)
            %how big should the moving window be
            cL = obj.axis_size;
            
            %set to false if you want to see world view
            %if(obj.drone_follow)
            %    axis([obj.pos(1)-cL obj.pos(1)+cL obj.pos(2)-cL obj.pos(2)+cL obj.pos(3)-cL obj.pos(3)+cL]);
            %end
            
            %create middle sphere
            [X Y Z] = sphere(8);
            %[X Y Z] = (obj.body(1)/5.).*[X Y Z];
            X = (obj.body(1)/5.).*X + obj.pos(1);
            Y = (obj.body(1)/5.).*Y + obj.pos(2);
            Z = (obj.body(1)/5.).*Z + obj.pos(3);
            s = surf(obj.axis,X,Y,Z);
            set(s,'edgecolor','none','facecolor',obj.colours(1,:));
            
            %create side spheres
            %front, right, back, left
            hOff = obj.body(3)/2;
            Lx = obj.body(1)/2;
            Ly = obj.body(2)/2;
            rotorsPosBody = [...
                0    Ly    0    -Ly;
                Lx    0    -Lx   0;
                hOff hOff hOff hOff];
            rotorsPosInertial = zeros(3,4);
            for i = 1:4
                rotorPosBody = rotorsPosBody(:,i);
                rotorsPosInertial(:,i) = bodyToInertial(obj,rotorPosBody);
                [X Y Z] = sphere(8);
                X = (obj.body(1)/8.).*X + obj.pos(1) + rotorsPosInertial(1,i);
                Y = (obj.body(1)/8.).*Y + obj.pos(2) + rotorsPosInertial(2,i);
                Z = (obj.body(1)/8.).*Z + obj.pos(3) + rotorsPosInertial(3,i);
                s = surf(obj.axis,X,Y,Z);
                set(s,'edgecolor','none','facecolor',obj.colours(i+1,:));
            end
            obj.axis.Title.String = ['Sim Time = ',num2str(obj.time,'%f'),' seconds'];
        end
        
        function vectorInertial = bodyToInertial(obj, vectorBody)
            vectorInertial = obj.R*vectorBody;
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %SIMULATION FUNCTIONS
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        %demo (not useful) code to show varying position and rotation
        %replace with your own functions!      
        
        function obj = change_pos_and_orientation(obj)

            t = obj.time;
%             dt = 0.02;
            % define inputs from 0-8s
            if t >0 && t<=2
                input = [0.49;0.49;0.49;0.49];
            elseif t>2 && t<=4
                input = [0.5635; 0.5635;0.5635;0.5635];
            elseif t>4
                input = [0.5635; 0.5635;0;0.5635]; 
            end 
            % Step through the simulation, updating the state.
            % Take input from our controller.
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            obj.A
            obj.B
            i = input-0.49;
            obj.x = obj.A*obj.x+obj.B*i;
            obj.y = obj.C*obj.x+obj.D*i;
            pos_mat = obj.y(1:3);
            pitch = obj.y(7);
            roll = obj.y(8);
            yaw = obj.y(9); 
            rot_mat = eul2rotm([yaw, roll, pitch]);
            
            %update position and rotation matrix of drone
            obj.pos = pos_mat;
            obj.R = rot_mat;
            
            
        end
        
        
        function update(obj)
            %update simulation time
            obj.time = obj.time + obj.time_interval;
            
            %change position and orientation of drone
            obj = change_pos_and_orientation(obj);
            
            %draw drone on figure
            draw(obj);
        end
    end
end
