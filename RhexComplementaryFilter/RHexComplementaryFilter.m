classdef RHexComplementaryFilter < handle
% A complimentary filter for the snake robot.  The goal of this is
% to get a state estimate for the pose of the snake robot in an inertial
% frame with its origin at the CoG of the snake with gravity along the
% negative z-axis
    
    methods(Access = public)
        %Constructor
        function this = RhexComplementaryFilter(varargin)
        %COMPLEMENTARYFILTER
        %Arguments:
        %   snakeData
        %
        %Optional Parameters:
        %  'mexCode'           - 'true' (default), 'false' 
        
        p = inputParser;
        expectedMexCode = {'true', 'false'};
        
        addRequired(p, 'snakeData' ,@isstruct);
        addParameter(p, 'mexCode', 'true', ...
                     @(x) any(validatestring(x, ...
                                             expectedMexCode)));     
        addParameter(p, 'accelOffsets', []);
        addParameter(p, 'gyroOffsets', []);
        addParameter(p, 'gyrosTrustability', []);
                                         
        parse(p, varargin{:});
        
        this.snakeData = p.Results.snakeData;
        this.miscData.mexCode = strcmpi(p.Results.mexCode, 'true');
        
        this.accelOffset = p.Results.accelOffsets;
        this.gyroOffset = p.Results.gyroOffsets;
        
        if isempty(this.accelOffset)
            this.accelOffset = zeros(3, this.snakeData.num_modules);
        else
            assert(size(this.accelOffset,1)==3);
%             assert(size(this.accelOffset,2)==this.snakeData.num_modules);
        end
        
        if isempty(this.gyroOffset)
            this.gyroOffset = zeros(3, this.snakeData.num_modules);
        else
            assert(size(this.gyroOffset,1)==3);
%             assert(size(this.gyroOffset,2)==this.snakeData.num_modules);
        end

        this.firstRun = true;  
        this.everUpdated = false;
                
        if isempty(p.Results.gyrosTrustability)
            this.gyrosTrustability = ones(1, this.snakeData.num_modules);
        else
            this.gyrosTrustability = abs(anglesSEAtoU(this.snakeData, p.Results.gyrosTrustability));
        end
        
        end
        
        function update (this, fbk)
            if ~isempty(fbk)
                fbk = this.removeGyroOffset(fbk);
                fbk = this.removeAccelOffset(fbk);
%                 fbk = fbkSEAtoU(this.snakeData,fbk);
                
                if this.firstRun
                    this.previousTime = fbk.time;
                    this.firstRun = false;
                end
                    
                if (fbk.time-this.previousTime)>0.01
                    updateFilter(this, fbk);
                    this.everUpdated = true;
                end
            end
        end
        
        function R = getBodyPose(this)
        %getBodyPose
        
        R = this.R;
            
        end
        
        function Vg = getAccelVec(this)
        %getAccelVec
        
        Vg = this.accelGrav;
            
        end
    end
        
    methods(Access = private, Hidden = true)
        
        function updateFilter(this, fbk)
            %%%%%%%%%
            % SETUP %
            %%%%%%%%%    
            
            % Weight on accelerometer correction term
            % Empirically set value
            accelWeight = .5;
            
            dt = fbk.time - this.previousTime;
            dt = max( min( dt, 1 ), .01 );
            
            %%%%%%%%%%%%%%%%%%
            % ACCELEROMETERS %
            %%%%%%%%%%%%%%%%%%
            
            accelVecModule = [ fbk.accelX; fbk.accelY; fbk.accelZ ];
            
            % Rotate accelerometer vectors into the body frame
            accelVecBody = zeros(size(accelVecModule));
            
            for i=1:this.snakeData.num_modules
                accelVecBody(:,i) = this.snakeData.snakeShape(1:3,1:3,i+1) * accelVecModule(:,i);
            end
            % Average accelerometers
            accelVecAvg = mean( accelVecBody, 2 );
            %%%%%%%%%
            % GYROS %
            %%%%%%%%%
            
            gyroVecModule = [ fbk.gyroX; fbk.gyroY; fbk.gyroZ];
            
            % Rotate gyros into the body frame, taking into account joint angle
            % velocities.
            gyroVecBody = zeros(size(gyroVecModule));
            for i=1:this.snakeData.num_modules
                gyroVecBody(:,i) = this.snakeData.snakeShape(1:3,1:3,i+1) * gyroVecModule(:,i);
            end
            
            % Average gyros
            gyroVecAvg = mean( gyroVecBody(:,this.gyrosTrustability>0), 2 );
            %TODO REMOVE HACK
            
%             display([gyroVecBody; accelVecBody]);
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % CALCULATE THE ORIENTATION %
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % ACCELEROMETER GRAVITY VECTOR
            gravityVec = accelVecAvg / norm(accelVecAvg);
            upVec = [0; 0; 1];
            
            if (this.everUpdated == false)
                
                  accelAxis = cross( upVec, gravityVec );
                  accelAxis = accelAxis / norm(accelAxis);

                  accelAngle = rad2deg( acos( dot(upVec,gravityVec) ) );

                  this.q = real(SpinCalc( 'EVtoQ', ...
                             [accelAxis', accelAngle], ...
                             1E6, 0 ))';
            end
            
            % ESTIMATE NEW ORIENTATION BY FORWARD INTEGRATING GYROS
            w_x = gyroVecAvg(1);
            w_y = gyroVecAvg(2);
            w_z = gyroVecAvg(3);

            q_from_gyros = quat_rotate( w_x, w_y, w_z, this.q, dt );    
            orientDCM = SpinCalc('QtoDCM',  q_from_gyros', 1E6, 0 )';
            
            %gravityVec
            accelGravity = orientDCM' * gravityVec;
            this.accelGrav = accelGravity;

            accelAxis = cross( upVec, accelGravity );
            if norm(accelAxis) ~= 0
                accelAxis = accelAxis / norm(accelAxis);
                accelAngle = rad2deg( acos( dot(upVec,accelGravity) ) );

                % MESS W/ THE ACCEL WEIGHT

                % Scale down if gyro readings are large
                gyroMag = norm(gyroVecAvg);
                gyroScale = 1;

                accelWeight = accelWeight / (1 + gyroScale * gyroMag);

                % Scale down if accelerometers deviate from 1g.
                accelMag = norm(accelVecAvg);
                accelThresh = 1; %0.1

                accelDev = abs(accelMag - 9.81) > accelThresh;

                if accelDev
                    accelWeight = 0;
                else
                    accelWeight = accelWeight * (1 - accelDev/accelThresh);
                end

                R_error = real(SpinCalc( 'EVtoDCM', ...
                              [-accelAxis', accelWeight * accelAngle], ...
                               1E6, 0 ));
                updatedDCM = R_error' * orientDCM';
            else
                updatedDCM = orientDCM';
            end
            
            this.R = updatedDCM;
            this.q = real(SpinCalc( 'DCMtoQ', ...
                                 updatedDCM, ...
                                 1E6, 0 ))';
            this.q = this.q / norm(this.q);
            this.previousTime = fbk.time;
        end
        
        function resetCoordinates(this)
            % resets the frame of reference of the pose
            this.R = eye(3);
        end
        
        function fbkFixed = removeGyroOffset(this, fbk)

            fbkFixed.gyroX = fbk.gyroX - this.gyroOffset(1,:);
            fbkFixed.gyroY = fbk.gyroY - this.gyroOffset(2,:);
            fbkFixed.gyroZ = fbk.gyroZ - this.gyroOffset(3,:);
            
            fbkFixed.accelX = fbk.accelX;
            fbkFixed.accelY = fbk.accelY;          
            fbkFixed.accelZ = fbk.accelZ; 
            
            fbkFixed.time = fbk.time;
        end
        
        function fbkFixed = removeAccelOffset(this, fbk)

            fbkFixed.gyroX = fbk.gyroX;
            fbkFixed.gyroY = fbk.gyroY;
            fbkFixed.gyroZ = fbk.gyroZ;
            
            fbkFixed.accelX = fbk.accelX - this.accelOffset(1,:);
            fbkFixed.accelY = fbk.accelY - this.accelOffset(2,:);          
            fbkFixed.accelZ = fbk.accelZ - this.accelOffset(3,:); 
            
            fbkFixed.time = fbk.time;
        end    
    end
    
    properties(Access = private, Hidden = true) 
        accelGrav;
        mexCode;
        snakeData;
        firstRun;
        everUpdated;                
        miscData;
        previousTime;      
        q;
        R;        
        snakeShape;         %modules in the head frame
        gyroOffset;
        accelOffset;
        gyrosTrustability;
    end
        
end
