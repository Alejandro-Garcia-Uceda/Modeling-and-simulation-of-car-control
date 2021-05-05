%ENTREGA 2 TFG

clear all
%Open the created world and import the car %%%%%%%%%%%%%%%%%%%%%
world=vrworld('cocheH.WRL');
open(world)
fig=view(world,'-internal');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%% CAR CONSTANTS %%%%%

Dt=0.7; % Distance between the rear axle of the car and the center of mass
Dd=0.7; % Distance between the front axle of the car and the center of mass 
D=Dt+Dd; % distances between the axles of the car

M=1; %car mass
Md=M/((Dd/Dt)+1); % mass of the car on the front axle
Mt=M-Md; % mass of the car on the rear axle

Af=2.73; % front surface
Al=5.89; % lateral surface

Rtd=[0.7,-0.75];Rti=[-0.7,-0.75];Rdd=[0.7,0.75];Rdi=[-0.7,0.75]; # The position of the wheels in relation to the car
%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%% SUPERFICE CONSTANTS %%%%%%%%%%%%%%%%%

vmaxv=3;
roC=0.1;  % roC=density of the air by the friction coefficient

Crest1=100; % Static friction coefficients surface 1
Crdin1=100; % Dynamic friction coefficients surface 1
Crest2=0.0; % Static friction coefficients surface 2
Crdin2=0.0; % Dynamic friction coefficients surface 2
Cr=[Crest1,Crdin1;Crest2,Crdin2]; % Friction coefficients matrix

sup=1;  % High friction zone 1, or low friction zone 2
dind=1; % Dynamic condition of front wheels, skidding or not
dint=1; % Dynamic condition of rear wheels, skidding or not

altroz=1; % High friction
bajroz=2; % Low friction
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

joy=vrjoystick(1);  %Joystick which we will call 1%

%%%%%%% INITIAL DATA %%%%%%%%%%%%%%%%%

ang1=0;ang2=0;ang3=0;       % Initial angles
ang2max=pi/4;ang2min=-pi/4; % Max and min wheels angle
dang1=0;
ANG2(1)=ang2                %rad

vz1=0;vx1d=0;vx1t=0;vy1=0;                                     % Initial speed 
vmax=10;vmin=-3;                                               % Maximum and minimum engine speed
Vz1(1)=vz1;Vx1d(1)=vx1d;Vx1t(1)=vx1t;Vy1(1)=vy1;dr=0;DR(1)=dr; %m/s
x=0;y=0;z=90;X(1)=x;Z(1)=z;Y(1)=y;                             %Initial position
dt=0.05;t=0;T(1)=t;                                            % Time seg
F=0;
Con=1;      % Counter
g=-9.8;     % Gravitational acceleration m/s^-2
estadod=1;  % The static or dynamic coefficient of friction should be used
estadot=1;

while Con<=300  % This while is the time.
               
    pause(0.05)
    %%%%%%%%%%%%%%%%%%%%%%% CAR MOVEMENT %%%%%%%%%%%%%%%%%%%%%%%%%
    a=axis(joy,[1 2]); #Joystick signal
    
    t=t+dt;
    Con=Con+1;T(Con)=t;
    % Front wheel angle %%%%%%%%%%%
    if round(a(1))~=0
        ang2n=ang2-0.025*round(a(1));
    else 
        if ang2<-pi/20
            ang2n=ang2+0.01*abs(vz1*ang2/vmax/ang2max); % Wheels regain their position
        elseif ang2>pi/20
            ang2n=ang2-0.01*abs(vz1*ang2/vmax/ang2max);
        else
            ang2n=0;
        end
    end
    
    if ang2n<ang2min
            ang2n=ang2min
    elseif ang2n>ang2max
            ang2n=ang2max
    end
    ang2=ang2n;
    ANG2(Con)=ang2;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %% ACCELERATIONS %%
    
    % gravitational acceleration

    agx=-g*sin(ang3)*sin(ang1);
    agz=g*sin(ang3)*cos(ang1);
    agy=g*cos(ang3);
   
    % engine acceleration, rear-wheel drive (can be changed to front wheel drive )
    if vz1<=vmin & round(a(2))>0
        amot=0;
    elseif vz1>=vmax & round(a(2))<0
            amot=0;
    else
            amot=-3*round(a(2));
    end
    
    
    % Air friction
    if x>=-50 % if the car is far to this line,
        
        vvx=-10; % Wind speed
    else
        vvx=0;
    end
    vvxz=vvx*cos(ang1);vvxx=-vvx*sin(ang1); % Wind speed car reference

    if (vx1d+vvxx)<=0
        sigd=-1;
    else 
        sigd=1;
    end
    
    if (vz1+vvxz)<=0
        sig2=-1;
    else 
        sig2=1;
    end
    
    arax=0.5*roC*sig1*((Af*abs(sin(ang1))+Al*abs(cos(ang1)))*(dr+vvxx)^2)/M;   % acceleration of air friction x-axis 
    araz=-0.5*roC*sig2*((Af*abs(cos(ang1))+Al*abs(sin(ang1)))*(vz1+vvxz)^2)/M; % acceleration of air friction z-axis 

    %% SUM OF ACCELERATIONS (without wheel friction)
    Axd=arax+agx;     % Acceleration at the front axle x-axis 
    Axt=agx+arax;     % Acceleration at the rear axle x-axis 
    Az=agz+araz+amot; % Acceleration z-axis
    Ay=agy;           % Acceleration y-axis
    
    % Ground friction acceleration
    if x>=50             %This zone has a different function by definition
        sup=2;
    elseif x>=0 & z>100  %This zone has a different function by definition
        sup=2;
    else
        sup=1;
    end
    Arozxt=-Mt*g*Cr(sup,dint);
    Arozxd=-Md*g*Cr(sup,dind)*cos(ang2);
    Arozz=-Md*g*Cr(sup,dind)*sin(ang2);
    Arozd=-Md*g*Cr(sup,dind);
    
   
    %%%%%%%%%%%%%END OF ACCELERATIONS%%%%%%%%
   
    %%Body movement%%%%%%
    vx1dn=vx1d+Axd*dt
    
    vx1tn=vx1t+Axt*dt
    
    vz1n=vz1+Az*dt
    
    
    
    %%%%%  With the speeds, we check if any of the wheels skid.
    %% Do the rear wheels skid?
    if dint==1
        if abs(vx1tn)<=abs(Arozxt*dt)
            vx1t=0
            difvz1n=0;
            
        else
            vx1t=vx1tn-Arozxt*dt*vx1tn/sqrt(vx1tn^2+vz1n^2)
            difvz1n=Arozxt*dt*vz1n/sqrt(vx1tn^2+vz1n^2);
            dint=2;
            
        end
    else
        if abs(vx1tn)<=abs(Arozxt*dt)
            vx1t=0
            difvz1n=0;
            dint=1;
            
        else
            vx1t=vx1tn-Arozxt*dt*vx1tn/sqrt(vx1tn^2+vz1n^2)
            difvz1n=(Arozxt*dt*vz1n/sqrt(vx1tn^2+vz1n^2))*vz1n/abs(vz1n);
            
        end
    end
   
    %%% Do the front wheels skid?
   
    if dind==1
        
        if abs(-sin(ang2)*vz1n+cos(ang2)*vx1dn)<=abs(Arozd*dt)
            
            vz1n=(vz1n-difvz1n)
            vd=vz1n*cos(ang2)+vx1dn*sin(ang2);
            vz1=vd*cos(ang2)
            vx1d=vd*sin(ang2)
            
        else
            vx1d=vx1dn-Arozd*dt*vx1dn/sqrt(vx1dn^2+vz1n^2)
            difvz1dn=Arozd*dt*vz1n/sqrt(vx1dn^2+vz1n^2);
            
            if abs(vz1n)>abs(difvz1n+difvz1dn)
                vz1=(vz1n-difvz1n-difvz1dn)
            else
                vz1=0
                
            end
            
            dind=2;
            
        end
        
    else
        
        if abs(-sin(ang2)*vz1n+cos(ang2)*vx1dn)<=abs(Arozd*dt)
            
            vz1n=(vz1n-difvz1n);
            vd=vz1n*cos(ang2)+vx1dn*sin(ang2);
            vz1=vd*cos(ang2);
            vx1d=vd*sin(ang2);
            dind=1;
            
        else
            vx1d=vx1dn-Arozd*dt*vx1dn/sqrt(vx1dn^2+vz1n^2)
            difvz1dn=Arozd*dt*vz1n/sqrt(vx1dn^2+vz1n^2);
            
            if abs(vz1n)>abs(difvz1n+difvz1dn)
                vz1=(vz1n-difvz1n-difvz1dn);
            else
                vz1=0;
            end
                 
        end
        
    end
          
    %->calculate velocities as a function of the above accelerations

    dang1=(vx1d-vx1t)*dt/D;
     if abs(vx1d-vx1t)<1E-12
        dr=vx1d;
        vang=0;
     else
        C=vx1t*D/(vx1d-vx1t);%distance to center of rotation
        dr=(C+Dt)*dang1/dt; %distance  x'' traversing the center of masses 
        vang=(vx1d-vx1t)/D;
     end
   
    % final displacement of the car
    x=x+(dr*cos(ang1)*dt+vz1*sin(ang1)*dt)*cos(ang3);
    X(Con)=x;

    z=z+(-dr*sin(ang1)*dt+vz1*cos(ang1)*dt)*cos(ang3);
    Z(Con)=z;

    y=y+((dr*cos(ang1)*dt+vz1*sin(ang1)*dt)*cos(ang3)+(dr*sin(ang1)*dt+vz1*cos(ang1)*dt))*sin(ang3);
    Y(Con)=y;

    % final speed
    Vx1d(Con)=vx1d;
    Vx1t(Con)=vx1t;
    DR(Con)=dr*dt;
    Vz1(Con)=vz1;

    % final car angle 
    ang1=ang1+dang1;
    
    
    
    %%%%%variation of ang3 as a function of x,z and ang1 on the contour
    %%%%%% FINAL CONTOUR CONDITIONS

    % Coefficient of friction
    if x>50
        sup=2;
    elseif z>=100 & x>=0
        sup=2;
    else
        sup=1;
    end
    
    
    % Slope and floor height
     if z>=100 & z<150
         ang3=5*pi/180;
         yt=(z-100)*tan(ang3);
     elseif z>=150 & z<200
         ang3=10*pi/180;
         yt=(50)*tan(5*pi/180)+(z-150)*tan(ang3);
     elseif z>=200 
         ang3=20*pi/180;
         yt=(50)*(tan(5*pi/180)+tan(10*pi/180))+(z-200)*tan(ang3);
     else
         ang3=0;
         yt=0;
     end
     
     % Height adjustment
     if yt~=y
         y=yt;
     end
   
     
     if z>=100 & abs(x)>48.5
             x=0;z=0;y=0;ang1=0;ang2=0;ang3=0;
             vx1d=0;vx1t=0;vz1=0;dr=0;vy1=0;
     end
    % The car moves in the simulation
    
   world.coche_pend.translation=([x y z]);
   world.cmcoche.rotation=([0 1 0 ang1]);
   world.ruedad.rotation=([0 1 0 ang2]);
   world.ruedai.rotation=([0 1 0 ang2]);
   world.coche_pend.rotation=([-1 0 0 ang3]);

    % Shaft speeds after the process
 
    vang=(vx1d-vx1t)/D;
    %disp(vang);disp(dr);disp(cos(dang1));
    disp(vz1);disp(sin(dang1))
    vx1d=vang*Dd+dr*cos(dang1)*dt-vz1*sin(dang1);
    vx1t=-vang*Dt+dr*cos(dang1)*dt-vz1*sin(dang1);
    vz1=dr*sin(dang1)*dt+vz1*cos(dang1)
    
   disp('---------------------------------------')
   end