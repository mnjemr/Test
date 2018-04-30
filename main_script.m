clear all

% Global variables 
global we wo In Gta kmg I mmax kw md_monitor t_monitor

% Intertia tensor
I = diag([0.33, 0.37, 0.35]);

mmax=2.0; %Maximum magnetic moment (A*m^2)
rc=7021; %Orbit radius (km)
Mt=7.8379e6; %Mangetic moment Earth T*km^3
Torb=5855; %Orbital period (s)
Gta=11.44*pi/180; %Geomagnetic tilt angle (rad)
In=65*pi/180; %Inclination (rad)
kmg=Mt/rc^3; %Dipole Magnitude (T)
wo=2*pi/Torb; %Orbital speed (rad/s) (mean motion? Assuming circular orbit)
we=7.2921150e-5; %Earth rotation speed (rad/s)

%GAIN CALCULATION
Beta1=0;
nim=atan2(-sin(Gta)*sin(Beta1),...
sin(In)*cos(Gta)-cos(In)*sin(Gta)*cos(Beta1));
if sin(nim)==0;
    sinepsm=sin(In)*cos(Gta)-cos(In)*sin(Gta)*cos(Beta1)/cos(nim);
else
    sinepsm=-sin(Gta)*sin(Beta1)/sin(nim);
end

% Control gain
kw=2*wo*(1+sinepsm)*min(diag(I));

% Initial conditions
q0=[-0.062;0.925;-0.007;0.375];
w0=[0.604;-0.760;-0.384];

states=[q0; w0];

% End time
EndTime = 10000;
tspan = [0 EndTime];

% System simulation
[t, y] = ode45 (@attitude_dynamics, tspan, states);

% Plotting

% Quaternions (Can we extract the angles from here?)
figure(1)
subplot(2,2,1)
plot(t,y(:,1))
ylabel('q1')
grid on
subplot(2,2,2)
plot(t,y(:,2))
ylabel('q2')
grid on
subplot(2,2,3)
plot(t,y(:,3))
ylabel('q3')
grid on
subplot(2,2,4)
plot(t,y(:,4))
ylabel('q4')
grid on

% Convert quaternions to angles
[azimuth,pitch,bank] = Quat2Eul(y(:,4),y(:,1),y(:,2),y(:,3));
%[euler] = Quat2Eul(y(:,1:4))


figure(2)
ylabel('Angles')
subplot(3,1,1)
plot(t,azimuth * 180 / pi)
xlabel('Time (s)')
ylabel('Azimuth (deg)')
grid on
subplot(3,1,2)
plot(t,pitch * 180 / pi)
xlabel('Time (s)')
ylabel('Pitch (deg)')
grid on
subplot(3,1,3)
plot(t,bank * 180 / pi)
xlabel('Time (s)')
ylabel('Roll (deg)')
grid on


% Angular velocities
figure(3)
subplot(3,1,1)
plot(t,y(:,5) * 180 / pi)
xlabel('Time (s)')
ylabel('Wx (deg/s)')
grid on
subplot(3,1,2)
plot(t,y(:,6) * 180 / pi)
xlabel('Time (s)')
ylabel('Wy (deg/s)')
grid on
subplot(3,1,3)
plot(t,y(:,7) * 180 / pi)
xlabel('Time (s)')
ylabel('Wy (deg/s)')
grid on

% Control dipoles
figure(4)
subplot(3,1,1)
plot(t_monitor,md_monitor(1,:))
xlabel('Time (s)')
ylabel('Mx (Am^2)')
grid on
subplot(3,1,2)
plot(t_monitor,md_monitor(2,:))
xlabel('Time (s)')
ylabel('My (Am^2)')
grid on
subplot(3,1,3)
plot(t_monitor,md_monitor(3,:))
xlabel('Time (s)')
ylabel('Mz (Am^2)')
grid on