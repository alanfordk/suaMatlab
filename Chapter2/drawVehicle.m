function drawVehicle(state)
% process inputs to function
pn       = state(1);       % inertial North position
pe       = state(2);       % inertial East position
pd       = state(3);
u        = state(4);
v        = state(5);
w        = state(6);
phi      = state(7);       % roll angle
theta    = state(8);       % pitch angle
psi      = state(9);       % yaw angle
p        = state(10);       % roll rate
q        = state(11);       % pitch rate
r        = state(12);       % yaw rate
t        = state(13);       % time

% define persistent variables
persistent vehicle_handle;
persistent Vertices
persistent Faces
persistent facecolors
persistent ax

% first time function is called, initialize plot and persistent vars
if t==0
    figure(1);
    clf;
    [Vertices,Faces,facecolors] = defineVehicleBody;
    vehicle_handle = drawVehicleBody(Vertices,Faces,facecolors,...
        pn,pe,pd,phi,theta,psi,...
        []);
    title('Vehicle')
    xlabel('East')
    ylabel('North')
    zlabel('Altitude')
    view(32,47)  % set the view angle for figure
    axis([-20,20,-20,20,0,20]);
    grid on;
    hold on;
    ax = draw_ground_trace(pn, pe, pd);
else
    drawVehicleBody(Vertices,Faces,facecolors,...
        pn,pe,pd,phi,theta,psi,...
        vehicle_handle);
    delete(ax);
    ax = draw_ground_trace(pn, pe, pd);
end
end
%% Draw vehicle body
% return handle if 3rd argument is empty, otherwise use 3rd arg as handle
function handle = drawVehicleBody(V,F,patchcolors,...
    pn,pe,pd,phi,theta,psi,...
    handle)

V = rotate(V, phi, theta, psi);  % rotate vehicle
V = translate(V, pn, pe, pd);  % translate vehicle

% transform vertices from NED to XYZ (for matlab rendering)
R = [...
    0, 1, 0;...
    1, 0, 0;...
    0, 0, -1;...
    ];
V = R*V;

if isempty(handle)
    handle = patch('Vertices', V', 'Faces', F,...
        'FaceVertexCData',patchcolors,...
        'FaceColor','flat');
else
    set(handle,'Vertices',V','Faces',F);
    drawnow
end
end
%% rotate vehicle
function pts=rotate(pts,phi,theta,psi)
% define rotation matrix (right handed)
ypr = [psi,theta,phi]';
R = v2b(ypr);
% note that R above either leaves the vector alone or rotates
% a vector in a left handed rotation.  We want to rotate all
% points in a right handed rotation, so we must transpose
R = R';
% rotate vertices
pts = R*pts;
end
%% Translate vertices by pn, pe, pd
function pts = translate(pts,pn,pe,pd)

pts = pts + repmat([pn;pe;pd],1,size(pts,2));

end
%% defineVehicleBody
function [V,F,facecolors] = defineVehicleBody

fuse_l1 = 1.5;
fuse_l2 = .5;
fuse_l3 = 4;
wing_l = 1.5;
fuse_h = 1;
tail_h = 1;
tailwing_l = 1;
fuse_w = 1;
wing_w = 3;
tailwing_w = 2;

% Define the vertices (physical location of vertices
V = [...
    fuse_l1, 0, 0;...                       % pt 1
    fuse_l2, 0.5*fuse_w, -0.5*fuse_h;...    % pt 2
    fuse_l2, -0.5*fuse_w, -0.5*fuse_h;...   % pt 3
    fuse_l2, -0.5*fuse_w, 0.5*fuse_h;...    % pt 4
    fuse_l2, 0.5*fuse_w, 0.5*fuse_h;...     % pt 5
    -fuse_l3, 0, 0;...                      % pt 6
    0, 0.5*wing_w, 0;...                    % pt 7
    -wing_l, 0.5*wing_w, 0;...              % pt 8
    -wing_l, -0.5*wing_w, 0;...             % pt 9
    0, -0.5*wing_w, 0;...                   % pt 10
    -fuse_l3+tailwing_l, 0.5*tailwing_w, 0;...  % pt 11
    -fuse_l3, 0.5*tailwing_w, 0;...         % pt 12
    -fuse_l3, -0.5*tailwing_w, 0;...        % pt 13
    -fuse_l3+tailwing_l, -0.5*tailwing_w, 0;...  % pt 14
    -fuse_l3+tailwing_l, 0, 0;...           % pt 15
    -fuse_l3, 0, -tail_h;...  % pt 5
    ]';

% define faces as a list of vertices numbered above
F = [...
    1, 2, 3, 1;...      % top nose
    1, 3, 4, 1;...      % left nose
    1, 4, 5, 1;...      % bottom nose
    1, 2, 5, 1;...      % right nose
    3, 4, 6, 3;...      % left body
    2, 3, 6, 2;...      % top body
    4, 5, 6, 4;...      % bottom body
    2, 5, 6, 2;...      % right body
    6, 15, 16, 6;...    % tail fin
    11, 12, 13, 14;...  % tail wing
    7, 8, 9, 10;...     % wing
    ];

% define colors for each face
myred = [1, 0, 0];
mygreen = [0, 1, 0];
myblue = [0, 0, 1];

facecolors = [...
    myred;...       % top nose
    myblue;...      % left nose
    myblue;...      % bottom nose
    myblue;...      % right nose
    myblue;...      % left body
    myblue;...      % top body
    myblue;...      % bottom body
    myblue;...      % right body
    myred;...       % tail fin
    mygreen;...     % tail wing
    mygreen;...     % wing
    ];
end
