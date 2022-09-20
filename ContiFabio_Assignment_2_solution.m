clc
clear
%% Robot struct 

% Number of links
my_r.NL = 2;

% Links type
my_r.type = ["revolute";"revolute"];

% Links ang 

my_r.ang(:,:,1) = eye(3);
my_r.ang(:,:,2) = eye(3);


% Links' length
my_r.len = [1;0.8]; % m

% Links' center of mass 
my_r.cm = [0.5 0 0; 0.4 0 0]; % m

% Links' position wrt the previous frame
my_r.pos = [0 0 0;1 0 0]; % m 

% Links' mass
my_r.m = [22;19]; % kg

% Links' inertia matrices
my_r.I(:,:,1) = diag([0.4 0.4 0.4]); % kg*m*m
my_r.I(:,:,2) = diag([0.3 0.3 0.3]); % kg*m*m


%% configurations

% 1.1
% subtructure of the configuration
my_config.q = [pi/9; 2*pi/9]; % rad, rad
my_config.qd = [0.2;0.15]; % rad/s, rad/s
my_config.qdd = [0.1;0.085];% rad/s*s, rad/s*s

% Phisical structure of the robot combined with the configuration
rob.R = my_r;
rob.C = my_config;

robot{1,1} = rob; 

% 1.2
% subtructure of the configuration
my_config.q = [pi/2; pi/4];
my_config.qd = [-0.8;0.35];
my_config.qdd = [-0.4;0.1];

% Phisical structure of the robot combined with the configuration in a
% unique structure
rob.R = my_r;
rob.C = my_config;

robot{1,2} = rob;


%% 1.1

% Solution w/ Gravity 
F_ext = [0;0;0];
M_ext = [0;0;0];
g = [0 ; -9.81 ; 0];

tau{1,1}(1,:) = NewtEuler(robot{1,1},F_ext,M_ext,g);

% Solution w/o Gravity
F_ext = [0;0;0];
M_ext = [0;0;0];
g = [0 ; 0 ; 0];

tau{1,1}(2,:) = NewtEuler(robot{1,1},F_ext,M_ext, g);

%% 1.2

% Solution w/ Gravity 
F_ext = [0;0;0];
M_ext = [0;0;0];
g = [0 ; -9.81 ; 0];

tau{1,2}(1,:) = NewtEuler(robot{1,2},F_ext,M_ext, g);

% Solution w/o Gravity 
F_ext = [0;0;0];
M_ext = [0;0;0];
g = [0 ; 0 ; 0];

tau{1,2}(2,:) = NewtEuler(robot{1,2},F_ext,M_ext, g);

%% Robot struct ex.2

% Number of links
my_r.NL = 2;

% Links type
my_r.type = ["revolute";"prismatic"];

% Links ang 

my_r.ang(:,:,1) = eye(3);
my_r.ang(:,:,2) = axang2rotm([0 1 0 pi/2]) *  axang2rotm([0 0 1 pi/2]);


% Links' length
my_r.len = [1;0]; % m

% Links' center of mass 
my_r.cm = [0.5 0 0; 0 0 0]; % m

% Links' position wrt the previous frame
my_r.pos = [0 0 0;1 0 0]; % m 

% Links' mass
my_r.m = [10;6]; % kg

% Links' inertia matrices
my_r.I(:,:,1) = diag([0.4 0.4 0.4]) ; % kg*m*m
my_r.I(:,:,2) = diag([0.3 0.3 0.3]) ; % kg*m*m



%% configurations

% 2.1
% subtructure of the configuration
my_config.q = [pi/9; 0.2]; % rad, m
my_config.qd = [0.08;0.03]; % rad/s, m/s
my_config.qdd = [0.1;0.01]; % rad/s*s, m/s*s

% Phisical structure of the robot combined with the configuration
rob.R = my_r;
rob.C = my_config;

robot{2,1} = rob; 

% 2.2
% subtructure of the configuration
my_config.q = [2*pi/3; 0.6];
my_config.qd = [-0.4;-0.08];
my_config.qdd = [-0.1;-0.01];

% Phisical structure of the robot combined with the configuration in a
% unique structure
rob.R = my_r;
rob.C = my_config;

robot{2,2} = rob;

%% 2.1

% Solution w/ Gravity 
F_ext = [0;0;0];
M_ext = [0;0;0];
g = [0 ; -9.81 ; 0];

tau{2,1}(1,:) = NewtEuler(robot{2,1},F_ext,M_ext,g);

% Solution w/o Gravity
F_ext = [0;0;0];
M_ext = [0;0;0];
g = [0 ; 0 ; 0];

tau{2,1}(2,:) = NewtEuler(robot{2,1},F_ext,M_ext, g);

%% 2.2

% Solution w/ Gravity 
F_ext = [0;0;0];
M_ext = [0;0;0];
g = [0 ; -9.81 ; 0];

tau{2,2}(1,:) = NewtEuler(robot{2,2},F_ext,M_ext, g);

% Solution w/o Gravity 
F_ext = [0;0;0];
M_ext = [0;0;0];
g = [0 ; 0 ; 0];

tau{2,2}(2,:) = NewtEuler(robot{2,2},F_ext,M_ext, g);


%% Robot struct ex.3
% Number of links
my_r.NL = 3;

% Links type
my_r.type = ["revolute";"revolute";"revolute"];

% Links ang 

my_r.ang(:,:,1) = eye(3);
my_r.ang(:,:,2) = axang2rotm([1 0 0 pi/2]);
my_r.ang(:,:,3) = eye(3);


% Links' length
my_r.len = [1;0.8;0.35]; % m

% Links' center of mass 
my_r.cm = [0.5 0 0; 0.4 0 0; 0.35/2 0 0]; % m

% Links' position wrt the previous frame
my_r.pos = [0 0 0;1 0 0;0.8 0 0]; % m 

% Links' mass
my_r.m = [20;20;6]; % kg

% Links' inertia matrices
my_r.I(:,:,1) = diag([0.2 0.2 0.8]); % kg*m*m
my_r.I(:,:,2) = diag([0.2 0.2 0.8]); % kg*m*m
my_r.I(:,:,3) = diag([0.08 0.08 0.1]); % kg*m*m



%% configurations

% 3.1
% subtructure of the configuration
my_config.q = [pi/9; 2 * pi/9; pi/18]; % rad, rad, rad
my_config.qd = [0.2;0.15;-0.2]; % rad/s, rad/s, rad/s
my_config.qdd = [0.1;0.085; 0]; % rad/s*s, rad/s*s, rad/s*s

% Phisical structure of the robot combined with the configuration in a
% unique structure
rob.R = my_r;
rob.C = my_config;

robot{3,1} = rob; 

%% 3.1

% Solution w/ Gravity 
F_ext = [0;0;0];
M_ext = [0;0;0];
g = [0 ; 0 ; -9.81];

tau{3,1}(1,:) = NewtEuler(robot{3,1},F_ext,M_ext, g);

% Solution w/o Gravity
F_ext = [0;0;0];
M_ext = [0;0;0];
g = [0 ; 0 ; 0];

tau{3,1}(2,:) = NewtEuler(robot{3,1},F_ext,M_ext, g);






