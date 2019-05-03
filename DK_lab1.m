% Robotica - Direct Kinematic
% Pedro Fareleira 79074
% David Ribeiro 84027
% Pedro Custodio 84169

clear
%% Values to set
angulos = [0 0 0 0 0 0]; % em graus

%% Program
% Parametros
P_6 = [0; 0; 0; 1]; % ponto segundo o referencial da ponta do braço
teta = angulos*sym(pi)/180; % angulos nos graus de liberdade

% identificação dos graus de liberdade
number_joints = 6;
number_auxiliar_joints = 7;
total_joints = number_auxiliar_joints + number_joints;

% matrix com o, d, a e alpha (pela ordem dada)
data_matrix = [teta(1)  -sym(pi)/2  0  teta(2)  0  -teta(3)-sym(pi)/2    0  teta(4)-sym(pi)/2  -sym(pi)/2  0  teta(5)    0   teta(6);
               0        99          0  0        0  0                     0  -40                195         0  0          0   10; 
               0        40          0  -120     0  20                    0  0                  0           0  0          0   0;
               0        -sym(pi)/2  0  0        0  -sym(pi)/2            0  0                  -sym(pi)/2  0  sym(pi)/2  0   0];

T = zeros(4, 4);

% calculo das matrizes de transformação
for index=1:total_joints
    
    if index == 1
        a_1 = 0;
        alpha_1 = 0;
    else
        a_1 = data_matrix(3,index-1);
        alpha_1 = data_matrix(4,index-1);
    end
    
    o = data_matrix(1,index);
    d = data_matrix(2,index);

    T_zd = [1 0 0 0;
            0 1 0 0;
            0 0 1 d;
            0 0 0 1];

    T_zo = [cos(o)  -sin(o)  0  0;
            sin(o)  cos(o)   0  0;
            0       0        1  0;
            0       0        0  1];

    T_xa = [1 0 0 a_1;
            0 1 0 0;
            0 0 1 0;
            0 0 0 1];

    T_xalpha = [1  0             0              0;
                0  cos(alpha_1)  -sin(alpha_1)  0;
                0  sin(alpha_1)  cos(alpha_1)   0;
                0  0             0              1];

    T(:,:,index) = T_zd*T_zo*T_xa*T_xalpha;
    
end

% matriz de transformação do sistema
T_ = eye(4);
for index=1:total_joints
    T_ = T_*T(:,:,index);
end
% cordenadas do ponto segundo a origem
P_0 = T_*P_6;

P_0(4) = []; % eleminar o 1 da quarta posição

% Find the euler angles
rotation_matrix = [T_(1,1) T_(1,2) T_(1,3); T_(2,1) T_(2,2) T_(2,3); T_(3,1) T_(3,2) T_(3,3)];
euler = rotm2eul(rotation_matrix,'ZYZ')*180/pi;

% imprimir os resultados
disp(['Cordenadas do ponto: ', num2str(P_0(1)),', ', num2str(P_0(2)),', ', num2str(P_0(3))])
disp(['Angulos de Euler: ', num2str(euler(1)),', ', num2str(euler(2)),', ', num2str(euler(3))])