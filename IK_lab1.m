% Robotica - Inverse Kinematic
% Pedro Fareleira 79074
% David Ribeiro 84027
% Pedro Custodio 84169

clear
%% Values to set
Ponto = [0 -85 119]; % em milimetros
angulos = [90 -90 -180]; % em graus
N = 0; % numero de casas decimais

%% Program
% Parametros
% identificação dos graus de liberdade
number_joints = 6;
number_auxiliar_joints = 7;
total_joints = number_auxiliar_joints + number_joints;

% translação
P_0 = [transpose(Ponto); 1];
P_6 = [0; 0; 0; 1]; % ponto segundo o referencial da ponta do braço

% rotação dos eixos
euler_angles = angulos*sym(pi)/180;
r11 = cos(euler_angles(1))*cos(euler_angles(2))*cos(euler_angles(3)) - sin(euler_angles(1))*sin(euler_angles(3));
r12 = -cos(euler_angles(1))*cos(euler_angles(2))*sin(euler_angles(3)) - sin(euler_angles(1))*cos(euler_angles(3));
r13 = cos(euler_angles(1))*sin(euler_angles(2));
r21 = sin(euler_angles(1))*cos(euler_angles(2))*cos(euler_angles(3)) + cos(euler_angles(1))*sin(euler_angles(3));
r22 = -sin(euler_angles(1))*cos(euler_angles(2))*sin(euler_angles(3)) + cos(euler_angles(1))*cos(euler_angles(3));
r23 = sin(euler_angles(1))*sin(euler_angles(2));
r31 = -sin(euler_angles(2))*cos(euler_angles(3));
r32 = sin(euler_angles(2))*sin(euler_angles(3));
r33 = cos(euler_angles(2));

rotation_matrix = [r11 r12 r13; r21 r22 r23; r31 r32 r33];

% matrix transformação do end-effector para a base
T_0_6 = [rotation_matrix; 0 0 0];
T_0_6 = [T_0_6 P_0];

% origem do referencial 5
P_5 = T_0_6*[0; 0; -10; 1];

x = P_5(1);
y = P_5(2);
z = P_5(3);

% matrix com os angulos encontrados (maximo de 8 combinações)
teta = zeros(8,6);

% angulo 1 (dois resultados possiveis)
teta(1:4,1) = atan2(y,x)+sym(pi)/2;
teta(5:8,1) = atan2(y,x)-sym(pi)/2;

% coordenada criada a partir de x e y
lambda = round(sqrt( x^2 + y^2 )*10^95)/10^95;

% angulos 2 e 3 (duas hipoteses cada)
% para primeiro angulo 1
syms o2 o3
eqns = [99 + 120*sin(o2) + 20*sin(sym(pi/2)+o3-o2) + 155*sin(o3-o2) - z == 0, 40 - 120*cos(o2) + 20*cos(sym(pi/2)+o3-o2) + 155*cos(o3-o2) - lambda == 0];
vars = [o2 o3];
[solo2, solo3] = solve(eqns, vars);
% angulos nos graus de liberdade
teta(1:2,2) = solo2(1);
teta(1:2,3) = solo3(1);
teta(3:4,2) = solo2(2);
teta(3:4,3) = solo3(2);

% para segundo angulo 1
syms o2 o3
eqns = [99 + 120*sin(o2) + 20*sin(sym(pi/2)+o3-o2) + 155*sin(o3-o2) - z == 0, 40 - 120*cos(o2) + 20*cos(sym(pi/2)+o3-o2) + 155*cos(o3-o2) + lambda == 0];
vars = [o2 o3];
[solo2, solo3] = solve(eqns, vars);
% angulos nos graus de liberdade
teta(5:6,2) = solo2(1);
teta(5:6,3) = solo3(1);
teta(7:8,2) = solo2(2);
teta(7:8,3) = solo3(2);

% angulos 4, 5 e 6 (metodo algebrico, duas hipoteses)
euler_ = zeros(8,3);
P_0_ = zeros(4,8);
T = zeros(4, 4);
for i=1:8
    if imag(teta(i,2))~=0 || imag(teta(i,3))~=0
        teta(i,:) = NaN;
        continue
    end
    % angulo 5
    T_1_0 = [cos(teta(i,1))   sin(teta(i,1))  0  0;
             -sin(teta(i,1))  cos(teta(i,1))  0  0;
             0                0               1  0;
             0                0               0  1];

    T_2_1 = [0  -cos(teta(i,2))  -sin(teta(i,2))  99*sin(teta(i,2))-40*cos(teta(i,2));
             0  sin(teta(i,2))   -cos(teta(i,2))  40*sin(teta(i,2))+99*cos(teta(i,2));
             1  0                0                0;
             0  0                0                1];

    T_3_2 = [-sin(teta(i,3))  -cos(teta(i,3))  0  -120*sin(teta(i,3));
             cos(teta(i,3))   -sin(teta(i,3))  0  120*cos(teta(i,3));
             0                0                1  0;
             0                0                0  1];

    T_2_0 = T_2_1*T_1_0;
    T_3_0 = T_3_2*T_2_0;

    T_2_6 = T_2_0*T_0_6;
    T_3_6 = T_3_0*T_0_6;

    if mod(i,2)==0
        teta(i,5) = acos(T_3_6(2,3));
    else
        teta(i,5) = -acos(T_3_6(2,3));
    end
    
    % angulo 4
    signal = sin(teta(i,5))/abs(sin(teta(i,5)));
    if abs(signal) == 1
        teta(i,4) = atan2(T_2_6(3,3)*signal, (T_2_6(1,3)*sin(teta(i,3)) + T_2_6(2,3)*cos(teta(i,3)))*signal );
    else
        % singularidade (teta4 tem mesmo efeito de teta6)
        teta(i,4) = 0;
    end

    % angulo 6
    T_4_3 = [sin(teta(i,4))  0  cos(teta(i,4))   -20*sin(teta(i,4));
             cos(teta(i,4))  0  -sin(teta(i,4))  -20*cos(teta(i,4));
             0               1  0                40;
             0               0  0                1];

    T_5_4 = [0  -cos(teta(i,5))  -sin(teta(i,5))  195*sin(teta(i,5));
             0  sin(teta(i,5))   -cos(teta(i,5))  195*cos(teta(i,5));
             1  0                0                0;
             0  0                0                1];

    T_4_0 = T_4_3*T_3_0;
    T_5_0 = T_5_4*T_4_0;

    T_4_6 = T_4_0*T_0_6;
    T_5_6 = T_5_0*T_0_6;

    teta(i,6) = atan2(T_5_6(3,1), T_5_6(3,2));
end

% passar para graus
teta_ = round(teta*180/pi*10^N)/10^N;

% Impressão do resultado
for i=1:8
    if isnan(teta(i,1))
        continue
    end
    disp(['Combinação de angulos ', num2str(i),' do ponto:  ', num2str(teta_(i,1)),',  ', num2str(teta_(i,2)),',  ', num2str(teta_(i,3)), ',  ', num2str(teta_(i,4)), ',  ', num2str(teta_(i,5)), ',  ', num2str(teta_(i,6))])
end



%% confirmar inversa
for i=1:8
    % matrix com o, d, a e alpha (pela ordem dada)
    data_matrix = [teta(i,1)  -sym(pi)/2  0  teta(i,2)  0  -teta(i,3)-sym(pi)/2    0  teta(i,4)-sym(pi)/2  -sym(pi)/2  0  teta(i,5)    0   teta(i,6);
                   0          99          0  0          0  0                       0  -40                  195         0  0            0   10; 
                   0          40          0  -120       0  20                      0  0                    0           0  0            0   0;
                   0          -sym(pi)/2  0  0          0  -sym(pi)/2              0  0                    -sym(pi)/2  0  sym(pi)/2    0   0];

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

        T_zo = [cos(o) -sin(o) 0 0;
                sin(o) cos(o)  0 0;
                0       0        1 0;
                0       0        0 1];

        T_xa = [1 0 0 a_1;
                0 1 0 0;
                0 0 1 0;
                0 0 0 1];

        T_xalpha = [1 0           0            0;
                    0 cos(alpha_1) -sin(alpha_1) 0;
                    0 sin(alpha_1) cos(alpha_1)  0;
                    0 0           0            1];

        T(:,:,index) = T_zd*T_zo*T_xa*T_xalpha;

    end

    % matriz de transformação do sistema
    T_ = eye(4);
    for index=1:total_joints
        T_ = T_*T(:,:,index);
    end
    % cordenadas do ponto segundo a origem
    P_0_(:,i) = T_*P_6;

    rotation_matrix_ = [T_(1,1) T_(1,2) T_(1,3); T_(2,1) T_(2,2) T_(2,3); T_(3,1) T_(3,2) T_(3,3)];

    euler_(i,:) = rotm2eul(rotation_matrix_,'ZYZ')*180/pi;
end