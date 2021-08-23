//==================================================================
// Programa: Modelo mátemático de um quadrirrotor pela formulação de 
//           Newton-Euler e controle P.I.D.
// Autor: Diego Souza
// Data: 03/09/2015
//==================================================================

clear();        // Limpa variáveis
clc;            // Limpa terminal de comando
xdel(winsid()); // Fecha todos os gráficos abertos previamente

disp("-- MODELAGEM DE UM QUADRIRROTOR - TRAJETÓRIA - NEWTON-EULER - CONTROLE P.I.D. --")
disp("Simulação iniciada com sucesso em:")
disp(clock());
disp("Simulando... Aguarde...")
tic();          // Inicia contador

// Diretório de trabalho
diretorio = (get_absolute_file_path('MODELO_QUADCOPTER_TRAJETORIA_CONTROLE_NEWTON.sce'))

//*************************************
// VARIÁVEIS
//*************************************

g = 9.81;       // Aceleração da gravidade
m = 0.468;      // Massa do quadrirrotor
l = 0.225;      // Distância rotor x centro de massa
b = 1.140e-7;   // Coeficiente de arrasto da estrutura
k = 2.980e-6;   // Coeficiente de empuxo das hélices
ir = 3.357e-5;  // Momento de inércia do rotor
ixx = 4.856e-3; // Momento de inércia em x
iyy = 4.856e-3; // Momento de inércia em y
izz = 8.801e-3; // Momento de inércia em z
ax = 0.25;      // Coeficiente de força de arrasto em x
ay = 0.25;      // Coeficiente de força de arrasto em y
az = 0.25;      // Coeficiente de força de arrasto em z

//*************************************
// CRIAÇÃO DO JOUNCE
//*************************************

pos_final_x = 1; // Posição final desejada em x [m]
a_jounce_x = pos_final_x / 0.66666;
b_jounce_x = 0.5;

pos_final_y = 2; // Posição final desejada em y [m]
a_jounce_y = pos_final_y / 0.66666;
b_jounce_y = 0.5;

pos_final_z = 3; // Posição final desejada em z [m]
a_jounce_z = pos_final_z / 0.66666;
b_jounce_z = 0.5;

disp("Posição final desejada em x [m]:")
disp(pos_final_x)
disp("Posição final desejada em y [m]:")
disp(pos_final_y)
disp("Posição final desejada em z [m]:")
disp(pos_final_z)

//*************************************
// PARÂMETROS P.I.D.
//*************************************
//  P    D   DD (sugeridos no paper)
// 1.85 0.75 1
// 8.55 6.25 1
// 1.85 0.75 1
// 3    0.75 0
// 3    0.75 0
// 3    0.75 0

kp_x=0.2;    // Ganho kp do eixo z
ki_x=0;      // Ganho ki do eixo z
kd_x=0.2;    // Ganho kd do eixo z
kdd_x=0;     // Ganho kd do eixo z

kp_y=2;      // Ganho kp do eixo z
ki_y=0;      // Ganho ki do eixo z
kd_y=1;      // Ganho kd do eixo z
kdd_y=0;     // Ganho kd do eixo z

kp_z=2;      // Ganho kp do eixo z
ki_z=0;      // Ganho ki do eixo z
kd_z=1;      // Ganho kd do eixo z
kdd_z=0;     // Ganho kd do eixo z

kp_phi=3;    // Ganho kp do angulo de rolagem
ki_phi=0;    // Ganho ki do angulo de rolagem
kd_phi=0;    // Ganho kd do angulo de rolagem
kdd_phi=0;   // Ganho kdd do angulo de rolagem

kp_theta=3;  // Ganho kp do angulo de arfagem
ki_theta=0;  // Ganho ki do angulo de arfagem
kd_theta=0;  // Ganho kd do angulo de arfagem
kdd_theta=0; // Ganho kdd do angulo de arfagem

kp_psi=3;    // Ganho kp do angulo de guinada
ki_psi=0;    // Ganho ki do angulo de guinada
kd_psi=0;    // Ganho kd do angulo de guinada
kdd_psi=0;   // Ganho kdd do angulo de arfagem

//*************************************
// EXECUÇÃO DO DIAGRAMA DE BLOCOS
//*************************************

importXcosDiagram(diretorio + "MODELO_QUADCOPTER_TRAJETORIA_CONTROLE_NEWTON.zcos") // Importa diagrama
typeof(scs_m)                                                                      // Tipo do objeto
xcos_simulate(scs_m,4)                                                             // Realiza a simulação
xdel(winsid());                                                                    // Fecha gráficos da simulação

//*************************************4
// PLOTAGEM DOS GRÁFICOS
//*************************************

// Plot de Entradas de Controle
subplot(221)
plot(w1.time,w1.values,'g-')
plot(w2.time,w2.values,'r--')
plot(w3.time,w3.values,'b:')
plot(w4.time,w4.values,'k-.')

title("Entradas de controle ωi","color","blue","fontsize",3, "fontname",8);
xlabel("Tempo t (s)","fontsize",2,"color","blue");
ylabel("Ent. de controle ωi (rad/s)","fontsize",3,"color","blue");
legend(['ω1';'ω2';'ω3';'ω4'],opt=1); 
set(gca(),"grid",[4 4]); 

// Plot de Ângulos
subplot(222)
plot(PHI_T.time,PHI_T.values,'b-')
plot(THETA_T.time,THETA_T.values,'r-')
plot(PSI_T.time,PSI_T.values,'g-')

title("Ângulos de ‎Ø (Rolagem), Ɵ (Arfagem) e Ψ (Guinada)","color","blue","fontsize",3, "fontname",8);
xlabel("Tempo t (s)","fontsize",2,"color","blue");
ylabel("Ângulo (graus)","fontsize",3,"color","blue");
legend(['Ø (Rolagem)';'Ɵ (Arfagem)';'Ψ (Guinada)'],opt=4); 
set(gca(),"grid",[4 4]); 

// Plot de Posições 2D
subplot(223)
plot(X_T.time,X_T.values,'g-')
plot(Y_T.time,Y_T.values,'r-')
plot(Z_T.time,Z_T.values,'b-')
plot(POS_X.time,POS_X.values,'g-.')
plot(POS_Y.time,POS_Y.values,'r-.')
plot(POS_Z.time,POS_Z.values,'b-.')

title("Posições x, y, e z","color","blue","fontsize",3, "fontname",8);
xlabel("Tempo t (s)","fontsize",2,"color","blue");
ylabel("Posição (m)","fontsize",3,"color","blue");
legend(['Posição x';'Posição y';'Posição z'],opt=4); 
set(gca(),"grid",[4 4]); 

// Plot de Posições 3D
subplot(224)
param3d(X_T.values, Y_T.values,Z_T.values,theta=280,alpha=80,leg="X@Y@Z",flag=[2,4]);
title("Posições x, y, e z","color","blue","fontsize",3, "fontname",8);
xlabel("Posição x (m)","fontsize",2,"color","blue");
ylabel("Posição y (m)","fontsize",3,"color","blue");
zlabel("Posição z (m)","fontsize",3,"color","blue");
set(gca(),"grid",[4 4]); 
e=gce() //the handle on the 3D polyline
e.foreground=color('blue');
set(gca(),"grid",[4 4]); 

//*************************************
// ESCREVENDO EM UM ARQUIVO DE SAÍDA
//*************************************

// Carrega função de concatenação
exec (diretorio + "fprintfMatAppend.sci");

// Diretório e arquivo de escrita
arquivo = "MODELO_QUADCOPTER_TRAJETORIA_CONTROLE_NEWTON.txt";

// Deleta o arquivo se ele já existir
deletefile(diretorio + arquivo);

// Escreve o arquivo
fprintfMatAppend(diretorio + arquivo, w1.time,"[TEMPO]")
fprintfMatAppend(diretorio + arquivo, w1.values,"[W1]")
fprintfMatAppend(diretorio + arquivo, w2.values,"[W2]")
fprintfMatAppend(diretorio + arquivo, w3.values,"[W3]")
fprintfMatAppend(diretorio + arquivo, w4.values,"[W4]")
fprintfMatAppend(diretorio + arquivo, PHI_T.values,"[PHI]")
fprintfMatAppend(diretorio + arquivo, THETA_T.values,"[THETA]")
fprintfMatAppend(diretorio + arquivo, PSI_T.values,"[PSI]")
fprintfMatAppend(diretorio + arquivo, X_T.values,"[X]")
fprintfMatAppend(diretorio + arquivo, Y_T.values,"[Y]")
fprintfMatAppend(diretorio + arquivo, Z_T.values,"[Z]")
fprintfMatAppend(diretorio + arquivo, POS_X.values,"[X]")
fprintfMatAppend(diretorio + arquivo, POS_Y.values,"[Y]")
fprintfMatAppend(diretorio + arquivo, POS_Z.values,"[Z]")

// Finaliza a simulação
disp("Simulação finalizada com sucesso em:")
disp(clock());
disp("Tempo necessário para execução (s): ")
disp(toc())

