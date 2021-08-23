//==================================================================
// Programa: Modelo mátemático de um quadrirrotor pela formulação de 
//           Euler-Lagrange
// Autor: Diego Souza
// Data: 03/09/2015
//==================================================================

clear();        // Limpa variáveis
clc;            // Limpa terminal de comando
xdel(winsid()); // Fecha todos os gráficos abertos previamente

disp("-- MODELAGEM DE UM QUADRIRROTOR - FORMULAÇÃO DE EULER-LAGRANGE --")
disp("Simulação iniciada com sucesso em:")
disp(clock());
disp("Simulando... Aguarde...")
tic();          // Inicia contador

// Diretório de trabalho
diretorio = (get_absolute_file_path('MODELO_QUADCOPTER_LAGRANGE.sce'))

//*************************************
// VARIÁVEIS
//*************************************

g = 9.81;       // Aceleração da gravidade
m = 0.468;      // Massa do quadrirrotor
l = 0.225;      // Distância rotor x centro de massa
b = 1.140e-7;   // Coeficiente de arrasto da estrutura
k = 2.980e-6;   // Coeficiente de empuxo das hélices
Ir = 3.357e-5;  // Momento de inércia do rotor
ixx = 4.856e-3; // Momento de inércia em x
iyy = 4.856e-3; // Momento de inércia em y
izz = 8.801e-3; // Momento de inércia em z
ax = 0.25;      // Coeficiente de força de arrasto em x
ay = 0.25;      // Coeficiente de força de arrasto em y
az = 0.25;      // Coeficiente de força de arrasto em z

//*************************************
// EXECUÇÃO DO DIAGRAMA DE BLOCOS
//*************************************

importXcosDiagram(diretorio + "MODELO_QUADCOPTER_LAGRANGE.zcos") // Importa diagrama
typeof(scs_m)                                        // Tipo do objeto
xcos_simulate(scs_m,4)                               // Realiza a simulação
xdel(winsid());                                      // Fecha gráficos da simulação

//*************************************
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
legend(['ω1';'ω2';'ω3';'ω4'],opt=4); 
set(gca(),"grid",[4 4]); 

// Plot de Ângulos
subplot(222)
plot(PHI.time,PHI.values,'b-')
plot(THETA.time,THETA.values,'r-')
plot(PSI.time,PSI.values,'g-')

title("Ângulos de ‎Ø (Rolagem), Ɵ (Arfagem) e Ψ (Guinada)","color","blue","fontsize",3, "fontname",8);
xlabel("Tempo t (s)","fontsize",2,"color","blue");
ylabel("Ângulo (graus)","fontsize",3,"color","blue");
legend(['Ø (Rolagem)';'Ɵ (Arfagem)';'Ψ (Guinada)'],opt=2); 
set(gca(),"grid",[4 4]); 

// Plot de Posições 2D
subplot(223)
plot(X.time,X.values,'b-')
plot(Y.time,Y.values,'r-')
plot(Z.time,Z.values,'g-')

title("Posições x, y, e z","color","blue","fontsize",3, "fontname",8);
xlabel("Tempo t (s)","fontsize",2,"color","blue");
ylabel("Posição (m)","fontsize",3,"color","blue");
legend(['Posição x';'Posição y';'Posição z'],opt=3); 
set(gca(),"grid",[4 4]); 

// Plot de Posições 3D
subplot(224)
param3d(X.values, Y.values,Z.values,theta=280,alpha=80,leg="X@Y@Z",flag=[2,4]);
title("Posições x, y, e z","color","blue","fontsize",3, "fontname",8);
xlabel("Posição x (m)","fontsize",2,"color","blue");
ylabel("Posição y (m)","fontsize",3,"color","blue");
zlabel("Posição z (m)","fontsize",3,"color","blue");
e=gce() //the handle on the 3D polyline
e.foreground=color('blue');
set(gca(),"grid",[4 4]); 

//*************************************
// ESCREVENDO EM UM ARQUIVO DE SAÍDA
//*************************************

// Carrega função de concatenação
exec (diretorio + "fprintfMatAppend.sci");

// Diretório e arquivo de escrita
arquivo = "MODELO_QUADCOPTER_LAGRANGE.txt";

// Deleta o arquivo se ele já existir
deletefile(diretorio + arquivo);

// Escreve o arquivo
fprintfMatAppend(diretorio + arquivo, w1.time,"[TEMPO]")
fprintfMatAppend(diretorio + arquivo, w1.values,"[W1]")
fprintfMatAppend(diretorio + arquivo, w2.values,"[W2]")
fprintfMatAppend(diretorio + arquivo, w3.values,"[W3]")
fprintfMatAppend(diretorio + arquivo, w4.values,"[W4]")
fprintfMatAppend(diretorio + arquivo, PHI.values,"[PHI]")
fprintfMatAppend(diretorio + arquivo, THETA.values,"[THETA]")
fprintfMatAppend(diretorio + arquivo, PSI.values,"[PSI]")
fprintfMatAppend(diretorio + arquivo, X.values,"[X]")
fprintfMatAppend(diretorio + arquivo, Y.values,"[Y]")
fprintfMatAppend(diretorio + arquivo, Z.values,"[Z]")

// Finaliza a simulação
disp("Simulação finalizada com sucesso em:")
disp(clock());
disp("Tempo necessário para execução (s): ")
disp(toc())
