//==================================================================
// Programa: Modelo mátemático de um quadrirrotor pela formulação de 
//           Newton-Euler (usando somente script)
// Autor: Diego Souza
// Data: 16/09/2015
//=================================================================

clear();        // Limpa variáveis
clc;            // Limpa terminal de comando
xdel(winsid()); // Fecha todos os gráficos abertos previamente

disp("-- MODELAGEM DE UM QUADRIRROTOR - FORMULAÇÃO DE NEWTON-EULER --")
disp("Simulação iniciada com sucesso em:")
disp(clock());
disp("Simulando... Aguarde...")
tic();          // Inicia contador

// Diretório de trabalho
diretorio = (get_absolute_file_path('MODELO_QUADCOPTER_NEWTON_SCRIPT.sce'))

// Parâmetros de Simulação:
g = 9.81;       // Aceleração da gravidade [m/s^2]
m = 0.468;      // Massa do quadricóptero [kg]
l = 0.225;      // Distância entre hélice e centro [m]
k = 2.980e-6;   // Coeficiente de empuxo das hélices
d = 1.140e-7;   // Coeficiente de arrasto da estrutura
Im = 3.357e-5;  // Momento de Inércia [kg*m^2]
Ixx = 4.856e-3; // Momento de Inércia (x) [kg*mE^2]
Iyy = 4.856e-3; // Momento de Inércia (y) [kg*m^2]
Izz = 8.801e-3; // Momento de Inércia (z) [kg*m^2]
Ax = 0.25;      // Coeficientes de força de arrasto (x) [kg/s]
Ay = 0.25;      // Coeficientes de força de arrasto (y) [kg/s]
Az = 0.25;      // Coeficientes de força de arrasto (z) [kg/s]
Ir = 3.357e-5;

// Intervalo de tempo da simulação
dt = 0.01;

//*************************************
// CRIANDO AS ENTRADAS DE CONTROLE
//*************************************

disp("Carregando primeiro movimento (Aceleração).")

j = 0;

tempo_inicial = 0; tempo_final = 0.50; w_inicial = 620; amplitude = 65;

for i = tempo_inicial+dt:dt:tempo_final
   j = j + 1;   
   w1(j) = (amplitude*sin(((2*%pi)/tempo_final)*i)+w_inicial);   
   w2(j) = (amplitude*sin(((2*%pi)/tempo_final)*i)+w_inicial);
   w3(j) = (amplitude*sin(((2*%pi)/tempo_final)*i)+w_inicial);  
   w4(j) = (amplitude*sin(((2*%pi)/tempo_final)*i)+w_inicial);  
end

disp("Carregando segundo movimento (Rolagem).")

tempo_inicial = 0; tempo_final = 0.50; w_inicial = 620; amplitude = 30;

for i = tempo_inicial+dt:dt:tempo_final
   j = j + 1;   
   w1(j) = w_inicial; 
   w2(j) = (amplitude*sin(((2*%pi)/tempo_final)*i*-1)+w_inicial);  
   w3(j) = w_inicial; 
   w4(j) = (amplitude*sin(((2*%pi)/tempo_final)*i)+w_inicial);     
end

disp("Carregando terceiro movimento (Arfagem).")

for i = tempo_inicial+dt:dt:tempo_final
   j = j + 1;   
   w1(j) = (amplitude*sin(((2*%pi)/tempo_final)*i*-1)+w_inicial); 
   w2(j) = w_inicial;     
   w3(j) = (amplitude*sin(((2*%pi)/tempo_final)*i)+w_inicial); 
   w4(j) = w_inicial;        
end

disp("Carregando quarto movimento (Guinada).")

for i = tempo_inicial+dt:dt:tempo_final
   j = j + 1;   
   w1(j) = (amplitude*sin(((2*%pi)/tempo_final)*i)+w_inicial);     
   w2(j) = (amplitude*sin(((2*%pi)/tempo_final)*i*-1)+w_inicial);  
   w3(j) = (amplitude*sin(((2*%pi)/tempo_final)*i)+w_inicial);  
   w4(j) = (amplitude*sin(((2*%pi)/tempo_final)*i*-1)+w_inicial);
end

x=[1:j];

disp("Guardando o tempo...")

for i = 1:1:j
  tempo(i) = x(i)/(1/dt);
end

disp("Número de Amostras Para Simulação:")
disp(j)

//*************************************
// CALCULANDO O EMPUXO
//*************************************

disp("Calculando Empuxo...")

for i = 1:1:j
  u1(i) = (k*((w1(i))^2+(w2(i))^2+(w3(i))^2+(w4(i))^2));
end

//********************************************************
// CALCULANDO AS ACELERAÇÕES ANGULARES NO REFERENCIAL FIXO
//********************************************************

disp("Calculando as Acelerações no Referencial Fixo...")

p(1)=0;q(1)=0;r(1)=0;
p_ponto(1)=0;q_ponto(1)=0;r_ponto(1)=0;

p_ponto(1) = ((q(1)*r(1)*((Iyy-Izz)/Ixx))-(Ir*(q(1)/Ixx))+((l*k*(-(w2(1))^2+(w4(1))^2))/Ixx));
      
q_ponto(1) = ((p(1)*r(1)*((Izz-Ixx)/Iyy))-(Ir*(-p(1)/Iyy))+((l*k*(-(w1(1))^2+(w3(1))^2))/Iyy));
      
r_ponto(1) = ((p(1)*q(1)*((Ixx-Iyy)/Izz))+((d*(-(w1(1))^2+(w2(1))^2-(w3(1))^2+(w4(1))^2))/Izz));
      
for i = 2:1:j
      p_ponto(i)= ((q(i-1)*r(i-1)*((Iyy-Izz)/Ixx))-(Ir*(q(i-1)/Ixx))+((l*k*(-(w2(i))^2+(w4(i))^2))/Ixx)); 
      p(i) = p(i-1) + p_ponto(i) * dt;
      
      q_ponto(i) = ((p(i-1)*r(i-1)*((Izz-Ixx)/Iyy))-(Ir*(-p(i-1)/Iyy))+((l*k*(-(w1(i))^2+(w3(i))^2))/Iyy));
      q(i) = q(i-1) + q_ponto(i) * dt; 
            
      r_ponto(i)= ((p(i-1)*q(i-1)*((Ixx-Iyy)/Izz))+((d*(-(w1(i))^2+(w2(i))^2-(w3(i))^2+(w4(i))^2))/Izz));
      r(i) = r(i-1) + r_ponto(i) * dt;    
end

//************************************************************
// CALCULANDO AS ACELERAÇÕES ANGULARES NO REFERENCIAL INERCIAL
//************************************************************

disp("Calculando as Acelerações no Referencial Inercial...")

phi(1)=0; theta(1)=0; psi(1)=0;
phi_ponto(1)=0; theta_ponto(1)=0; psi_ponto(1)=0;

phi_ponto_ponto(1) = ((0*p(1))+((phi_ponto(1)*cos(phi(1))*tan(theta(1)))+(theta_ponto(1)*sin(phi(1))/cos(theta(1)^2)))*q(1)+((-phi_ponto(1)*sin(phi(1))*cos(theta(1)))+(theta_ponto(1)*cos(phi(1))/cos(theta(1))^2))*r(1)+(1*p_ponto(1))+(sin(phi(1))*tan(theta(1)))*q_ponto(1)+(cos(phi(1))*tan(theta(1)))*r_ponto(1));

theta_ponto_ponto(1) = ((0*p(1))+(-phi_ponto(1)*sin(phi(1)))*q(1)+(-phi_ponto(1)*cos(phi(1)))*r(1)+(0*p_ponto(1))+(cos(phi(1)))*q_ponto(1)+(-sin(phi(1)))*r_ponto(1));
      
psi_ponto_ponto(1) = ((0*p(i-1))+((phi_ponto(1)*cos(phi(1))/cos(theta(1)))+(phi_ponto(1)*sin(phi(1))*tan(theta(1))/cos(theta(1))))*q(1)+((-phi_ponto(1)*sin(phi(1))/cos(theta(1)))+(theta_ponto(1)*cos(phi(1))*tan(theta(1))/cos(theta(1))))*r(1)+(0*p_ponto(1))+(sin(phi(1))/cos(theta(1)))*q_ponto(1)+(cos(phi(1))/cos(theta(1)))*r_ponto(1));
     
for i = 2:1:j
    phi_ponto_ponto(i) = ((0*p(i-1))+((phi_ponto(i-1)*cos(phi(i-1))*tan(theta(i-1)))+(theta_ponto(i-1)*sin(phi(i-1))/cos(theta(i-1)^2)))*q(i-1)+((-phi_ponto(i-1)*sin(phi(i-1))*cos(theta(i-1)))+(theta_ponto(i-1)*cos(phi(i-1))/cos(theta(i-1))^2))*r(i-1)+(1*p_ponto(i-1))+(sin(phi(i-1))*tan(theta(i-1)))*q_ponto(i-1)+(cos(phi(i-1))*tan(theta(i-1)))*r_ponto(i-1));

    phi_ponto(i) = phi_ponto(i-1) + phi_ponto_ponto(i) * dt;
    phi(i) = phi(i-1) + phi_ponto(i) * dt;

    theta_ponto_ponto(i) = ((0*p(i-1))+(-phi_ponto(i-1)*sin(phi(i-1)))*q(i-1)+(-phi_ponto(i-1)*cos(phi(i-1)))*r(i-1)+(0*p_ponto(i-1))+(cos(phi(i-1)))*q_ponto(i-1)+(-sin(phi(i-1)))*r_ponto(i-1));
    
    theta_ponto(i) = theta_ponto(i-1) + theta_ponto_ponto(i) * dt;
    theta(i) = theta(i-1) + theta_ponto(i) * dt;

    psi_ponto_ponto(i) = ((0*p(i-1))+((phi_ponto(i-1)*cos(phi(i-1))/cos(theta(i-1)))+(phi_ponto(i-1)*sin(phi(i-1))*tan(theta(i-1))/cos(theta(i-1))))*q(i-1)+((-phi_ponto(i-1)*sin(phi(i-1))/cos(theta(i-1)))+(theta_ponto(i-1)*cos(phi(i-1))*tan(theta(i-1))/cos(theta(i-1))))*r(i-1)+(0*p_ponto(i-1))+(sin(phi(i-1))/cos(theta(i-1)))*q_ponto(i-1)+(cos(phi(i-1))/cos(theta(i-1)))*r_ponto(i-1));
      
    psi_ponto(i) = psi_ponto(i-1) + psi_ponto_ponto(i) * dt;
    psi(i) = psi(i-1) + psi_ponto(i) * dt;  
end

//************************************************************
// CALCULANDO AS ACELERAÇÕES LINEARES
//************************************************************
disp("Calculando as Acelerações Lineares...")

// Condições inciais (posições e acelerações)
x0(1)=0; y0(1)=0; z0(1)=0;
x_ponto(1)=0; y_ponto(1)=0; z_ponto(1)=0;

for i = 2:1:j 
  // Cálculo da translação em x  
  x_ponto_ponto(i) = (cos(phi(i-1))*sin(theta(i-1))*cos(psi(i-1))+sin(phi(i-1))*sin(psi(i-1))*(u1(i-1)/m)-(Ax*x_ponto(i-1)/m));
  x_ponto(i) = x_ponto(i-1) + x_ponto_ponto(i) * dt; // Velocidade em x
  x0(i) = x0(i-1) + x_ponto(i) * dt;                 // Posição em x
  // Cálculo da translação em y
  y_ponto_ponto(i) = (cos(phi(i))*sin(theta(i))*sin(psi(i))-sin(phi(i))*cos(psi(i))*(u1(i)/m)-(Ay*y_ponto(i-1)/m));
  y_ponto(i) = y_ponto(i-1) + y_ponto_ponto(i) * dt; // Velocidade em y
  y0(i) = y0(i-1) + y_ponto(i) * dt;                 // Posição em y
  // Cálculo da translação em z  
  z_ponto_ponto(i) = (cos(phi(i))*cos(theta(i))*(u1(i)/m)-(Az*z_ponto(i-1)/m)) - g;
  z_ponto(i) = z_ponto(i-1) + z_ponto_ponto(i) * dt; // Velocidade em z
  z0(i) = z0(i-1) + z_ponto(i) * dt;                 // Posição em z   
end

//*************************************
// PLOTAGEM DOS GRÁFICOS
//*************************************
disp("Plotando Gráficos...")

// Plot de Entradas de Controle
subplot(221)
plot(x/(1/dt),w1,'g-');
plot(x/(1/dt),w2,'r--');
plot(x/(1/dt),w3,'b:');
plot(x/(1/dt),w4,'k-.');

title("Entradas de controle ωi","color","blue","fontsize",3, "fontname",8);
xlabel("Tempo t (s)","fontsize",2,"color","blue");
ylabel("Ent. de controle ωi (rad/s)","fontsize",3,"color","blue");
legend(['ω1';'ω2';'ω3';'ω4'],opt=4); 
set(gca(),"grid",[4 4]); 

// Plot de Ângulos
subplot(222)
plot(x/(1/dt),phi*180/%pi,'g-');
plot(x/(1/dt),theta*180/%pi,'r--');
plot(x/(1/dt),psi*180/%pi,'b:');

title("Ângulos de ‎Ø (Rolagem), Ɵ (Arfagem) e Ψ (Guinada)","color","blue","fontsize",3, "fontname",8);
xlabel("Tempo t (s)","fontsize",2,"color","blue");
ylabel("Ângulo (graus)","fontsize",3,"color","blue");
legend(['Ø (Rolagem)';'Ɵ (Arfagem)';'Ψ (Guinada)'],opt=2); 
set(gca(),"grid",[4 4]); 

// Plot de Posições 2D
subplot(223)
plot(x/(1/dt),x0,'g-');
plot(x/(1/dt),y0,'r--');
plot(x/(1/dt),z0,'b:');

title("Posições x, y, e z","color","blue","fontsize",3, "fontname",8);
xlabel("Tempo t (s)","fontsize",2,"color","blue");
ylabel("Posição (m)","fontsize",3,"color","blue");
legend(['Posição x';'Posição y';'Posição z'],opt=3); 
set(gca(),"grid",[4 4]); 

// Plot de Posições 3D
subplot(224)
param3d(x0, y0, z0,theta=280,alpha=80,leg="X@Y@Z",flag=[2,4]);
title("Posições x, y, e z","color","blue","fontsize",3, "fontname",8);
xlabel("Posição x (m)","fontsize",2,"color","blue");
ylabel("Posição y (m)","fontsize",3,"color","blue");
zlabel("Posição z (m)","fontsize",3,"color","blue");
set(gca(),"grid",[4 4]); 
e=gce()
e.foreground=color('blue');
set(gca(),"grid",[4 4]); 

//*************************************
// ESCREVENDO EM UM ARQUIVO DE SAÍDA
//*************************************

// Carrega função de concatenação
exec (diretorio + "fprintfMatAppend.sci");

// Diretório e arquivo de escrita
arquivo = "MODELO_QUADCOPTER_NEWTON.txt";

// Deleta o arquivo se ele já existir
deletefile(diretorio + arquivo);

// Escreve o arquivo
fprintfMatAppend(diretorio + arquivo, tempo,"[TEMPO]");
fprintfMatAppend(diretorio + arquivo, w1,"[W1]");
fprintfMatAppend(diretorio + arquivo, w2,"[W2]");
fprintfMatAppend(diretorio + arquivo, w3,"[W3]");
fprintfMatAppend(diretorio + arquivo, w4,"[W4]");
fprintfMatAppend(diretorio + arquivo, phi*180/%pi,"[PHI]");
fprintfMatAppend(diretorio + arquivo, theta*180/%pi,"[THETA]");
fprintfMatAppend(diretorio + arquivo, psi*180/%pi,"[PSI]");
fprintfMatAppend(diretorio + arquivo, x0,"[X]");
fprintfMatAppend(diretorio + arquivo, y0,"[Y]");
fprintfMatAppend(diretorio + arquivo, z0,"[Z]");

// Finaliza a simulação
disp("Simulação finalizada com sucesso em:")
disp(clock());
disp("Tempo necessário para execução (s): ")
disp(toc())

