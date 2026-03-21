tensao = 4;
tempoTotal = 30;
T = 0.1;                    %periodo
tempo = 0:T:tempoTotal;
velocidade_rotacao = zeros(size(tempo));

RT050_SetMotorVoltage(tensao);
fprintf('A Estabilizar a 4V (15segundos)')
pause(15);

RT050_SetMotorVoltage(1);
for i = 1:length(tempo)
    tic;
    velocidade_rotacao(i) = RT050_GetMotorSpeed;
    while toc<T
    end
end
RT050_SetMotorVoltage(0);
pause(10);

%figure;
%plot(tempo, velocidade_rotacao);
%xlabel('Tempo (s)');
%ylabel('Velocidade de Rotação (RPM)');
%title('Velocidade de Rotação');
%grid on;

saida_final = velocidade_rotacao(end);
percentual_632 = 1.896 * saida_final; % 63,2% da diferença final

[~, idx_tau] = min(abs(velocidade_rotacao - percentual_632));   % y(t) = -3K + Ke^(-t/tau)
tau = tempo(idx_tau); % constante de tempo tau

figure;
plot(tempo, velocidade_rotacao, 'b');
hold on;
yline(percentual_632, 'r--', '63,2% da variação total', 'LabelHorizontalAlignment', 'left');
xline(tau, 'g--', ['\tau = ', num2str(tau), ' s'], 'LabelVerticalAlignment', 'bottom');

xlabel('Tempo (s)');
ylabel('Velocidade de Rotação (RPM)');
title('Estimativa da Constante de Tempo \tau');
grid on;
legend('Resposta do Sistema', '63,2% da Variação', 'Estimativa de \tau', 'Location', 'Best');
hold off;

fprintf(['\nConstante de tempo tau estimada: ', num2str(tau), ' segundos']);
 
%saida_final = velocidade_rotacao(end);
%[~, idx_tau] = min(abs(velocidade_rotacao - 1.896* saida_final));
%tempo_tau = tempo(idx_tau);

%% 2.1
%tau = 7.9;          
%PO = 15;
%xi = sqrt((log(PO/100))^2 / (pi^2 + ((log(PO/100))^2)));
%omega = 1/(tau*xi) ; 

%syms t s
%s = tf('s');
%Gs = 1/(s^2 + 2*omega*xi*s + omega^2);

% inversa de laplace de Gs 
%g_t = ilaplace(Gs, s, t);    % g(t)

T = 0.15;    %tempo de amostragem 
Xs = 4000;              
K = 1/5000;

tempoTotal = 30;
tempo = 0:T:tempoTotal;
velocidade_rotacao = zeros(size(tempo));

Kp = 5;           % 5   5ganho proporcional - resposta inicial aumenta-lo reduz tempo de estabilizacao
Ki = 2;             %2.4    2ganho integral deve ser baixo 
Kd = 6;          %2  6ganho derivativo - reduz oscilacoes
Es =zeros(size(tempo));
Es_i = 0;       %erro integral

for i = 1:length(tempo)
    tic;

   Ys = RT050_GetMotorSpeed;       
   Es(i+1) = Xs*K - Ys*K;
   Es_i = Es_i + Es(i+1)*T;     %integral da area  - soma das areas

   Vs = Kp*Es(i+1) + Ki*Es_i + Kd*(Es(i+1)-Es(i))/T;% implementando um controlador pid
           
   RT050_SetMotorVoltage(Vs);
   velocidade_rotacao(i) = RT050_GetMotorSpeed;
    while toc<T
    end
end

figure;
plot(tempo, velocidade_rotacao);
xlabel('Tempo (s)');
ylabel('Velocidade de Rotação (RPM)');
title('Velocidade de Rotação');
grid on;
RT050_SetMotorVoltage(0);
pause(10);


%% 2.2
T = 0.15;
K = 1/5000;
tempoTotal = 80;
tempo = 0:T:tempoTotal;      % alterar o tamnho da matriz de 30s para 80segundos
velocidade_rotacao = zeros(size(tempo));
Xs = zeros(size(tempo));
Es = zeros(size(tempo));
Es_i = 0;       %erro integral 

Kp = 5;         % 5   5ganho proporcional - resposta inicial aumenta-lo reduz tempo de estabilizacao
Ki = 2;       %2.4    2ganho integral deve ser baixo 
Kd = 6;         %2  6ganho derivativo - reduz oscilacoes

for i = 2:length(tempo)
    tic;

    if tempo(i) >= 0 && tempo(i) < 20
        Xs(i) = 2000;
    elseif tempo(i) >= 20 && tempo(i) < 40
        Xs(i) = 3000;
    elseif tempo(i) >= 40 && tempo(i) < 60
        Xs(i) = 2500;
    else
        Xs(i) = 0;
    end
%%
       Ys = RT050_GetMotorSpeed;       
    Es(i) = Xs(i)*K - Ys*K;
    Es_i = Es_i + Es(i)*T;     %integral da area  - soma das areas

    Vs = Kp*Es(i) + Ki*Es_i + Kd*(Es(i)-Es(i-1))/T;% implementando um controlador pid
           
   RT050_SetMotorVoltage(Vs);
   velocidade_rotacao(i) = RT050_GetMotorSpeed;
    while toc < T
    end
end

figure;
plot(tempo, velocidade_rotacao);
xlabel('Tempo (s)');
ylabel('Velocidade de Rotação (RPM)');
title('Velocidade de Rotação');
grid on;
RT050_SetMotorVoltage(0);



