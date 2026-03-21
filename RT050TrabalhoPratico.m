% parâmetros iniciais
Xmed = (1 + 1 + 5 + 6 + 4 + 0) / 6; % Cálculo da média dos valores nmec
refVoltage = (Xmed / 2) + 2;    % valor de tensao de referencia 4.83 volts    
amplitude = 0.8;                   
frequencia = 1;                     
tempo_total = 30;                   % Tempo total de medição
timestep = 0.1;                     % Intervalo de amostragem 
RT050_SetMotorVoltage(refVoltage);
pause(15);                          %tempo que leva para estabilizar
% Inicialização
tempo = 0:timestep:tempo_total;    % Vetor de tempo
velocidade_rotacao = zeros(size(tempo)); % Vetor para armazenar a velocidade do motor


% Aplicar sinal de tensão sinusoidal
tic;
disp('Tensão aplicada ao Motor.');
for i = 1:length(tempo)
    % Gerar tensão sinusoidal com valor médio refVoltage
    tensao = refVoltage + amplitude * sin(2 * pi * frequencia * tempo(i));
    RT050_SetMotorVoltage(tensao); % Aplicar a tensão ao moto
    velocidade_rotacao(i) = RT050_GetMotorSpeed(); %ler a velocidade atual do motor
    
    pause(timestep);
end
RT050_SetMotorVoltage(0); % Desligar o motor
disp('Tensão retirada. Motor desacelerando.');

% Calcular a amplitude e fase da resposta
amplitude_resp = max(velocidade_rotacao) - min(velocidade_rotacao);
fase_resposta = angle(hilbert(velocidade_rotacao)); % Fase da resposta com a função hilbert

% Plotar a resposta em freq(amplitude e fase)
figure;
hold on
subplot(2,1,1);
plot(tempo, velocidade_rotacao, 'g');
xlabel('Tempo (s)');
ylabel('Velocidade de Rotação (RPM)');
title('Resposta em Amplitude');
grid on;

subplot(2,1,2);
plot(tempo, fase_resposta, 'b');
xlabel('Tempo (s)');
ylabel('Fase (radianos)');
title('Resposta em Fase');
grid on;

amplitude_quadrada = 0.6;       % Amplitude do sinal quadrado
frequencia_quadrada = 0.5;      % Frequência do sinal

disp('Tensão refVoltage e onda Quadrada aplicada ao Motor.');
velocidade_rotacaoQuad = zeros(size(tempo));

RT050_SetMotorVoltage(refVoltage);
pause(15);                          %tempo que leva para estabilizar

for i = 1:length(tempo)
    tensao = refVoltage + amplitude_quadrada * square(2 * pi * frequencia_quadrada *  tempo(i));
    RT050_SetMotorVoltage(tensao);
    velocidade_rotacaoQuad(i) = RT050_GetMotorSpeed();
    pause(timestep);
end

RT050_SetMotorVoltage(0);

disp('Tensão retirada. Motor desacelerando.');

% Comparação com o modelo ajustado  
figure;
plot(tempo, velocidade_rotacaoQuad, 'r');
xlabel('Tempo (s)');
ylabel('Velocidade de Rotação (RPM)');
title('Resposta do Sistema e Comparação com o Modelo');
hold off
