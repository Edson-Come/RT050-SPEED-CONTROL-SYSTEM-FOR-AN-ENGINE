
% Parâmetros Iniciais
tensao = 5;
tempo_total = 30;
timestep = 0.1;                   % Intervalo entre as amostras
tempo = 0:timestep:tempo_total;    % Vetor de tempo
velocidade_rotacao = zeros(size(tempo)); % Vetor para armazenar a velocidade do motor

% 1.1 Teste com tensão de 5V
RT050_SetMotorVoltage(tensao); % Aplica 5V ao motor
disp('Tensão de 5V aplicada ao motor por 30 segundos.');

% Inicialização do contador de tempo e leitura da velocidade
tic;
for amostra = 1:length(tempo)
    tempo(amostra) = toc; % Armazena o tempo atual
    velocidade_rotacao(amostra) = RT050_GetMotorSpeed(); % Ler a velocidade atual do motor
    pause(timestep); % Pausa para sincronização
end

% Desligar o motor após o teste de 5V
%RT050_SetMotorVoltage(0);
%disp('Teste de 5V concluído. Motor parado.');

% Plot da Velocidade de Rotação com 5V
figure;
plot(tempo, velocidade_rotacao);
xlabel('Tempo (s)');
ylabel('Velocidade de Rotação (RPM)');
title('Evolução da Velocidade de Rotação do Motor com 5V');
grid on;

% 1.2 Cálculo de RefVoltage
Xmed = (1 + 1 + 5 + 6 + 4 + 0) / 6;  %
refVoltage = (Xmed / 2) + 2; % Cálculo da tensão de referência
disp(['RefVoltage calculada: ', num2str(refVoltage), ' V']);

% 1.3 Aplicação de Sinal Sinusoidal para Resposta em Frequência
amplitude = 0.8;  
frequencia = 1;

% Aplicação de tensão constante de referência antes de aplicar o sinal sinusoidal
RT050_SetMotorVoltage(refVoltage);
disp('Tensão de referência aplicada. Esperando estabilização...(15 segundos)');
pause(15); % Tempo para estabilizar a velocidade
disp('Sinal Sinusoildal Aplicado');
% Aplicação do sinal sinusoidal
for i = 1:length(tempo)
    tensao = refVoltage + amplitude * sin(2 * pi * frequencia * tempo(i));
    RT050_SetMotorVoltage(tensao); 
    velocidade_rotacao(i) = RT050_GetMotorSpeed(); 
    pause(timestep);
end

% Calcular a Amplitude e Fase da Resposta
amplitude_resp = max(velocidade_rotacao) - min(velocidade_rotacao);
fase_resposta = angle(hilbert(velocidade_rotacao));

% Plot da Resposta em Fase e Amplitude
figure;
subplot(2,1,1);
plot(tempo, velocidade_rotacao, 'b');
xlabel('Tempo (s)');
ylabel('Velocidade de Rotação (RPM)');
title('Resposta em Amplitude (Sinal Sinusoidal)');
grid on;

subplot(2,1,2);
plot(tempo, fase_resposta, 'b');
xlabel('Tempo (s)');
ylabel('Fase (radianos)');
title('Resposta em Fase (Sinal Sinusoidal)');
grid on;

% Varredura de Frequências para Resposta em Frequência
frequencias = logspace(-2, -1, 20);                                     
tempo_total = 10;
tempo = 0:timestep:tempo_total;
amplitudes_resposta = zeros(size(frequencias));
fases_resposta = zeros(size(frequencias));
disp('Iniciando varredura de frequências...');

for f = 1:length(frequencias)
    frequencia = frequencias(f);
    velocidade_rotacao = zeros(size(tempo));
    
    % Aplicação do sinal sinusoidal para cada frequência
    for i = 1:length(tempo)
        tensao = refVoltage + amplitude * sin(2 * pi * frequencia * tempo(i));
        RT050_SetMotorVoltage(tensao); % Aplica tensão
        velocidade_rotacao(i) = RT050_GetMotorSpeed(); % Medir a velocidade do motor
        pause(timestep);
    end
    
    % Calcular amplitude e fase da resposta
    amplitudes_resposta(f) = (max(velocidade_rotacao) - min(velocidade_rotacao)) / 2;
    fase_instantanea = angle(hilbert(velocidade_rotacao(end-99:end)));
    fases_resposta(f) = mean(fase_instantanea);
    
    fprintf('Freq: %f Hz, Amplitude: %f, Fase: %f rad\n', frequencia, amplitudes_resposta(f), fases_resposta(f));
end

% Plot da Resposta em Frequência
figure;
subplot(2,1,1);
semilogx(frequencias, 20*log10(amplitudes_resposta), 'b');
xlabel('Frequência (Hz)');
ylabel('Amplitude (dB)');
title('Resposta em Amplitude (Varredura de Frequências)');
grid on;

subplot(2,1,2);
semilogx(frequencias, fases_resposta, 'b');
xlabel('Frequência (Hz)');
ylabel('Fase (radianos)');
title('Resposta em Fase (Varredura de Frequências)');
grid on;

% 2.2 Aplicação de Onda Quadrada para Estimar o Tau do Sistema
tempo_total = 30;
tempo = 0:timestep:tempo_total;
amplitude_quadrada = 0.6;
frequencia_quadrada = 0.5;

disp('Aplicando onda quadrada ao motor...');
for i = 1:length(tempo)
    tensao = refVoltage + amplitude_quadrada * square(2 * pi * frequencia_quadrada * tempo(i));
    RT050_SetMotorVoltage(tensao); % Aplica onda quadrada
    velocidade_rotacao(i) = RT050_GetMotorSpeed(); % Ler velocidade do motor
    pause(timestep);
end

% Cálculo do Tau (63,2% do valor final)
saida_final = velocidade_rotacao(end);
[~, idx_tau] = min(abs(velocidade_rotacao - 0.632 * saida_final));
tau_aproximado = tempo(idx_tau);

disp(['Constante de tempo tau estimada: ', num2str(tau_aproximado), ' segundos']);
RT050_SetMotorVoltage(0); % Desligar motor
disp('Motor desligado.');

% Plot da Resposta Observada com a Onda Quadrada
figure;
plot(tempo, velocidade_rotacao, 'r');
xlabel('Tempo (s)');
ylabel('Velocidade de Rotação (RPM)');
title('Evolução da Velocidade com Onda Quadrada Aplicada');
grid on;

% Modelo de Primeira Ordem
K = 40; % Ganho estimado
tau = tau_aproximado;

resposta_modelo = zeros(size(tempo));

for i = 1:length(tempo)
    resposta_modelo(i) = resposta_modelo(i-1) + ...
                         (K * amplitude_quadrada * square(2 * pi * frequencia_quadrada * tempo(i)) - resposta_modelo(i-1)) * timestep / tau;
end

% Plot Comparativo da Resposta Observada e Modelo
hold on;
plot(tempo, resposta_modelo, 'b--');
legend('Resposta Observada', 'Resposta Estimada pelo Modelo');
hold off;
