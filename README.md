Pêndulo Invertido Rotativo

Firmware para ESP32 que estabiliza um pêndulo invertido rotativo na posição vertical (180°). O controlador é um PID projetado pelo método do lugar geométrico das raízes (LGR), desenvolvido na disciplina de Sistemas de Controle II (UFERSA, campus Pau dos Ferros).

Projeto do controlador
Planta linearizada em torno do topo: G(s) = 1/(ml²) / (s² − g/l), com polo instável em +5,82 rad/s
Controlador: Gc(s) = K(s + z)²/s, com dois zeros iguais e integrador, sem cancelamento de polos da planta
Especificações dos polos dominantes: sobressinal de 5% e tempo de acomodação de 0,5 s
Ganhos: Kp = 0,1870, Ki = 0,5601, Kd = 0,0156
Implementação
Malha de controle a 1 kHz
Geração de passos por DDS em interrupção de 20 kHz, para o driver A4988 (1/16 de passo)
Leitura de encoder de 600 PPR em quadratura (2400 contagens por volta)
Derivada por diferença de uma amostra, sem filtro, para manter o código fiel ao controlador projetado
Zona de engate de ±12° e de operação de ±35° em torno do topo
Registro em RAM e relatório de queda pela serial
Hardware

ESP32, driver A4988, motor de passo (200 passos/rev) e encoder incremental de 600 PPR.

Limitações conhecidas

O modelo linear considera massa concentrada e torque direto no pivô. O código acrescenta saturação de torque, integrador com vazamento e realimentação do braço, que não fazem parte do projeto teórico. Os ganhos de projeto são o ponto de partida, e o ajuste final é feito em bancada.
