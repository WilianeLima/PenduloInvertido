#include <Arduino.h>
#include <math.h>
#include "soc/gpio_struct.h"
#include "soc/gpio_reg.h"

#define PI_F     3.14159265358979f
#define DEG2RAD  (PI_F / 180.0f)

#define K_LGR            0.015601f
#define Z_LGR            5.9888f
#define DT               0.001f
#define TORQUE_MAX_NM    0.14f
#define SETPOINT_GRAUS   180.0f
#define ZONA_ENGATE_GRAUS 12.0f
#define ZONA_ATIVA_GRAUS 35.0f

#define M_MASSA          0.0085f
#define L_COMP           0.29f
#define J_INERCIA        (M_MASSA * L_COMP * L_COMP)
#define PASSOS_POR_VOLTA 3200.0f

#define K_POS_BRACO_INI  0.0025f
#define K_VEL_BRACO_INI  0.0012f

#define K_LEAN_POS_INI   0.02f
#define K_LEAN_VEL_INI   0.005f
#define LEAN_MAX_GRAUS   0.0f

#define LIMITE_BRACO_DEG 150.0f

static float g_k_pos_braco = K_POS_BRACO_INI;
static float g_k_vel_braco = K_VEL_BRACO_INI;
static float g_sinal_braco = 1.0f;
static float g_k_lean_pos  = K_LEAN_POS_INI;
static float g_k_lean_vel  = K_LEAN_VEL_INI;
static float g_sinal_lean  = 1.0f;

#define PIN_ENC_A     32
#define PIN_ENC_B     33
#define PIN_STEP      25
#define PIN_DIR       26
#define PIN_EN        27
#define PIN_BTN_BOOT   0

#define PPR            600
#define CPR            (PPR * 4)
#define ENC_INVERT     0
#define DIR_INVERT     0

#define STEP_ISR_HZ    20000UL
#define FREQ_MAX_HZ    8000.0f

typedef struct {
    float Kp, Ki, Kd;
    float dt;
    float limite_torque;
    float integral;
    float erro_anterior;
    float torque_saida;
} PID_t;

void PID_Reset(PID_t *pid) {
    pid->integral       = 0.0f;
    pid->erro_anterior  = 0.0f;
    pid->torque_saida   = 0.0f;
}

void PID_Init(PID_t *pid, float Kp, float Ki, float Kd, float dt, float limite_torque) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->dt = dt;
    pid->limite_torque = limite_torque;
    PID_Reset(pid);
}

void PID_InitKz(PID_t *pid, float K, float z, float dt, float limite_torque) {
    PID_Init(pid, 2.0f * K * z, K * z * z, K, dt, limite_torque);
}

float PID_Calcular(PID_t *pid, float setpoint_graus, float angulo_encoder_graus) {
    float e = (setpoint_graus - angulo_encoder_graus) * DEG2RAD;

    float P = pid->Kp * e;
    float D = pid->Kd * (e - pid->erro_anterior) / pid->dt;

    float i_novo = (pid->integral * 0.995f) + e * pid->dt;
    float I      = pid->Ki * i_novo;

    float u = P + I + D;

    if (pid->limite_torque > 0.0f) {
        if (u > pid->limite_torque) {
            u = pid->limite_torque;
            if (e < 0.0f) pid->integral = i_novo;
        } else if (u < -pid->limite_torque) {
            u = -pid->limite_torque;
            if (e > 0.0f) pid->integral = i_novo;
        } else {
            pid->integral = i_novo;
        }
    } else {
        pid->integral = i_novo;
    }

    pid->erro_anterior = e;
    pid->torque_saida  = u;
    return u;
}

volatile bool    flag_1khz     = false;
volatile long    encoder_count = 0;
volatile uint8_t enc_state     = 0;

volatile int32_t g_rate_half   = 0;
volatile long    g_step_pos    = 0;
static uint32_t  s_phase       = 0;
static bool      s_pulse_high  = false;
static bool      s_pending     = false;
static int8_t    s_dir_cur     = 1;
static uint8_t   s_div_1khz    = 0;

static float     vel_cmd_rad_s = 0.0f;
static long      step_zero     = 0;

#define LOG_MAX_SAMPLES 1000
typedef struct {
    uint16_t t_ms;
    float    ang;
    float    erro;
    float    tau_pid;
    float    tau_total;
    float    step_hz;
    float    braco_deg;
} LogSample_t;

static LogSample_t g_log_buf[LOG_MAX_SAMPLES];
static int         g_log_count = 0;
static int         g_log_div   = 0;
static uint32_t    g_log_t0_ms = 0;

void imprimir_ganhos_braco(void) {
    Serial.printf("# K_pos=%.5f K_vel=%.5f sinal_braco=%+.0f | lean_pos=%.4f lean_vel=%.4f sinal_lean=%+.0f | lean_max=%.1f\n",
                  g_k_pos_braco, g_k_vel_braco, g_sinal_braco,
                  g_k_lean_pos, g_k_lean_vel, g_sinal_lean, (float)LEAN_MAX_GRAUS);
}

void imprimir_relatorio_queda(const char *motivo) {
    if (g_log_count <= 0) return;
    Serial.println("\n================ INICIO LOG ================");
    Serial.printf("# Motivo da parada: %s | Amostras: %d (dt = 5 ms)\n", motivo, g_log_count);
    Serial.printf("# K_LGR=%.6f | Z_LGR=%.4f | Kp=%.4f | Ki=%.4f | Kd=%.4f\n",
                  K_LGR, Z_LGR, 2.0f * K_LGR * Z_LGR, K_LGR * Z_LGR * Z_LGR, K_LGR);
    imprimir_ganhos_braco();
    Serial.println("t_ms,ang_deg,erro_deg,tau_pid_nm,tau_total_nm,step_hz,braco_deg");
    for (int i = 0; i < g_log_count; i++) {
        Serial.printf("%u,%.2f,%.2f,%.4f,%.4f,%.1f,%.1f\n",
                      g_log_buf[i].t_ms, g_log_buf[i].ang, g_log_buf[i].erro,
                      g_log_buf[i].tau_pid, g_log_buf[i].tau_total,
                      g_log_buf[i].step_hz, g_log_buf[i].braco_deg);
    }
    Serial.println("================ FIM LOG ================\n");
}

hw_timer_t *timer = NULL;

#define STEP_MASK (1UL << PIN_STEP)
#define DIR_MASK  (1UL << PIN_DIR)

static const int8_t DRAM_ATTR QUAD_TABLE[16] = {
     0, -1,  1,  0,
     1,  0,  0, -1,
    -1,  0,  0,  1,
     0,  1, -1,  0
};

static inline uint8_t IRAM_ATTR ler_ab_fast(void) {
    uint32_t in1 = GPIO.in1.val;
    uint8_t a = (in1 >> (PIN_ENC_A - 32)) & 1U;
    uint8_t b = (in1 >> (PIN_ENC_B - 32)) & 1U;
    return (a << 1) | b;
}

void IRAM_ATTR onTimer20kHz(void) {
    uint8_t ab = ler_ab_fast();
    enc_state = ((enc_state << 2) | ab) & 0x0F;
    int8_t delta = QUAD_TABLE[enc_state];
#if ENC_INVERT
    encoder_count -= delta;
#else
    encoder_count += delta;
#endif

    if (++s_div_1khz >= 20) {
        s_div_1khz = 0;
        flag_1khz = true;
    }

    bool just_cleared = false;
    if (s_pulse_high) {
        REG_WRITE(GPIO_OUT_W1TC_REG, STEP_MASK);
        s_pulse_high = false;
        just_cleared = true;
    }

    int32_t  r   = g_rate_half;
    int8_t   dir = (r >= 0) ? 1 : -1;
    uint32_t inc = (uint32_t)(r >= 0 ? r : -r) << 1;

    if (dir != s_dir_cur) {
        bool level = (dir > 0) ^ (DIR_INVERT != 0);
        REG_WRITE(level ? GPIO_OUT_W1TS_REG : GPIO_OUT_W1TC_REG, DIR_MASK);
        s_dir_cur = dir;
        s_pending = false;
        return;
    }

    uint32_t old = s_phase;
    s_phase += inc;
    if (s_phase < old) s_pending = true;

    if (s_pending && !just_cleared) {
        REG_WRITE(GPIO_OUT_W1TS_REG, STEP_MASK);
        s_pulse_high = true;
        s_pending    = false;
        g_step_pos  += s_dir_cur;
    }
}

float encoder_ler_graus(void) {
    long count = encoder_count;
    long mod = count % CPR;
    if (mod < 0) mod += CPR;
    return (float)mod * 360.0f / (float)CPR;
}

float braco_ler_rad(void) {
    long passos = g_step_pos - step_zero;
    return (float)passos * (2.0f * PI_F / PASSOS_POR_VOLTA);
}

void motor_set_freq_signed_hz(float f) {
    if (f >  FREQ_MAX_HZ) f =  FREQ_MAX_HZ;
    if (f < -FREQ_MAX_HZ) f = -FREQ_MAX_HZ;
    g_rate_half = (int32_t)(f * (2147483648.0f / (float)STEP_ISR_HZ));
}

void motor_enable(bool on) {
    if (PIN_EN >= 0) digitalWrite(PIN_EN, on ? LOW : HIGH);
}

static void aplicar_torque_motor(float tau_nm) {
    if (tau_nm >  TORQUE_MAX_NM) tau_nm =  TORQUE_MAX_NM;
    if (tau_nm < -TORQUE_MAX_NM) tau_nm = -TORQUE_MAX_NM;

    float alpha_cmd = tau_nm / J_INERCIA;
    vel_cmd_rad_s += alpha_cmd * DT;

    float freq_signed = vel_cmd_rad_s * (PASSOS_POR_VOLTA / (2.0f * PI_F));
    if (freq_signed > FREQ_MAX_HZ) {
        freq_signed = FREQ_MAX_HZ;
        vel_cmd_rad_s = freq_signed * (2.0f * PI_F / PASSOS_POR_VOLTA);
    } else if (freq_signed < -FREQ_MAX_HZ) {
        freq_signed = -FREQ_MAX_HZ;
        vel_cmd_rad_s = freq_signed * (2.0f * PI_F / PASSOS_POR_VOLTA);
    }
    motor_set_freq_signed_hz(freq_signed);
}

void executar_autocalibracao(bool *controle_ativo, int8_t *sinal_motor) {
    *controle_ativo = false;
    vel_cmd_rad_s   = 0.0f;
    motor_set_freq_signed_hz(0.0f);
    Serial.println("\n>>> [AUTO] Auto-calibracao (mantenha a haste solta embaixo)...");
    delay(300);
    long c0 = encoder_count;

    motor_set_freq_signed_hz(800.0f);
    delay(80);
    motor_set_freq_signed_hz(0.0f);
    delay(40);
    long delta_enc = encoder_count - c0;

    motor_set_freq_signed_hz(-800.0f);
    delay(80);
    motor_set_freq_signed_hz(0.0f);

    if (delta_enc == 0) {
        Serial.println(">>> [AUTO] AVISO: encoder nao contou durante o pulso! Verifique fios A/B.");
    } else {
        *sinal_motor = (delta_enc > 0) ? 1 : -1;
        Serial.printf(">>> [AUTO] delta_enc = %+ld -> sinal_motor = %+d\n", delta_enc, *sinal_motor);
    }

    Serial.println(">>> [AUTO] Aguardando 1.5 s para a haste parar e zerar em 0.00 graus...");
    delay(1500);
    encoder_count = 0;
    step_zero     = g_step_pos;
    Serial.printf(">>> [AUTO] Concluido! Encoder = 0.00, sinal_motor = %+d. Levante ate 180!\n\n", *sinal_motor);
}

void setup() {
    Serial.begin(115200);
    delay(200);
    Serial.println("==================================================");
    Serial.println("BOOT OK - Pendulo invertido (DDS 20 kHz + PID LGR)");
    Serial.printf("PINOS -> STEP:%d DIR:%d EN:%d ENC_A:%d ENC_B:%d\n",
                  PIN_STEP, PIN_DIR, PIN_EN, PIN_ENC_A, PIN_ENC_B);
    Serial.println("Comandos: a auto-cal | c zerar | t topo(180) | i inverter motor | l log");
    Serial.println("Braco:    q/w K_pos -/+ | e/r K_vel -/+ | b inverter | g mostrar");
    Serial.println("Retorno:  u/j forca +/- | y/h amortecimento +/- | n inverter");
    Serial.println("==================================================");

    pinMode(PIN_STEP, OUTPUT);
    digitalWrite(PIN_STEP, LOW);
    pinMode(PIN_DIR, OUTPUT);
    digitalWrite(PIN_DIR, DIR_INVERT ? LOW : HIGH);
    if (PIN_EN >= 0) pinMode(PIN_EN, OUTPUT);
    motor_enable(true);

    pinMode(PIN_ENC_A, INPUT_PULLUP);
    pinMode(PIN_ENC_B, INPUT_PULLUP);
    pinMode(PIN_BTN_BOOT, INPUT_PULLUP);
    enc_state = ler_ab_fast();

#if defined(ESP_ARDUINO_VERSION) && (ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0))
    timer = timerBegin(1000000);
    timerAttachInterrupt(timer, &onTimer20kHz);
    timerAlarm(timer, 1000000UL / STEP_ISR_HZ, true, 0);
#else
    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, &onTimer20kHz, true);
    timerAlarmWrite(timer, 1000000UL / STEP_ISR_HZ, true);
    timerAlarmEnable(timer);
#endif
}

void loop() {
    static PID_t pid;
    static bool primeira_vez = true;
    static bool controle_ativo = false;
    static int8_t sinal_motor = -1;
    static unsigned long btn_down_ms = 0;
    static bool btn_pressed = false;

    if (primeira_vez) {
        PID_InitKz(&pid, K_LGR, Z_LGR, DT, TORQUE_MAX_NM);
        primeira_vez = false;
    }

    if (digitalRead(PIN_BTN_BOOT) == LOW) {
        if (!btn_pressed) { btn_pressed = true; btn_down_ms = millis(); }
    } else if (btn_pressed) {
        unsigned long dur = millis() - btn_down_ms;
        btn_pressed = false;
        if (dur >= 1000) {
            executar_autocalibracao(&controle_ativo, &sinal_motor);
        } else if (dur >= 50) {
            encoder_count = 0;
            Serial.println("\n>>> [BOOT] Encoder zerado (0.00 graus)\n");
        }
    }

    while (Serial.available() > 0) {
        char ch = (char)Serial.read();
        if (ch == 'c' || ch == 'C') {
            encoder_count = 0;
            Serial.println("\n>>> [MANUAL] Encoder zerado (0.00 graus)\n");
        } else if (ch == 't' || ch == 'T') {

            encoder_count = CPR / 2;
            Serial.println("\n>>> [MANUAL] Posicao atual definida como TOPO (180.00 graus). Solte a haste.\n");
        } else if (ch == 'i' || ch == 'I') {
            sinal_motor = -sinal_motor;
            Serial.printf("\n>>> [MANUAL] sinal_motor = %+d\n\n", sinal_motor);
        } else if (ch == 'a' || ch == 'A') {
            executar_autocalibracao(&controle_ativo, &sinal_motor);
        } else if (ch == 'l' || ch == 'L') {
            imprimir_relatorio_queda("Solicitado manualmente via 'l'");
        } else if (ch == 'q' || ch == 'w' || ch == 'e' || ch == 'r' || ch == 'b' ||
                   ch == 'u' || ch == 'j' || ch == 'y' || ch == 'h' || ch == 'n' || ch == 'g') {
            if (ch == 'q') g_k_pos_braco *= 0.7f;
            if (ch == 'w') g_k_pos_braco *= 1.4f;
            if (ch == 'e') g_k_vel_braco *= 0.7f;
            if (ch == 'r') g_k_vel_braco *= 1.4f;
            if (ch == 'b') g_sinal_braco = -g_sinal_braco;
            if (ch == 'u') g_k_lean_pos *= 1.4f;
            if (ch == 'j') g_k_lean_pos *= 0.7f;
            if (ch == 'y') g_k_lean_vel *= 1.4f;
            if (ch == 'h') g_k_lean_vel *= 0.7f;
            if (ch == 'n') g_sinal_lean = -g_sinal_lean;
            Serial.println();
            imprimir_ganhos_braco();
            Serial.println();
        }
    }

    if (flag_1khz) {
        flag_1khz = false;

        float ang       = encoder_ler_graus();
        float braco_rad = braco_ler_rad();
        float braco_deg = braco_rad * (180.0f / PI_F);
        float r         = SETPOINT_GRAUS;
        float tau       = 0.0f;

        bool dentro_topo = (fabsf(SETPOINT_GRAUS - ang) <= ZONA_ATIVA_GRAUS);
        bool cabo_seguro = (!controle_ativo) || (fabsf(braco_deg) <= LIMITE_BRACO_DEG);

        bool pode_engatar = (fabsf(SETPOINT_GRAUS - ang) <= ZONA_ENGATE_GRAUS);
        bool zona_ok      = controle_ativo ? dentro_topo : pode_engatar;

        if (zona_ok && cabo_seguro) {
            if (!controle_ativo) {
                PID_Reset(&pid);
                pid.erro_anterior = (SETPOINT_GRAUS - ang) * DEG2RAD;
                vel_cmd_rad_s  = 0.0f;
                step_zero      = g_step_pos;
                braco_rad      = 0.0f;
                braco_deg      = 0.0f;
                g_log_count    = 0;
                g_log_div      = 0;
                g_log_t0_ms    = millis();
                controle_ativo = true;
            }

            float vel_braco_dps = vel_cmd_rad_s * (180.0f / PI_F);
            float lean = (float)sinal_motor * g_sinal_lean *
                         (g_k_lean_pos * braco_deg + g_k_lean_vel * vel_braco_dps);
            if (lean >  LEAN_MAX_GRAUS) lean =  LEAN_MAX_GRAUS;
            if (lean < -LEAN_MAX_GRAUS) lean = -LEAN_MAX_GRAUS;
            r = SETPOINT_GRAUS + lean;

            float tau_pid = PID_Calcular(&pid, r, ang);

            float tau_braco = g_sinal_braco * (-g_k_pos_braco * braco_rad
                                               - g_k_vel_braco * vel_cmd_rad_s);

            tau = tau_pid * (float)sinal_motor + tau_braco;

            aplicar_torque_motor(tau);

            if (++g_log_div >= 5 && g_log_count < LOG_MAX_SAMPLES) {
                g_log_div = 0;
                float freq_step = vel_cmd_rad_s * (PASSOS_POR_VOLTA / (2.0f * PI_F));
                g_log_buf[g_log_count].t_ms      = (uint16_t)(millis() - g_log_t0_ms);
                g_log_buf[g_log_count].ang       = ang;
                g_log_buf[g_log_count].erro      = r - ang;
                g_log_buf[g_log_count].tau_pid   = tau_pid;
                g_log_buf[g_log_count].tau_total = tau;
                g_log_buf[g_log_count].step_hz   = freq_step;
                g_log_buf[g_log_count].braco_deg = braco_deg;
                g_log_count++;
            }
        } else {
            if (controle_ativo) {
                const char *motivo = !dentro_topo ? "Haste saiu da zona do topo"
                                                  : "Braco atingiu limite de seguranca do cabo";
                controle_ativo = false;
                vel_cmd_rad_s  = 0.0f;
                motor_set_freq_signed_hz(0.0f);
                PID_Reset(&pid);
                imprimir_relatorio_queda(motivo);
            }
        }

        static unsigned long last_log = 0;
        unsigned long periodo = controle_ativo ? 500 : 100;
        if (millis() - last_log >= periodo) {
            last_log = millis();
            float freq_step = vel_cmd_rad_s * (PASSOS_POR_VOLTA / (2.0f * PI_F));
            Serial.printf("ctrl=%s | sin=%+d | pulsos=%6ld | ang=%6.2f | braco=%+6.1f | tau=%+6.4f | step=%+7.1f Hz\n",
                          controle_ativo ? "ON " : "OFF",
                          sinal_motor, encoder_count, ang, braco_deg, tau, freq_step);
        }
    }
}
