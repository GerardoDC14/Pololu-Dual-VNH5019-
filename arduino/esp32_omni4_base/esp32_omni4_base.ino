#include <Arduino.h>

#include "PPM.h"
#include "ControlOmni.h"


// ============================================================
// PPM
// ============================================================

#define PIN_PPM 27


// ============================================================
// VNH5019
// ============================================================

// Motor 1
#define M1_INA 5
#define M1_INB 18
#define M1_PWM 22

// Motor 2
#define M2_INA 17
#define M2_INB 19
#define M2_PWM 25

// Motor 3
#define M3_INA 26
#define M3_INB 21
#define M3_PWM 14

// Motor 4
#define M4_INA 32
#define M4_INB 33
#define M4_PWM 23

// EN/DIAG compartido
#define PIN_EN_DIAG 16


// ============================================================
// PWM
// ============================================================

constexpr uint32_t PWM_FRECUENCIA = 20000;
constexpr uint8_t PWM_RESOLUCION = 8;


// ============================================================
// Control
// ============================================================

constexpr int CENTRO = 1500;
constexpr int ZONA_MUERTA = 20;

// Máximo PWM utilizado
constexpr float DUTY_MAX = 1.00f;

// Actualización del control
constexpr uint32_t CONTROL_MS = 2;


// ============================================================
// Motor
// ============================================================

struct Motor
{
    uint8_t ina;
    uint8_t inb;
    uint8_t pwm;

    // Invertir polaridad lógica
    bool invertido;
};


// ============================================================
// Motores
// ============================================================

Motor motores[4] =
{
    // Rueda  INA       INB       PWM       Invertido
    /* F */ {M1_INA,     M1_INB,   M1_PWM,   false},
    /* I */ {M2_INA,     M2_INB,   M2_PWM,   true },
    /* T */ {M3_INA,     M3_INB,   M3_PWM,   false},
    /* D */ {M4_INA,     M4_INB,   M4_PWM,   false}
};


// ============================================================
// Detener motores
// ============================================================

void detenerMotores()
{
    for (int i = 0; i < 4; i++)
    {
        digitalWrite(
            motores[i].ina,
            LOW
        );

        digitalWrite(
            motores[i].inb,
            LOW
        );

        ledcWrite(
            motores[i].pwm,
            0
        );
    }
}


// ============================================================
// Aplicar velocidad
// ============================================================

void aplicarMotor(
    Motor &motor,
    int velocidad
)
{
    velocidad = constrain(
        velocidad,
        -255,
        255
    );


    // ========================================================
    // Invertir M2
    // ========================================================

    if (motor.invertido)
    {
        velocidad = -velocidad;
    }


    // ========================================================
    // PWM
    // ========================================================

    int duty = abs(velocidad);


    // ========================================================
    // Dirección
    // ========================================================

    if (velocidad > 0)
    {
        digitalWrite(
            motor.ina,
            HIGH
        );

        digitalWrite(
            motor.inb,
            LOW
        );
    }
    else if (velocidad < 0)
    {
        digitalWrite(
            motor.ina,
            LOW
        );

        digitalWrite(
            motor.inb,
            HIGH
        );
    }
    else
    {
        digitalWrite(
            motor.ina,
            LOW
        );

        digitalWrite(
            motor.inb,
            LOW
        );
    }


    // ========================================================
    // Aplicar PWM
    // ========================================================

    ledcWrite(
        motor.pwm,
        duty
    );
}


// ============================================================
// Configurar motores
// ============================================================

void configurarMotores()
{
    for (int i = 0; i < 4; i++)
    {
        pinMode(
            motores[i].ina,
            OUTPUT
        );

        pinMode(
            motores[i].inb,
            OUTPUT
        );

        ledcAttach(
            motores[i].pwm,
            PWM_FRECUENCIA,
            PWM_RESOLUCION
        );
    }

    detenerMotores();
}


// ============================================================
// SETUP
// ============================================================

void setup()
{
    Serial.begin(115200);

    delay(500);

    Serial.println();
    Serial.println("======================================");
    Serial.println("      ROBOT OMNIDIRECCIONAL");
    Serial.println("======================================");

    Serial.println("Inicializando PPM...");

    ppm::comenzar(PIN_PPM);

    Serial.println("PPM OK");


    // ========================================================
    // EN / DIAG
    // ========================================================

    pinMode(
        PIN_EN_DIAG,
        INPUT
    );


    // ========================================================
    // Motores
    // ========================================================

    configurarMotores();

    Serial.println("Motores OK");
    Serial.println("Sistema listo.");
}


// ============================================================
// LOOP
// ============================================================

void loop()
{
    static uint32_t ultimoControl = 0;
    static uint32_t ultimoPrint = 0;

    static uint16_t canales[ppm::MAX_CHANNELS] =
    {
        1500,
        1500,
        1500,
        1500,
        1000,
        1500
    };

    static uint8_t cantidad = 0;


    uint32_t ahora = millis();


    // ========================================================
    // Leer PPM
    // ========================================================

    uint16_t nuevosCanales[ppm::MAX_CHANNELS] = {0};
    uint8_t nuevaCantidad = 0;


    if (ppm::leer(
        nuevosCanales,
        nuevaCantidad
    ))
    {
        if (nuevaCantidad >= 5)
        {
            cantidad = nuevaCantidad;

            for (uint8_t i = 0; i < cantidad; i++)
            {
                canales[i] = nuevosCanales[i];
            }
        }
    }


    // ========================================================
    // Control cada 2 ms
    // ========================================================

    if (
        ahora - ultimoControl
        < CONTROL_MS
    )
    {
        return;
    }

    ultimoControl = ahora;


    // ========================================================
    // Verificar canales
    // ========================================================

    if (cantidad < 5)
    {
        detenerMotores();
        return;
    }


    // ========================================================
    // CH1 = lateral
    // ========================================================

    float lateral = omni::eje(
        canales[3],
        1000,
        CENTRO,
        2000,
        ZONA_MUERTA
    );


    // ========================================================
    // CH2 = avance / reversa
    // ========================================================

    float avance = omni::eje(
        canales[2],
        1000,
        CENTRO,
        2000,
        ZONA_MUERTA
    );


    // ========================================================
    // CH3 = giro
    // ========================================================

    float giro = omni::eje(
        canales[0],
        1000,
        CENTRO,
        2000,
        ZONA_MUERTA
    );


    // ========================================================
    // CH5 = habilitación
    // ========================================================

    bool habilitado =
        canales[4] > 1700;


    // ========================================================
    // Seguridad
    // ========================================================

    if (!habilitado)
    {
        detenerMotores();
        return;
    }


    // ========================================================
    // Cinemática
    // ========================================================

    float salida[4];

    omni::mezclar(
        avance,
        lateral,
        giro,
        salida
    );


    // ========================================================
    // Convertir a PWM
    // ========================================================

    int objetivo[4];

    for (int i = 0; i < 4; i++)
    {
        objetivo[i] =
            (int)(
                salida[i] *
                255.0f *
                DUTY_MAX
            );
    }


    // ========================================================
    // Aplicar motores
    // ========================================================

    for (int i = 0; i < 4; i++)
    {
        aplicarMotor(
            motores[i],
            objetivo[i]
        );
    }


    // ========================================================
    // DEBUG
    // ========================================================

    if (
        ahora - ultimoPrint
        >= 250
    )
    {
        ultimoPrint = ahora;


        Serial.print("CH1=");
        Serial.print(canales[0]);

        Serial.print(" | CH2=");
        Serial.print(canales[1]);

        Serial.print(" | CH3=");
        Serial.print(canales[2]);

        Serial.print(" | CH4=");
        Serial.print(canales[3]);

        Serial.print(" | CH5=");
        Serial.print(canales[4]);

        Serial.print(" | ENABLE=");
        Serial.print(habilitado ? "ON" : "OFF");


        Serial.print(" | A=");
        Serial.print(avance, 2);

        Serial.print(" L=");
        Serial.print(lateral, 2);

        Serial.print(" G=");
        Serial.print(giro, 2);


        Serial.print(" | M=");

        for (int i = 0; i < 4; i++)
        {
            Serial.print(objetivo[i]);

            if (i < 3)
                Serial.print(",");
        }

        Serial.println();
    }
}
