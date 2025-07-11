// ===== CONFIGURACIÓN DE PINES DEL PUENTE H (L298N) =====
// Canal A
const int ENA = 9;    // PWM Motor A (pin PWM)
const int IN1 = 8;    // Dirección 1 Motor A
const int IN2 = 7;    // Dirección 2 Motor A

// Canal B
const int ENB = 6;    // PWM Motor B (pin PWM)
const int IN3 = 5;    // Dirección 1 Motor B
const int IN4 = 4;    // Dirección 2 Motor B

// ===== ENCODERS Y FINALES DE CARRERA =====
// Encoder del péndulo (ángulo)
const byte pinA_angle = 2;   // Interrupción 0 en Mega
const byte pinB_angle = 3;   // Interrupción 1 en Mega

// Encoder del carro (posición)
const byte pinA_pos = 19;    // Interrupción 4 en Mega
const byte pinB_pos = 18;    // Interrupción 5 en Mega

// Finales de carrera
const byte limitSwitchPos = A0;
const byte limitSwitchNeg = A1;

// ===== PARÁMETROS DEL SISTEMA =====
const long PPR = 600;                     // Pulsos por revolución del encoder
const long countsPerRev = PPR * 4;        // Cuadratura (4 pulsos por ciclo)
const float degPerCount = 360.0 / countsPerRev;  // Grados por pulso
const float cmPerCount = 15.0 / countsPerRev;    // 15 cm por vuelta

// ===== VARIABLES GLOBALES =====
volatile long angle_count = 0;            // Cuenta del encoder de ángulo
volatile long position_count = 0;         // Cuenta del encoder de posición

// Ganancia LQR: [K1_theta, K2_theta_dot, K3_x, K4_x_dot]
float K_lqr[4] = {41.9561, 7.7892, 3.5355, 4.6583};  // Ajustar según modelo

// Vector de estado: [ángulo (°), velocidad angular (°/s), posición (cm), velocidad (cm/s)]
float x[4] = {0, 0, 0, 0};

// Tiempo de control (100Hz)
unsigned long lastTime = 0;
const float dt = 0.01;                    // Intervalo de control (10 ms)

// ===== SETUP =====
void setup() {
  // Configuración de pines del puente H
  // Canal A
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  
  // Canal B
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Encoders (pull-up interno)
  pinMode(pinA_angle, INPUT_PULLUP);
  pinMode(pinB_angle, INPUT_PULLUP);
  pinMode(pinA_pos, INPUT_PULLUP);
  pinMode(pinB_pos, INPUT_PULLUP);

  // Finales de carrera
  pinMode(limitSwitchPos, INPUT_PULLUP);
  pinMode(limitSwitchNeg, INPUT_PULLUP);

  // Interrupciones para encoders
  attachInterrupt(digitalPinToInterrupt(pinA_angle), isrEncoderA_angle, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pinB_angle), isrEncoderB_angle, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pinA_pos), isrEncoderA_pos, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pinB_pos), isrEncoderB_pos, CHANGE);

  Serial.begin(115200);
  Serial.println("Controlador LQR inicializado");
}

// ===== LOOP PRINCIPAL =====
void loop() {
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastTime) / 1000.0;

  if (deltaTime >= dt) {
    // Leer encoders (sin interrupciones)
    noInterrupts();
    long current_angle = angle_count;
    long current_position = position_count;
    interrupts();

    // Calcular ángulo (-180° a 180°)
    float angle = current_angle * degPerCount;
    angle = fmod(angle + 360.0, 360.0);
    if (angle > 180.0) angle -= 360.0;

    // Calcular posición (cm)
    float position = current_position * cmPerCount;

    // Calcular velocidades (sin filtro)
    float ang_velocity = (angle - x[0]) / dt;  // Velocidad angular (°/s)
    float velocity = (position - x[2]) / dt;   // Velocidad lineal (cm/s)

    // Actualizar vector de estado
    x[0] = angle;            // Ángulo actual
    x[1] = ang_velocity;     // Velocidad angular
    x[2] = position;         // Posición actual
    x[3] = velocity;         // Velocidad lineal

    // Verificar finales de carrera (detener motores si se activan)
    if (digitalRead(limitSwitchPos) == LOW || digitalRead(limitSwitchNeg) == LOW) {
      stopMotors();
      Serial.println("¡Final de carrera alcanzado!");
      lastTime = currentTime;
      return;
    }

    // ===== CONTROL LQR =====
    // u = K1*theta + K2*theta_dot + K3*x + K4*x_dot
    float u = K_lqr[0] * x[0]    // K1 * ángulo
            + K_lqr[1] * x[1]    // K2 * velocidad angular
            + K_lqr[2] * x[2]    // K3 * posición
            + K_lqr[3] * x[3];   // K4 * velocidad

    // Aplicar señal a los motores (mínimo PWM = 50 para vencer fricción)
    int pwm = constrain(abs(u), 80, 255);

    // Dirección de los motores (depende del signo de u)
    if (u > 0) {
      // Motor (avance)
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      // Motor (avance - sincronizado con A)
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
    } else {
      // Motor (retroceso)
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      // Motor (retroceso - sincronizado con A)
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
    }
    
    // Escribir PWM
    analogWrite(ENA, pwm);
    analogWrite(ENB, pwm);

    // Debug por Serial
    // Formato CSV
    Serial.print(x[0], 2);
    Serial.print(',');
    Serial.print(x[1], 2);
    Serial.print(',');
    Serial.print(x[2], 2);
    Serial.print(',');
    Serial.println(x[3], 2);


    // Formato Serial Monitor
    //Serial.print("Ángulo: "); Serial.print(x[0], 1); Serial.print("° | ");
    // Serial.print("VelAng: "); Serial.print(x[1], 1); Serial.print("°/s | ");
    // Serial.print("Pos: "); Serial.print(x[2], 1); Serial.print("cm | ");
    // Serial.print("Vel: "); Serial.print(x[3], 1); Serial.print("cm/s | ");
    // Serial.print("PWM: "); Serial.println(pwm);

    lastTime = currentTime;
  }
}

// ===== FUNCIONES AUXILIARES =====
void stopMotors() {
  // Detener motor
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

// ===== INTERRUPCIONES PARA ENCODERS =====
// Encoder de ángulo (péndulo)
void isrEncoderA_angle() {
  if (digitalRead(pinA_angle)) {
    angle_count += (digitalRead(pinB_angle) ? -1 : 1);
  } else {
    angle_count += (digitalRead(pinB_angle) ? 1 : -1);
  }
}

void isrEncoderB_angle() {
  if (digitalRead(pinB_angle)) {
    angle_count += (digitalRead(pinA_angle) ? 1 : -1);
  } else {
    angle_count += (digitalRead(pinA_angle) ? -1 : 1);
  }
}

// Encoder de posición (carro)
void isrEncoderA_pos() {
  if (digitalRead(pinA_pos)) {
    position_count += (digitalRead(pinB_pos) ? -1 : 1);
  } else {
    position_count += (digitalRead(pinB_pos) ? 1 : -1);
  }
}

void isrEncoderB_pos() {
  if (digitalRead(pinB_pos)) {
    position_count += (digitalRead(pinA_pos) ? 1 : -1);
  } else {
    position_count += (digitalRead(pinA_pos) ? -1 : 1);
  }
}