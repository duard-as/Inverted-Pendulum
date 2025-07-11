// ===== CONFIGURACIÓN DE PINES DEL PUENTE H (L298N) =====
// Canal A
const int ENA = 9;    // PWM Motor A
const int IN1 = 8;    // Dirección 1 Motor A
const int IN2 = 7;    // Dirección 2 Motor A

// Canal B
const int ENB = 6;    // PWM Motor B
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

// Ganancias del controlador PID para el ángulo
float Kp = -68.7;  // Proporcional
float Ki = -42.3;  // Integral
float Kd = -16.5;  // Derivativo

// Ganancias del controlador de espacio de estados para posición
float K[4] = {5.0196, 10.2530, 12.4430, 6.0000};  // [K1, K2, K3, K4]

// Factor para ajustar la referencia de posición basada en u_pid
float factor = 0.1;  // Ajustar según sea necesario

// Vector de estado: [ángulo (°), velocidad angular (°/s), posición (cm), velocidad (cm/s)]
float x[4] = {0, 0, 0, 0};

// Referencia original de posición (cm)
float ref_position_original = 0.0;

// Referencia de ángulo (grados)
float ref_angle = 0.0;

// Variables para el PID
float integral = 0;
float prev_error = 0;

// Tiempo de control (100Hz)
unsigned long lastTime = 0;
const float dt = 0.01;                    // Intervalo de control (10 ms)

// ===== FUNCIONES DE INTERRUPCIÓN PARA ENCODERS =====
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

// ===== SETUP =====
void setup() {
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(pinA_angle, INPUT_PULLUP);
  pinMode(pinB_angle, INPUT_PULLUP);
  pinMode(pinA_pos, INPUT_PULLUP);
  pinMode(pinB_pos, INPUT_PULLUP);

  pinMode(limitSwitchPos, INPUT_PULLUP);
  pinMode(limitSwitchNeg, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(pinA_angle), isrEncoderA_angle, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pinB_angle), isrEncoderB_angle, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pinA_pos), isrEncoderA_pos, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pinB_pos), isrEncoderB_pos, CHANGE);

  Serial.begin(115200);
  Serial.println("Controlador PID + SS inicializado");
}

// ===== LOOP PRINCIPAL =====
void loop() {
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastTime) / 1000.0;

  if (deltaTime >= dt) {
    noInterrupts();
    long current_angle = angle_count;
    long current_position = position_count;
    interrupts();

    float angle = current_angle * degPerCount;
    angle = fmod(angle + 360.0, 360.0);
    if (angle > 180.0) angle -= 360.0;

    float position = current_position * cmPerCount;

    float ang_velocity = (angle - x[0]) / dt;
    float velocity = (position - x[2]) / dt;

    x[0] = angle;        // Ángulo en grados
    x[1] = ang_velocity; // Velocidad angular en °/s
    x[2] = position;     // Posición en cm
    x[3] = velocity;     // Velocidad en cm/s

    if (digitalRead(limitSwitchPos) == LOW || digitalRead(limitSwitchNeg) == LOW) {
      stopMotors();
      Serial.println("¡Final de carrera alcanzado!");
      lastTime = currentTime;
      return;
    }

    // **Control PID para el ángulo**
    float error = ref_angle - x[0];        // Error: ángulo deseado - ángulo actual
    integral += error * dt;                // Término integral
    float derivative = (error - prev_error) / dt; // Término derivativo
    float u_pid = Kp * error + Ki * integral + Kd * derivative; // Señal PID
    prev_error = error;

    // Ajustar la referencia de posición basada en u_pid
    float ref_position = ref_position_original + factor * u_pid;

    // Vector de referencia: [ref_angle, 0, ref_position, 0]
    float x_ref[4] = {ref_angle, 0.0, ref_position, 0.0};

    // Calcular la señal de control del espacio de estados
    float u_ss = 0.0;
    for (int i = 0; i < 4; i++) {
      u_ss -= K[i] * (x[i] - x_ref[i]);
    }

    // La señal total es simplemente u_ss, ya que el PID ya influye en x_ref
    float u_total = u_ss;

    int pwm = constrain(abs(u_total), 80, 255);

    if (u_total > 0) {
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
    } else {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
    }

    analogWrite(ENA, pwm);
    analogWrite(ENB, pwm);

    Serial.print(x[0], 2); Serial.print(',');
    Serial.print(x[1], 2); Serial.print(',');
    Serial.print(x[2], 2); Serial.print(',');
    Serial.println(x[3], 2);

    lastTime = currentTime;
  }
}

// ===== FUNCIONES AUXILIARES =====
void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}