const int piezoPin = 34; // GPIO34
const int ledPin = 2;    // LED onboard do ESP32

int threshold = 120;

void setup() {

  Serial.begin(115200);

  pinMode(ledPin, OUTPUT);

  Serial.println("Piezo iniciado");
}

void loop() {

  int valor = analogRead(piezoPin);

  Serial.println(valor);

  if (valor > threshold) {

    digitalWrite(ledPin, HIGH);

    Serial.print("Impacto: ");
    Serial.println(valor);

    delay(100);

  } else {

    digitalWrite(ledPin, LOW);
  }

  delay(5);
}