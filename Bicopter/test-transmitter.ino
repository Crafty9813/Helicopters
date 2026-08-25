/*
============================================================
             RC INPUT TEST

Arduino Nano

EXPECTED:
    Stick left    -> ~1000 us
    Center        -> ~1500 us
    Stick right   -> ~2000 us
============================================================
*/

const uint8_t RC_ROLL_PIN = 3;

uint32_t lastPrintTime = 0;

const uint32_t PRINT_INTERVAL = 250;


void setup()
{
  Serial.begin(115200);

  pinMode(
    RC_ROLL_PIN,
    INPUT
  );

  Serial.println();
  Serial.println(
    "======================================"
  );

  Serial.println(
    "RC ROLL INPUT TEST"
  );

  Serial.println(
    "======================================"
  );

  Serial.println();

  Serial.println(
    "LEFT  = ~1000 us"
  );

  Serial.println(
    "CENTER = ~1500 us"
  );

  Serial.println(
    "RIGHT = ~2000 us"
  );

  Serial.println();
}


void loop()
{
  unsigned long rollUS =
    pulseIn(
      RC_ROLL_PIN,
      HIGH,
      25000
    );


  if (
    millis()
    - lastPrintTime
    >= PRINT_INTERVAL
  )
  {
    lastPrintTime =
      millis();


    Serial.print(
      "ROLL: "
    );


    if (
      rollUS >= 900 &&
      rollUS <= 2100
    )
    {
      Serial.print(
        rollUS
      );

      Serial.println(
        " us"
      );
    }
    else
    {
      Serial.println(
        "NO SIGNAL"
      );
    }
  }
}
