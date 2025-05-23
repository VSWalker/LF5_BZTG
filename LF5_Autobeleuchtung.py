import RPi.GPIO as GPIO
import time
import dht11  # DHT11-Modul (Bibliothek)

# BCM-Nummern verwenden
GPIO.setmode(GPIO.BCM)

# Temperatur Sensor GPIO PIN Belegung
TEMP_LED_PIN = 27  # LED für Temperaturanzeige (leuchtet ab 25 Grad)
TEMP_SENSOR_PIN = 17  # DHT11-Sensor 

# LED/BUTTON GPIO PIN Belegung
LED_PIN = 12 # Rücklicht
LED_PIN2 = 25 # Frontlicht
LED_PINL = 21 # Blinker Links
LED_PINR = 20 # Blinker Rechts
BUTTON_PIN = 16 # Taster für Tagfahrlicht
BUTTON_PINL = 23 # Taster für Blinker Links
BUTTON_PINR = 24 # Taster für Blinker Rechts
BUTTON_BUZZER = 19 # Taster für Radio
BUZZER_PIN = 13 # Radio

# Temperatur Sensor als Ausgang festlegen
GPIO.setup(TEMP_LED_PIN, GPIO.OUT)  

# Setups (PINS als Ausgang festlegen)
GPIO.setup(LED_PIN, GPIO.OUT)
GPIO.setup(LED_PIN2, GPIO.OUT)
GPIO.setup(LED_PINL, GPIO.OUT)
GPIO.setup(LED_PINR, GPIO.OUT)
GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(BUTTON_PINL, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(BUTTON_PINR, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(BUTTON_BUZZER, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(BUZZER_PIN, GPIO.OUT)

# Buzzer PWM starten
buzzer_pwm = GPIO.PWM(BUZZER_PIN, 1000)

# DHT11-Sensor starten
sensor = dht11.DHT11(pin=TEMP_SENSOR_PIN)

# Frequenz (Melodie) von der Hupe
def play_melody():
    melody = [500, 600, 700, 800, 700, 600, 500]
    for freq in melody:
        buzzer_pwm.ChangeFrequency(freq)
        buzzer_pwm.start(10)
        time.sleep(0.3)
        buzzer_pwm.stop()
        time.sleep(0.1)

print("Warte auf Tastendrücke...")



try:
    last_blink_time_l = time.time() # Zeit bis Blinker angeht
    last_blink_time_r = time.time() # Zeit bis Blinker angeht
    led_state_l = False # LEDS Dauerhaft aus bis Knopfdruck
    led_state_r = False # LEDS Dauerhaft aus bis Knopfdruck


    # Temperatur Funktion zum messen
    last_temp_check = 0

    while True:
        current_time = time.time()

        if current_time - last_temp_check >= 2.0: # Temperatur alle 2 Sekunden prüfen
            result = sensor.read() # Liest Temperatur Daten vom Sensor
            if result.is_valid(): # Prüft die Gültigkeit der Sensor Daten
                temperature = result.temperature # Speichert gemessene Temperatur
                print(f"Aktuelle Temperatur: {temperature}°C") # Printet die Temperatur
                if temperature >= 25: 
                    GPIO.output(TEMP_LED_PIN, GPIO.HIGH) # Über 25 Grad LED an
                else:
                    GPIO.output(TEMP_LED_PIN, GPIO.LOW) # Unter 25 Grad LED aus
            last_temp_check = current_time # Zeitpunkt der letzten Temperaturüberprüfung setzen (Das es alle 2 Sekunden ausgeführt wird)

        # Tagfahrlicht Taster gedrückt (LED_PIN & LED_PIN2 an)
        if GPIO.input(BUTTON_PIN) == GPIO.LOW: # Prüft ob Taster gedrückt ist (Wenn gedrückt LEDS AN)
            print("Tagfahrlicht an")
            GPIO.output(LED_PIN, GPIO.HIGH) # LEDS einschalten
            GPIO.output(LED_PIN2, GPIO.HIGH) # LEDS einschalten
        else:                                # Wenn Taster nicht gedrückt LEDS AUS
            GPIO.output(LED_PIN, GPIO.LOW)
            GPIO.output(LED_PIN2, GPIO.LOW)

        # Linker Blinker Taster gedrückt (LED_PINL blinkt)
        if GPIO.input(BUTTON_PINL) == GPIO.LOW:
            print("links blinken")
            if current_time - last_blink_time_l >= 1.0:
                led_state_l = not led_state_l
                GPIO.output(LED_PINL, led_state_l)
                last_blink_time_l = current_time
        else:
            GPIO.output(LED_PINL, GPIO.LOW)
            led_state_l = False
            last_blink_time_l = current_time

        # Rechter Blinker Taster gedrückt (LED_PINR blinkt)
        if GPIO.input(BUTTON_PINR) == GPIO.LOW:
            print("rechts blinken")
            if current_time - last_blink_time_r >= 1.0:
                led_state_r = not led_state_r
                GPIO.output(LED_PINR, led_state_r)
                last_blink_time_r = current_time
        else:
            GPIO.output(LED_PINR, GPIO.LOW)
            led_state_r = False
            last_blink_time_r = current_time

        # Prüfen, ob der Buzzer-Taster gedrückt wurde
        if GPIO.input(BUTTON_BUZZER) == GPIO.LOW:
            print("Radio an, Melodie wird abgespielt")
            play_melody()

        time.sleep(0.05)

except KeyboardInterrupt:
    print("\nBeende...")

finally:
    buzzer_pwm.stop()
    GPIO.cleanup() # GPIO-Pins in ihren Ursprungszustand zurückzusetzen
