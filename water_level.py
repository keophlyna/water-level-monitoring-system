"""
Cleaned & merged version of test_stable_value.py
Integrated I2cLcd from machine_i2c_lcd.py (uses lcd_api.py)
Kept original functionality: WiFi, MQTT, Blynk, Telegram, HC-SR04, pump control, app server.
"""

import machine
import time
import network
import urequests as requests
import ujson
from machine import Pin, I2C
from umqtt.simple import MQTTClient
import _thread

# Local LCD driver (uploaded file)
from machine_i2c_lcd import I2cLcd

# ===========================
# CONFIGURATION
# ===========================

# WiFi
WIFI_SSID = "thou sini"
WIFI_PASSWORD = "077469933"

# MQTT
MQTT_BROKER = "test.mosquitto.org"
MQTT_PORT = 1883
MQTT_TOPIC_LEVEL = "tank/level"
MQTT_TOPIC_PERCENTAGE = "tank/percentage"
MQTT_TOPIC_PUMP_STATUS = "tank/pumpStatus"
MQTT_TOPIC_PUMP_CONTROL = "tank/pumpControl"
MQTT_CLIENT_ID = "esp32_water_tank"

# Blynk
BLYNK_TOKEN = "NFvHYoean48rBJuU5Vqm68aBt9lXBQSS"
BLYNK_API = "http://blynk.cloud/external/api"

# Telegram
TELEGRAM_BOT_TOKEN = "8587546082:AAEFQd4nCYqrmEia8L3-53GfQl9ArfZBwlQ"
TELEGRAM_CHAT_ID = "-4993754511"

# Tank
TANK_HEIGHT = 14.2         # cm
SENSOR_OFFSET = 5        # cm (sensor mounting offset)

# Thresholds (percentage)
LOW_LEVEL = 20
HIGH_LEVEL = 90
CRITICAL_LOW = 15
CRITICAL_HIGH = 95

# Pump Hysteresis (AUTO mode)
PUMP_MIN_ON_TIME = 30    # seconds
PUMP_MIN_OFF_TIME = 30   # seconds

# Pins (adjust if needed)
TRIG_PIN = 5
ECHO_PIN = 18
RELAY_PIN = 13
SDA_PIN = 21
SCL_PIN = 22

# Sensor and timing
SENSOR_SAMPLES = 8
SENSOR_MAX_DISTANCE = 400
SENSOR_MIN_DISTANCE = 2

POLL_BLYNK_MS = 500        # poll V2 every 500ms
PUSH_DATA_MS = 10000       # push V0,V1 every 10s
MQTT_PUBLISH_MS = 3000     # publish MQTT every 3s
APP_SERVER_PORT = 80

# LCD config
I2C_ADDR = 0x27
LCD_ROWS = 2
LCD_COLS = 16

# Debug
DEBUG = True

# ===========================
# GLOBALS
# ===========================

water_level_cm = 0.0
water_percentage = 0.0
pump_state = False
pump_last_change_time = 0
auto_mode = True

last_alert_time = 0
alert_cooldown = 300000  # ms

mqtt_client = None
lcd = None

sensor_stable = False
EMPTY_CONFIDENCE_DISTANCE = 20  # cm threshold to treat as empty

# ===========================
# HARDWARE INIT
# ===========================

trig = Pin(TRIG_PIN, Pin.OUT)
echo = Pin(ECHO_PIN, Pin.IN)
relay = Pin(RELAY_PIN, Pin.OUT)
relay.value(0)

# Initialize I2C and LCD
try:
    i2c = I2C(0, scl=Pin(SCL_PIN), sda=Pin(SDA_PIN), freq=100000)
    lcd = I2cLcd(i2c, I2C_ADDR, LCD_ROWS, LCD_COLS)
    print("✓ LCD Ready")
except Exception as e:
    print("LCD initialization failed:", e)
    lcd = None

# ===========================
# WIFI
# ===========================

def wifi_connect(timeout_ms=15000):
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    if wlan.isconnected():
        return True

    print("Connecting WiFi...")
    if lcd:
        try:
            lcd.clear()
            lcd.move_to(0, 0)
            lcd.putstr("Connecting WiFi")
        except Exception:
            pass

    wlan.connect(WIFI_SSID, WIFI_PASSWORD)
    t0 = time.ticks_ms()
    while not wlan.isconnected():
        if time.ticks_diff(time.ticks_ms(), t0) > timeout_ms:
            print("✗ WiFi timeout")
            return False
        time.sleep_ms(200)

    ip = wlan.ifconfig()[0]
    print(f"✓ WiFi: {ip}")
    if lcd:
        try:
            lcd.clear()
            lcd.move_to(0, 0)
            lcd.putstr("WiFi Connected")
            lcd.move_to(0, 1)
            lcd.putstr(ip[:16])
            time.sleep(2)
        except Exception:
            pass
    return True

def wifi_ensure(wlan):
    """Attempt to keep WiFi alive; returns boolean connected."""
    if not wlan.isconnected():
        try:
            wlan.disconnect()
        except:
            pass
        wlan.active(True)
        wlan.connect(WIFI_SSID, WIFI_PASSWORD)
        t0 = time.ticks_ms()
        while not wlan.isconnected():
            if time.ticks_diff(time.ticks_ms(), t0) > 10000:
                break
            time.sleep_ms(200)
    return wlan.isconnected()

# ===========================
# MQTT
# ===========================

def mqtt_callback(topic, msg):
    global auto_mode
    try:
        msg_str = msg.decode().strip().upper()
    except:
        msg_str = str(msg).upper()

    print(f"MQTT << {msg_str}")

    if msg_str == "ON" and not auto_mode:
        control_pump(True, "mqtt")
    elif msg_str == "OFF" and not auto_mode:
        control_pump(False, "mqtt")
    elif msg_str == "AUTO":
        auto_mode = True
        update_blynk_mode()
    elif msg_str == "MANUAL":
        auto_mode = False
        update_blynk_mode()

def connect_mqtt():
    global mqtt_client
    try:
        print("Connecting MQTT...")
        mqtt_client = MQTTClient(MQTT_CLIENT_ID, MQTT_BROKER, port=MQTT_PORT)
        mqtt_client.set_callback(mqtt_callback)
        mqtt_client.connect()
        mqtt_client.subscribe(MQTT_TOPIC_PUMP_CONTROL)
        print("✓ MQTT OK")
        return True
    except Exception as e:
        print("✗ MQTT Failed:", e)
        mqtt_client = None
        return False

def publish_mqtt():
    global mqtt_client
    try:
        if mqtt_client:
            mqtt_client.check_msg()
            mqtt_client.publish(MQTT_TOPIC_LEVEL, str(water_level_cm))
            mqtt_client.publish(MQTT_TOPIC_PERCENTAGE, str(water_percentage))
            mqtt_client.publish(MQTT_TOPIC_PUMP_STATUS, "ON" if pump_state else "OFF")
    except Exception:
        pass

# ===========================
# BLYNK
# ===========================

def _safe_close(r):
    try:
        r.close()
    except:
        pass

def blynk_get_v2():
    """Get pump button state (V2). Returns 0,1 or None."""
    url = f"{BLYNK_API}/get?token={BLYNK_TOKEN}&V2"
    r = None
    try:
        r = requests.get(url, timeout=2)
        if r.status_code != 200:
            return None
        txt = r.text.strip()
        try:
            obj = ujson.loads(txt)
        except:
            obj = txt

        val = obj[0] if isinstance(obj, list) and obj else obj
        # normalize
        if isinstance(val, bool):
            return 1 if val else 0
        if isinstance(val, (int, float)):
            return 1 if int(val) != 0 else 0
        if isinstance(val, str):
            s = val.strip().strip('"').lower()
            if s in ("1", "true", "on"):
                return 1
            if s in ("0", "false", "off"):
                return 0
            try:
                return 1 if int(s) != 0 else 0
            except:
                return None
        return None
    except:
        return None
    finally:
        if r:
            _safe_close(r)

def blynk_get_v3():
    """Get mode button state (V3). Returns 0=AUTO,1=MANUAL or None."""
    url = f"{BLYNK_API}/get?token={BLYNK_TOKEN}&V3"
    r = None
    try:
        r = requests.get(url, timeout=2)
        if r.status_code != 200:
            return None
        txt = r.text.strip()
        try:
            obj = ujson.loads(txt)
        except:
            obj = txt

        val = obj[0] if isinstance(obj, list) and obj else obj
        if isinstance(val, bool):
            return 1 if val else 0
        if isinstance(val, (int, float)):
            return 1 if int(val) != 0 else 0
        if isinstance(val, str):
            s = val.strip().strip('"').lower()
            if s in ("1", "true", "on", "manual"):
                return 1
            if s in ("0", "false", "off", "auto"):
                return 0
            try:
                return 1 if int(s) != 0 else 0
            except:
                return None
        return None
    except:
        return None
    finally:
        if r:
            _safe_close(r)

def blynk_update_data():
    """Push V0 (percentage), V1 (cm), V2 (pump state)."""
    url = f"{BLYNK_API}/batch/update?token={BLYNK_TOKEN}&V0={water_percentage}&V1={water_level_cm}&V2={1 if pump_state else 0}"
    r = None
    try:
        r = requests.get(url, timeout=3)
        if r.status_code == 200 and DEBUG:
            print(f"📱 Blynk: {water_percentage}% | {water_level_cm}cm | Pump={pump_state}")
    except:
        pass
    finally:
        if r:
            _safe_close(r)

def update_blynk_mode():
    """Update V3 to match current auto/manual mode."""
    url = f"{BLYNK_API}/update?token={BLYNK_TOKEN}&V3={1 if not auto_mode else 0}"
    r = None
    try:
        r = requests.get(url, timeout=2)
    except:
        pass
    finally:
        if r:
            _safe_close(r)

# ===========================
# SENSOR (HC-SR04)
# ===========================

def measure_distance():
    """
    HC-SR04 measurement using time.ticks_us() and timeouts.
    Returns distance in cm or -1 on failure.
    """
    try:
        trig.value(0)
        time.sleep_us(2)
        trig.value(1)
        time.sleep_us(10)
        trig.value(0)

        start_wait = time.ticks_us()
        timeout_us = 35000  # 35ms
        while echo.value() == 0:
            if time.ticks_diff(time.ticks_us(), start_wait) > timeout_us:
                if DEBUG: print("⏱️ measure_distance: timeout waiting for rising edge")
                return -1

        start = time.ticks_us()

        while echo.value() == 1:
            if time.ticks_diff(time.ticks_us(), start) > 30000:
                if DEBUG: print("⏱️ measure_distance: timeout waiting for falling edge")
                return -1

        end = time.ticks_us()
        duration = time.ticks_diff(end, start)  # us
        distance = (duration * 0.0343) / 2.0

        if DEBUG:
            print(f"🔊 echo ok: duration={duration}us dist={distance:.1f}cm")

        if distance < SENSOR_MIN_DISTANCE or distance > SENSOR_MAX_DISTANCE:
            if DEBUG: print(f"⚠ measure_distance: out of range {distance:.1f}cm")
            return -1

        return distance
    except Exception as e:
        if DEBUG: print("❌ measure_distance exception:", e)
        return -1

def calculate_water_level():
    """
    Smoothing, deadband, drop detection. Updates globals:
    - water_level_cm, water_percentage, sensor_stable
    """
    global water_level_cm, water_percentage, sensor_stable

    readings = []
    for _ in range(SENSOR_SAMPLES):
        d = measure_distance()
        if d > 0:
            readings.append(d)
        time.sleep_ms(50)

    if not readings:
        water_level_cm = 0.0
        water_percentage = 0.0
        sensor_stable = False
        return True

    readings.sort()
    if len(readings) >= 5:
        readings = readings[1:-1]

    avg_dist = sum(readings) / len(readings)
    max_distance = SENSOR_OFFSET + TANK_HEIGHT

    # CUSTOM EMPTY DETECTION — USE REAL SENSOR BASELINE
    REAL_EMPTY_DIST = 14.5  # your sensor empty reading

    # If distance is close to empty (±1 cm), treat tank as EMPTY
    if abs(avg_dist - REAL_EMPTY_DIST) <= 1.0:
        raw_cm = 0.0
        raw_pct = 0.0
    else:
        water_depth = max_distance - avg_dist
        water_depth = max(0.0, min(water_depth, TANK_HEIGHT))
        raw_cm = round(water_depth, 1)
        raw_pct = round((water_depth / TANK_HEIGHT) * 100.0, 1)


    smoothing = 0.85
    if sensor_stable:
        smooth_cm = (water_level_cm * smoothing) + (raw_cm * (1 - smoothing))
        smooth_pct = (water_percentage * smoothing) + (raw_pct * (1 - smoothing))
    else:
        smooth_cm = raw_cm
        smooth_pct = raw_pct

    smooth_cm = round(smooth_cm, 1)
    smooth_pct = round(smooth_pct, 1)

    # Deadband: ignore small percent changes when stable
    if sensor_stable and abs(smooth_pct - water_percentage) < 3:
        return True

    # Limit jump per cycle
    if sensor_stable:
        max_step = 3
        diff = smooth_pct - water_percentage
        if diff > max_step:
            smooth_pct = water_percentage + max_step
        elif diff < -max_step:
            smooth_pct = water_percentage - max_step

    water_level_cm = smooth_cm
    water_percentage = smooth_pct
    sensor_stable = True

    if DEBUG:
        print(f"✅ FIXED: avg={avg_dist:.1f}cm raw={raw_cm}cm/{raw_pct}% smooth={smooth_cm}cm/{smooth_pct}%")

    return True

# ===========================
# PUMP CONTROL
# ===========================

def control_pump(state, source="auto"):
    global pump_state, pump_last_change_time

    now = time.time()
    elapsed = now - pump_last_change_time

    if source == "auto":
        if pump_state and not state:
            if elapsed < PUMP_MIN_ON_TIME:
                if DEBUG: print("Hysteresis: skipping OFF (min on time)")
                return
        if not pump_state and state:
            if elapsed < PUMP_MIN_OFF_TIME:
                if DEBUG: print("Hysteresis: skipping ON (min off time)")
                return
    else:
        if DEBUG:
            print(f"⚡ INSTANT control from {source}")

    pump_state = state
    pump_last_change_time = now
    try:
        relay.value(1 if state else 0)
    except Exception:
        pass

    print(f"{'🟢' if state else '🔴'} Pump {'ON' if state else 'OFF'} [{source}]")

def auto_pump_logic():
    if water_percentage <= LOW_LEVEL and not pump_state:
        control_pump(True, "auto")
    elif water_percentage >= HIGH_LEVEL and pump_state:
        control_pump(False, "auto")

# ===========================
# LCD UPDATES
# ===========================

def update_lcd():
    if not lcd:
        return
    try:
        lcd.clear()
        lcd.move_to(0, 0)
        lcd.putstr("Water: {}%".format(water_percentage))
        lcd.move_to(0, 1)
        pump_txt = "ON " if pump_state else "OFF"
        mode_txt = "AUTO" if auto_mode else "MAN"
        # Format to fit columns
        line2 = f"Pump:{pump_txt} {mode_txt}"
        lcd.putstr(line2[:LCD_COLS])
    except Exception as e:
        if DEBUG: print("LCD error:", e)

# ===========================
# TELEGRAM ALERTS
# ===========================

def send_telegram(msg):
    global last_alert_time
    now = time.ticks_ms()
    if time.ticks_diff(now, last_alert_time) < alert_cooldown:
        return

    try:
        encoded = msg.replace(" ", "%20").replace(":", "%3A").replace("\n", "%0A")
        url = f"https://api.telegram.org/bot{TELEGRAM_BOT_TOKEN}/sendMessage?chat_id={TELEGRAM_CHAT_ID}&text={encoded}"
        r = requests.get(url, timeout=5)
        if r.status_code == 200:
            last_alert_time = now
        r.close()
    except Exception:
        pass

def check_alerts():
    if water_percentage <= CRITICAL_LOW:
        send_telegram(f"⚠️ LOW: {water_percentage}%")
    elif water_percentage >= CRITICAL_HIGH:
        send_telegram(f"⚠️ HIGH: {water_percentage}%")

# ===========================
# APP SERVER
# ===========================

def handle_request(conn):
    global auto_mode
    try:
        req = conn.recv(1024).decode()
        if "GET /status" in req:
            res = ujson.dumps({
                "level_cm": water_level_cm,
                "percentage": water_percentage,
                "pump": pump_state,
                "mode": "auto" if auto_mode else "manual"
            })
        elif "GET /pump/on" in req:
            if not auto_mode:
                control_pump(True, "app")
            res = '{"ok":true}'
        elif "GET /pump/off" in req:
            if not auto_mode:
                control_pump(False, "app")
            res = '{"ok":true}'
        elif "GET /mode/auto" in req:
            auto_mode = True
            update_blynk_mode()
            res = '{"ok":true}'
        elif "GET /mode/manual" in req:
            auto_mode = False
            update_blynk_mode()
            res = '{"ok":true}'
        else:
            res = '{"error":"unknown"}'

        conn.send("HTTP/1.1 200 OK\r\nContent-Type: application/json\r\n\r\n")
        conn.send(res)
    except Exception:
        pass
    finally:
        try:
            conn.close()
        except:
            pass

def app_server():
    import socket
    addr = socket.getaddrinfo('0.0.0.0', APP_SERVER_PORT)[0][-1]
    s = socket.socket()
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(addr)
    s.listen(1)
    print(f"📱 App server: {APP_SERVER_PORT}")
    while True:
        try:
            conn, _ = s.accept()
            handle_request(conn)
        except Exception:
            pass

# ===========================
# MAIN LOOP
# ===========================

def main():
    global pump_last_change_time, auto_mode

    print("\n" + "="*40)
    print("🌊 Water Monitor - INSTANT Control")
    print("="*40)

    # Startup LCD message
    if lcd:
        try:
            lcd.clear()
            lcd.move_to(0, 0)
            lcd.putstr("Water Monitor")
            lcd.move_to(0, 1)
            lcd.putstr("Starting...")
        except Exception:
            pass

    time.sleep(2)

    # WiFi
    if not wifi_connect():
        print("❌ No WiFi")
        while True:
            time.sleep(1)

    wlan = network.WLAN(network.STA_IF)

    # MQTT
    connect_mqtt()

    # App server thread
    try:
        _thread.start_new_thread(app_server, ())
    except Exception:
        pass

    pump_last_change_time = time.time()
    send_telegram("🚀 Water Monitor Started!")

    # Timing
    next_blynk_poll = time.ticks_ms()
    next_blynk_push = time.ticks_ms()
    next_mqtt = time.ticks_ms()
    next_sensor = time.ticks_ms()
    next_alert = time.ticks_ms()

    last_v2 = None
    last_v3 = None

    print("\n🔄 Running... (V2=Pump, V3=Mode)\n")

    while True:
        try:
            wifi_ensure(wlan)
            now = time.ticks_ms()

            # 1) Poll Blynk (V2/V3)
            if time.ticks_diff(now, next_blynk_poll) >= 0:
                v3 = blynk_get_v3()
                if v3 is not None and v3 != last_v3:
                    auto_mode = (v3 == 0)
                    print(f"🎛️ Mode: {'AUTO' if auto_mode else 'MANUAL'}")
                    last_v3 = v3

                if not auto_mode:
                    v2 = blynk_get_v2()
                    if v2 is not None and v2 != last_v2:
                        print(f"📱 Blynk V2: {v2}")
                        if v2 == 1 and not pump_state:
                            control_pump(True, "blynk")
                        elif v2 == 0 and pump_state:
                            control_pump(False, "blynk")
                        last_v2 = v2

                next_blynk_poll = time.ticks_add(now, POLL_BLYNK_MS)

            # 2) Sensor (every 3 seconds)
            if time.ticks_diff(now, next_sensor) >= 0:
                if calculate_water_level():
                    if auto_mode:
                        auto_pump_logic()
                    update_lcd()
                    print(f"{water_level_cm}cm ({water_percentage}%) | Pump={'ON' if pump_state else 'OFF'} | {'AUTO' if auto_mode else 'MANUAL'}")
                next_sensor = time.ticks_add(now, 3000)

            # 3) Push to Blynk
            if time.ticks_diff(now, next_blynk_push) >= 0:
                blynk_update_data()
                next_blynk_push = time.ticks_add(now, PUSH_DATA_MS)

            # 4) MQTT publish
            if time.ticks_diff(now, next_mqtt) >= 0:
                publish_mqtt()
                next_mqtt = time.ticks_add(now, MQTT_PUBLISH_MS)

            # 5) Alerts
            if time.ticks_diff(now, next_alert) >= 0:
                check_alerts()
                next_alert = time.ticks_add(now, 60000)

            time.sleep_ms(40)

        except KeyboardInterrupt:
            raise
        except Exception as e:
            print(f"❌ Error in main loop: {e}")
            time.sleep(2)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n🛑 Stopped")
        try:
            relay.value(0)
        except:
            pass
        if mqtt_client:
            try:
                mqtt_client.disconnect()
            except:
                pass
        if lcd:
            try:
                lcd.clear()
                lcd.move_to(0, 0)
                lcd.putstr("Stopped")
            except:
                pass

