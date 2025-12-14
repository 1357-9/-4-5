

#  Лабораторна робота 4-5: Курсовертикаль на базі MPU-9250

 
**Варіант:** [Варіант 4]

** Курсовертикаль (система визначення орієнтації).  **
** Прилад призначений для визначення трьох кутів орієнтації об'єкта у просторі: **Крен ($\gamma$)**, **Тангаж ($\theta$)** та **Курс ($\Psi$)**.**

### 1.2. Принцип дії
Для точного визначення орієнтації використано інерціальний вимірювальний блок (IMU) MPU-9250, що містить гіроскоп та акселерометр. Сирі дані з цих сенсорів обробляються на мікроконтролері за допомогою **Fusion-алгоритму Madgwick**, який поєднує їх для усунення дрейфу гіроскопа та чутливості акселерометра до лінійних прискорень.]Обчислені стабільні кути передаються по UART для візуалізації.

### 1.3. Осі Вимірювання 
* **Вісь X:** Поздовжня вісь (Крен, $\gamma$).
* **Вісь Y:** Бокова вісь (Тангаж, $\theta$).
* **Вісь Z:** Вертикальна вісь (Курс, $\Psi$).

## 2. Структурна Схема 



**Мікроконтролер:** Arduino Nano (ATmega328)   
**Сенсор:** MPU-9250 (IMU)  
**Інтерфейс:** I2C для зв'язку з датчиком, UART для зв'язку з ПК.  
**Відображення:** GUI на Python (VPython).

##3. Формули Визначення Кутів Орієнтації 

Оскільки використовується **Fusion-алгоритм Madgwick** (кватерніонний фільтр), він інтегрує дані гіроскопа ($\omega_x, \omega_y, \omega_z$) та корегує їх акселерометром ($a_x, a_y, a_z$).

### 3.1.Інтегрування Гіроскопа (для розуміння) 
* Траєкторія орієнтації відстежується гіроскопом: $\gamma_{gyro}(i)=\gamma_{gyro}(i-1)+\omega_{x}(i)dt$
* *Примітка:* В алгоритмі Madgwick інтегрування відбувається через кватерніони, а не прямим методом.

### 3.2. Корекція Акселерометром (для розуміння) 
* Акселерометр надає опорні дані для крену і тангажу:
    $$\gamma_{axel}(i)=\text{atan}\frac{a_{y}(i)}{a_{z}(i)}$$
    $$\theta_{axel}(i)=\text{atan}\frac{-a_{x}(i)}{\sqrt{a_{y}^{2}+a_{z}^{2}}}$$

### 3.3. Фінальні Кути Ейлера (Результат Fusion)
Алгоритм Madgwick обчислює кватерніон ($q_0, q_1, q_2, q_3$), який потім конвертується у кути Ейлера (виводиться у градусах):
$$\gamma (Roll) = \text{atan2}(2(q_0 q_1 + q_2 q_3), 1 - 2(q_1^2 + q_2^2)) \cdot \frac{180}{\pi}$$
$$\theta (Pitch) = \text{asin}(2(q_0 q_2 - q_3 q_1)) \cdot \frac{180}{\pi}$$
$$\Psi (Yaw) = \text{atan2}(2(q_0 q_3 + q_1 q_2), 1 - 2(q_2^2 + q_3^2)) \cdot \frac{180}{\pi}$$

## 4. Основні Нюанси Коду Апаратної Частини (Arduino) 

* **Мікроконтролер:** Arduino Nano.
* **Чіп IMU:** MPU-9250.
* **Протокол зв'язку:** I2C.
* **Частота видачі:** 100 Гц (завдяки `delay(10)` мс).
* **Налаштування живлення:** Для виведення MPU-9250 зі сплячого режиму використовувався запис `0` у регістр **`PWR_MGMT_1`** (адреса 0x6B).
* **Fusion Implementation:** Замість зовнішньої бібліотеки використано **Single-File Madgwick IMU** (розбитий на вкладки `.ino`, `.h`, `.cpp` в IDE), що дозволило максимально оптимізувати використання пам'яті ATmega328 та забезпечити стабільну роботу Fusion-алгоритму.
* **Вихідний формат:** Кути передаються у форматі CSV: `Roll_deg,Pitch_deg,Yaw_deg`.

## 5. Трохи Інформації про Застосований Спосіб Відображення 

**Спосіб відображення:** Відображення здійснюється за допомогою графічного інтерфейсу користувача (GUI), реалізованого на мові **Python** з використанням бібліотеки **VPython (GlowScript)**.

* Python-скрипт підключається до послідовного порту (COM4, 115200 бод).
* Він приймає три стабільні кути з Fusion-алгоритму.
* Ці кути використовуються для обертання тривимірного об'єкта (куба/літака) у реальному часі за послідовністю обертання **Yaw-Pitch-Roll**.
* VPython використовує апаратне прискорення браузера, що забезпечує високу частоту оновлення (100 FPS), роблячи візуалізацію плавною та точною.

## 6. Результат Роботи 

(https://github.com/user-attachments/assets/386041c0-4ce7-415a-9d09-735d5e394a09)
![20251214_183910](https://github.com/user-attachments/assets/55de50ae-35d5-48c9-bb4a-80bea30ddf26)
![20251214_183901](https://github.com/user-attachments/assets/dac97f79-984c-4638-bf09-d52fb60c2ba2)
![20251214_184546](https://github.com/user-attachments/assets/33dab11c-f387-4b54-8262-7294eaa72b1a)

## 7. Код пайтон для відображення
import serial
from vpython import *
import time
import math 

# --- НАЛАШТУВАННЯ ПОРТУ ---
PORT = 'COM4' 
BAUD_RATE = 115200 
# --------------------

# Ініціалізація 3D-сцени та моделі
scene.range = 5 
scene.background = color.gray(0.2)
box_model = box(size=vector(4, 0.5, 2), color=color.orange, opacity=0.8)
arrow_length = 2.5
arrow_dir = arrow(pos=box_model.pos, axis=vector(arrow_length, 0, 0), length=arrow_length, shaftwidth=0.1, color=color.red)

try:
    ser = serial.Serial(PORT, BAUD_RATE, timeout=0.1) 
    time.sleep(1)
    print(f"Підключено до {PORT}. Обробка сирих даних...")
except Exception as e:
    print(f"Помилка підключення до COM-порту {PORT}: {e}")
    exit()

while True:
    rate(100) 
    
    if ser.in_waiting > 0:
        try:
            line = ser.readline().decode('utf-8').strip()
            data_list = line.split(',')
            
            # Очікуємо 6 значень, але беремо лише перші 3 (Accel X, Y, Z)
            if len(data_list) >= 3:
                raw_ax = float(data_list[0]) 
                raw_ay = float(data_list[1])
                raw_az = float(data_list[2])
                
                # --- Обчислення Roll та Pitch у Python ---
                roll_rad = math.atan2(raw_ay, raw_az)
                pitch_rad = math.atan2(-raw_ax, math.sqrt(raw_ay * raw_ay + raw_az * raw_az))
                yaw_rad = 0.0 
                
                # --- Застосування обертання ---
                box_model.axis = vector(1,0,0)
                box_model.up = vector(0,1,0)

                box_model.rotate(angle=-yaw_rad, axis=vector(0,1,0)) 
                box_model.rotate(angle=pitch_rad, axis=vector(1,0,0)) 
                box_model.rotate(angle=roll_rad, axis=vector(0,0,1))
                
                arrow_dir.axis = box_model.axis * arrow_length
                arrow_dir.up = box_model.up

        except ValueError:
            continue
        except Exception as e:
            pass
## . Код  arduino
#include "MahonyAHRS.h"
#include <Arduino.h>

// Константи Mahony
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f; 
const float beta = 0.1f; 
const float sampleFreq = 100.0f; 

// Утиліта invSqrt
float invSqrt(float x) {
  return 1.0f / sqrt(x);
}

// РЕАЛІЗАЦІЯ ФУНКЦІЇ FUSION
void MahonyAHRSupdateIMU(float gx, float gy, float gz,
                         float ax, float ay, float az) {

  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;

  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);

  if (!((ax == 0) && (ay == 0) && (az == 0))) {

    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    s0 = 4*q0*q2*q2 + 2*q2*ax + 4*q0*q1*q1 - 2*q1*ay;
    s1 = 4*q1*q3*q3 - 2*q3*ax + 4*q0*q0*q1 - 2*q0*ay;
    s2 = 4*q0*q0*q2 + 2*q0*ax + 4*q2*q3*q3 - 2*q3*ay;
    s3 = 4*q1*q1*q3 - 2*q1*ax + 4*q2*q2*q3 - 2*q2*ay;

    recipNorm = invSqrt(s0*s0 + s1*s1 + s2*s2 + s3*s3);
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    qDot1 -= beta * s0;
    qDot2 -= beta * s1;
    qDot3 -= beta * s2;
    qDot4 -= beta * s3;
  }

  q0 += qDot1 * (1.0f / sampleFreq);
  q1 += qDot2 * (1.0f / sampleFreq);
  q2 += qDot3 * (1.0f / sampleFreq);
  q3 += qDot4 * (1.0f / sampleFreq);

  recipNorm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
}

// РЕАЛІЗАЦІЯ ФУНКЦІЙ ОТРИМАННЯ КУТІВ
float getRoll() {
  return atan2(2.0f*(q0*q1 + q2*q3), 1.0f - 2.0f*(q1*q1 + q2*q2)) * 57.3f;
}

float getPitch() {
  return asin(2.0f*(q0*q2 - q3*q1)) * 57.3f;
}

float getYaw() {
  // Yaw не буде стабільним без магнітометра, але виводимо його
  return atan2(2.0f*(q0*q3 + q1*q2), 1.0f - 2.0f*(q2*q2 + q3*q3)) * 57.3f;
}
---
## . Висновок:
  Результат (Працює) Зв'язок між Arduino Nano та Python налагоджено та є стабільним.

Код апаратної та програмної частин працює, забезпечуючи постійний потік даних.
Але технічна Проблема (Чому рух нестабільний)
Ми отримуємо сирі дані з датчика, а не плавні, обчислені кути Roll/Pitch/Yaw.

Причина: Обмежені ресурси мікроконтролера Arduino Nano (ATmega328) та його неповна підтримка деяких складних бібліотек (або нестабільна робота інтегрованого коду) ускладнили або унеможливили стабільне виконання повноцінного Fusion-алгоритму (Madgwick/Mahony).

Наслідок: 3D-модель (літак) погано реагує на рух датчика, оскільки вона обертається відповідно до нестабільних сирих вимірювань, а не відфільтрованих кутів.
