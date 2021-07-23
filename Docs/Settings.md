# Variáveis do CLI

### AirSpeed_Samples

Número de amostras para calibrar o Tubo de Pitot

| Valor Padrão | Min | Max |
| --- | --- | --- |
| 15 | 0 | 255 |

---

### Angle_Block_Arm

Se o Coseno de Z for maior que o valor definido aqui,o sistema não irá armar [Graus]

| Valor Padrão | Min | Max |
| --- | --- | --- |
| 25 | 0 | 180 |

---

### Arm_Time_Safety

Tempo seguro para armar com os sticks em posição de armamento [Segundos]

| Valor Padrão | Min | Max |
| --- | --- | --- |
| 2 | 0 | 255 |

---

### AutoDisarm_Throttle_Min

Valor maximo do Throttle tolerado para iniciar a contagem do Auto-Desarmamento [uS]

| Valor Padrão | Min | Max |
| --- | --- | --- |
| 1100 | 800 | 1500 |

---

### AutoDisarm_Time

Estouro de tempo para desarmar a controladora em nivel baixo de Throttle [Segundos]

| Valor Padrão | Min | Max |
| --- | --- | --- |
| 5 | 0 | 255 |

---

### AutoDisarm_YPR_Max

Valor maximo tolerado nos canais Yaw,Pitch e Roll para validar o Auto-Desarmamento [uS]

| Valor Padrão | Min | Max |
| --- | --- | --- |
| 1550 | 800 | 2200 |

---

### AutoDisarm_YPR_Min

Valor minimo tolerado nos canais Yaw,Pitch e Roll para validar o Auto-Desarmamento [uS]

| Valor Padrão | Min | Max |
| --- | --- | --- |
| 1450 | 800 | 1500 |

---

### AutoLaunch_AHRS_BankAngle

Ângulo no AHRS para considerar que o Auto-Launch deve iniciar [Graus]

| Valor Padrão | Min | Max |
| --- | --- | --- |
| 45 | 5 | 180 |

---

### AutoLaunch_Altitude

Cancela o Auto-Launch após atingir essa altitude (O tempo acima será ignorado) [Metros]

| Valor Padrão | Min | Max |
| --- | --- | --- |
| 0 | 0 | 255 |

---

### AutoLaunch_Elevator

Inclinação no Pitch (Elevator) ao fazer o Auto-Launch [Graus]

| Valor Padrão | Min | Max |
| --- | --- | --- |
| 18 | 0 | 100 |

---

### AutoLaunch_Exit

Cancela o Auto-Launch após o estouro desse tempo [MillisSegundos]

| Valor Padrão | Min | Max |
| --- | --- | --- |
| 5000 | 0 | 30000 |

---

### AutoLaunch_MaxThrottle

Valor do Throttle aplicado ao motor durante o Auto-Launch [uS]

| Valor Padrão | Min | Max |
| --- | --- | --- |
| 1700 | 1000 | 2000 |

---

### AutoLaunch_Motor_Delay

Tempo para subir o Throttle em rampa,assim fazendo uma partida suave,isso evita um pico de corrente no ESC [MillisSegundos]

| Valor Padrão | Min | Max |
| --- | --- | --- |
| 1500 | 0 | 10000 |

---

### AutoLaunch_Motor_SpinUp_Time

Tempo para iniciar o motor após o lançamento for detectado [MillisSegundos]

| Valor Padrão | Min | Max |
| --- | --- | --- |
| 500 | 0 | 1000 |

---

### AutoLaunch_Sticks_Exit

O piloto pode cancelar o Auto-Launch através dos Sticks do rádio somente após o estouro desse tempo [MillisSegundos]

| Valor Padrão | Min | Max |
| --- | --- | --- |
| 2000 | 0 | 15000 |

---

### AutoLaunch_Velocity_Thresh

Velocidade da IMU ou GPS para validar o Auto-Launch [Metros/Segundo]

| Valor Padrão | Min | Max |
| --- | --- | --- |
| 3 | 0 | 20 |

---

### Compass_Cal_Timer

Tempo maximo de calibração do Compass [Segundos]

| Valor Padrão | Min | Max |
| --- | --- | --- |
| 60 | 0 | 120 |

---

### Cont_Servo_Trim_Rot_Limit

Os pontos médios dos servos serão atualizados sempre que a rotação total do UAV for menor que esse limite [Graus/s]

| Valor Padrão | Min | Max |
| --- | --- | --- |
| 15 | 1 | 60 |

---

### CrashCheck_BankAngle

Valor da aceleração da IMU [Metros/Segundo^2]

| Valor Padrão | Min | Max |
| --- | --- | --- |
| 3 | 0 | 20 |

---

### CrashCheck_Timer

Estouro de tempo para validar o Crash [Segundos]

| Valor Padrão | Min | Max |
| --- | --- | --- |
| 2 | 0 | 255 |

---

### Disarm_Time_Safety

Tempo seguro para desarmar com os sticks em posição de desarmamento [Segundos]

| Valor Padrão | Min | Max |
| --- | --- | --- |
| 2 | 0 | 255 |

---

### GPS_Baud_Rate

0 - 9600KBPS / 1 - 19200KBPS / 2 - 38400KBPS / 3 - 57600KBPS / 4 - 115200KBPS

| Valor Padrão | Min | Max |
| --- | --- | --- |
| 4 | 0 | 4 |

---

### GPS_RTH_Land_Radius

Em modo RTH,inicia o Land se o UAV estiver dentro do tamanho desse raio definido aqui,caso contrario,o UAV irá subir até a altitude definido em 'RTH Altitude' nas configurações basicas,voltar ao Home-Point,e fazer o Land [Metros]

| Valor Padrão | Min | Max |
| --- | --- | --- |
| 10 | 0 | 255 |

---

### GPS_TiltCompensation

Valor para compensar o rate de navegação em modo WayPoint e RTH (Multirotores apenas)

| Valor Padrão | Min | Max |
| --- | --- | --- |
| 20 | 0 | 100 |

---

### GPS_WP_Radius

Raio do ponto para validar que o mesmo foi alcançado em modo WayPoint e RTH [Metros]

| Valor Padrão | Min | Max |
| --- | --- | --- |
| 2 | 0 | 255 |

---

### GimbalMaxValue

Valor maximo do pulso a ser aplicado no Gimbal [uS]

| Valor Padrão | Min | Max |
| --- | --- | --- |
| 2000 | 800 | 2200 |

---

### GimbalMinValue

Valor minimo do pulso a ser aplicado no Gimbal [uS]

| Valor Padrão | Min | Max |
| --- | --- | --- |
| 1000 | 800 | 2200 |

---

### Navigation_Vel

Velocidade maxima de navegação em modos de voo que utilizam o GPS [Centimetos/Segundo]

| Valor Padrão | Min | Max |
| --- | --- | --- |
| 400 | 0 | 400 |

---

### ThrottleMixGain

Valor de ganho do Throttle para o mixer do PID

| Valor Padrão | Min | Max |
| --- | --- | --- |
| 1.0 | 0 | 1 |

---

### kI_Acc_AHRS

Ganho Integral para correção da estimativa de Attitude

| Valor Padrão | Min | Max |
| --- | --- | --- |
| 50 | 0 | 255 |

---

### kI_Mag_AHRS

Ganho Integral para correção da estimativa de direção do Yaw

| Valor Padrão | Min | Max |
| --- | --- | --- |
| 0 | 0 | 255 |

---

### kP_Acc_AHRS

Ganho Proporcional para correção da estimativa de Attitude

| Valor Padrão | Min | Max |
| --- | --- | --- |
| 25 | 0 | 255 |

---

### kP_Mag_AHRS

Ganho Proporcional para correção da estimativa de direção do Yaw

| Valor Padrão | Min | Max |
| --- | --- | --- |
| 10 | 0 | 255 |

---


> Esse arquivo é gerado automaticamente,não o edite manualmente!