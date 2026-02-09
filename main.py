from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor
from pybricks.parameters import Port, Direction
from pybricks.tools import wait

# ===============================
# CONFIGURAÇÃO DE HARDWARE
# ===============================

hub = PrimeHub()

motor_left = Motor(Port.A, Direction.COUNTERCLOCKWISE)
motor_right = Motor(Port.B)

K_THETA = 0 
K_OMEGA = 0
DT_MS = 10
BASE_POWER = 100
MAX_POWER = 100



def straight(target_rotation=None, tolerance=2):
    # ===============================
    # PARÂMETROS DO LQR
    # ===============================

    # Ganhos LQR (ajustáveis)
    K_THETA = 1      # ganho do erro angular
    K_OMEGA = 0.1    # ganho da velocidade angular

    # Feedforward (potência base)
    BASE_POWER = 100   # ajuste conforme o robô

    # Tempo de amostragem (ms)
    DT_MS = 10

    # ===============================
    # INICIALIZAÇÃO
    # ===============================

    hub.imu.reset_heading(0)
    # Zera contadores de ângulo dos motores para referência
    try:
        motor_left.reset_angle(0)
        motor_right.reset_angle(0)
    except Exception:
        pass

    wait(10)

    # ===============================
    # LOOP DE CONTROLE LQR
    # Parada opcional por rotação média dos dois motores
    # ===============================

    while True:
        # Estados do sistema
        theta = hub.imu.heading()                 # erro angular (graus)
        omega = hub.imu.angular_velocity()[2]     # velocidade angular Z (°/s)

        # Controle LQR: u = -Kx
        u = -(K_THETA * theta + K_OMEGA * omega)

        # Saturação de segurança
        if u > 100:
            u = 100
        elif u < -100:
            u = -100

        # Aplicação do controle (feedforward + feedback)
        motor_left.dc(BASE_POWER + u)
        motor_right.dc(BASE_POWER - u)

        # Condição de parada por rotação média dos dois motores
        if target_rotation is not None:
            avg_angle = (motor_left.angle() + motor_right.angle()) / 2
            if abs(avg_angle) >= abs(target_rotation) - tolerance:
                break

        wait(DT_MS)

    motor_left.hold()
    motor_right.hold()

def gyro(target_angle, tolerance=2, mode="pivot"):
    """
    Gira o robô até o ângulo desejado usando LQR.

    target_angle: ângulo alvo em graus
    tolerance: erro aceitável
    mode:
        "pivot"      -> gira no próprio eixo
        "one_left"   -> motor esquerdo parado
        "one_right"  -> motor direito parado
    """

    # Ganhos LQR (POSITIVOS!)
    K_THETA = -1.25
    K_OMEGA = 0.135

    hub.imu.reset_heading(0)
    wait(200)

    while True:
        heading = hub.imu.heading()
        theta_error = target_angle - heading
        omega = hub.imu.angular_velocity()[2]

        # Condição de parada
        if abs(theta_error) < tolerance and abs(omega) < 1.0:
            break

        # Controle LQR
        u = -(K_THETA * theta_error + K_OMEGA * omega)

        # Saturação
        if u > MAX_POWER:
            u = MAX_POWER
        elif u < -MAX_POWER:
            u = -MAX_POWER

        # Aplicação conforme modo escolhido
        if mode == "pivot":
            motor_left.dc(u)
            motor_right.dc(-u)

        elif mode == "one_left":
            motor_left.dc(0)
            motor_right.dc(-u)

        elif mode == "one_right":
            motor_left.dc(u)
            motor_right.dc(0)

        else:
            raise ValueError("Modo inválido")

        wait(DT_MS)
        print(f"Heading: {heading:.2f}°, Error: {theta_error:.2f}°, Omega: {omega:.2f}°/s, Control: {u:.2f}%")

    motor_left.hold()
    motor_right.hold()



def main():
    # Exemplo de uso: girar 90 graus
    straight(360)
    gyro(90)
    gyro(-270)
    straight(360)
    for _ in range(5):
        gyro(90)
        gyro(-90)
    for _ in range(5):
        gyro(180)
        gyro(-180)
main()



