#!/usr/bin/env python3

import rospy
import signal
import time
from std_msgs.msg import Float32, UInt16
from geometry_msgs.msg import Twist

posicao_atual_servo = 90
limiar_obstaculo = 50
cmd_vel_pub = rospy.Publisher("/bob/cmd_vel", Twist, queue_size=10)
cmd_vel = Twist()

obstaculo_detectado = False

# Função de retorno de chamada para a posição do servo
def callback_servo(msg):
    global posicao_atual_servo 
    posicao_atual_servo = msg.data

# Função de retorno de chamada para a leitura do sensor Sharp
def callback_sharp(msg):
    distancia = msg.data
    global obstaculo_detectado

    if distancia < limiar_obstaculo:
        # Obstáculo detectado, realiza o desvio
        
        if not obstaculo_detectado:
            # Inicia o desvio somente se não estiver em desvio atualmente
            obstaculo_detectado = True

            if posicao_atual_servo < 90:
                # Girar para a esquerda
                print('Girar para a esquerda')
                cmd_vel.linear.x = -0.1
                cmd_vel.angular.z = 1.0
                
            else:
                print('Girar para a direita')
                cmd_vel.linear.x = -0.1
                cmd_vel.angular.z = -1.0

    else:
        # Seguir em frente
        obstaculo_detectado = False
        print( 'Seguir em frente')
        cmd_vel.linear.x = 0.5
        cmd_vel.angular.z = 0.0

    cmd_vel_pub.publish(cmd_vel)

def cleanup(signal, frame):
    # Executar ações de limpeza aqui
    # Zerar as variáveis globais
    global cmd_vel
    cmd_vel.angular.z = 0.0
    cmd_vel.linear.x = 0.0
    cmd_vel_pub.publish(cmd_vel)
    rospy.loginfo("Programa encerrado. Variáveis zeradas.")
    rospy.signal_shutdown("Programa encerrado.")

def main():
    rospy.init_node('desvio_obstaculo')
 

    # Cria os publicadores e os inscritores
    rospy.Subscriber("/bob/ir_motor", UInt16, callback_servo)
    rospy.Subscriber("/bob/ir_sensor", Float32, callback_sharp)

    # Registrar o manipulador de sinal para capturar o sinal de encerramento
    signal.signal(signal.SIGINT, cleanup)
    signal.signal(signal.SIGTERM, cleanup)

    # Inicia o loop principal do ROS
    rospy.spin()

if __name__ == '__main__':
    main()
