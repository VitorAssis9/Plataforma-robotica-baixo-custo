function sysCall_init() 
    sensorHandle = sim.getObjectHandle('Sensor_Infravermelho')
    sensorServo = sim.getObjectHandle('Revolute_joint_Sensor')
    Roda3 = sim.getObjectHandle('Revolute_joint_Roda3')
    Roda4 = sim.getObjectHandle('Revolute_joint_Roda4')
    Roda2 = sim.getObjectHandle('Revolute_joint_Roda2')
    Roda1 = sim.getObjectHandle('Revolute_joint_Roda1')
    RobotHandle = sim.getObjectHandle('Robot')
    if simROS then
        sim.addLog(sim.verbosity_scriptinfos,"Interface ROS Encontrada.")
        cmd_vel_sub=simROS.subscribe('/cmd_vel','geometry_msgs/Twist','subscriber_cmd_vel_callback')
        ir_sensor_sub=simROS.subscribe('/ir_sensor','std_msgs/Float32','subscriber_ir_sensor_callback')
        ir_sensor=simROS.advertise('/ir_sensor', 'std_msgs/Float32')
        servoMotor=simROS.advertise('/servo_motor', 'std_msgs/Float32')
    else
        sim.addLog(sim.verbosity_scripterrors,"Interface ROS nao Encontrada.")
    end
    servoPosition = 0
    direction = 1
end
function subscriber_cmd_vel_callback(msg)
    print(msg)
    spdLin = msg["linear"]["x"]
    spdAng = msg["angular"]["z"]
    kLin = 10.0
    kAng = -2.5
    vLeft = (kLin*spdLin)+(kAng*spdAng) 
    vRight = (kLin*spdLin)-(kAng*spdAng)  
    sim.setJointTargetVelocity(Roda3,vLeft)
    sim.setJointTargetVelocity(Roda2,vRight)
    sim.setJointTargetVelocity(Roda4,vLeft)
    sim.setJointTargetVelocity(Roda1,vRight)
end
function subscriber_ir_sensor_callback(msg)
    local result, distance = sim.readProximitySensor(sensorHandle)
    if result > 0 then
        print('Distância do objeto: ' .. distance)
        simROS.publish(ir_sensor, {data = distance})
    else
        print('Nenhum objeto detectado')
        simROS.publish(ir_sensor, {data = 0})
    end
end
function sysCall_actuation()
    if subscriber_ir_sensor_callback then
        subscriber_ir_sensor_callback(msg)
    else
        sim.addLog(sim.verbosity_scripterrors, "Erro ao chamar subscriber_ir_sensor_callback - função não encontrada")
    end
    servoPosition = servoPosition + direction
    local amplitude = 45
    if servoPosition >= amplitude or servoPosition <= -amplitude then
        direction = -direction
    end
    simROS.publish(servoMotor, {data = servoPosition})
    sim.setJointTargetPosition(sensorServo, math.rad(servoPosition))
end
function sysCall_cleanup()
    if simROS then
        simROS.shutdownSubscriber(cmd_vel_sub)
        simROS.shutdownSubscriber(ir_sensor_sub)
    end
end

