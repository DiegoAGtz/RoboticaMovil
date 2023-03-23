# Aquí hay un ejemplo de algoritmo en Python para esquivar obstáculos con sensores ultrasónicos en CoppeliaSim. Este ejemplo utiliza el robot Pioneer P3DX y sensores ultrasónicos para detectar obstáculos. El algoritmo utiliza la información de los sensores para calcular la distancia y la dirección del obstáculo y luego ajusta la velocidad y la dirección del robot para evitar el obstáculo⁶.

#```python
import sim
import math

def getSensorData(sensorHandle):
    # Get the distance to the detected object
    res, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = sim.simxReadProximitySensor(clientID, sensorHandle, sim.simx_opmode_blocking)
    if res == sim.simx_return_ok:
        if detectionState:
            distance = math.sqrt(detectedPoint[0]**2 + detectedPoint[1]**2 + detectedPoint[2]**2)
            return distance
    return None

def avoidObstacle():
    # Get the sensor handles
    sensorHandles = []
    for i in range(1, 17):
        res, sensorHandle = sim.simxGetObjectHandle(clientID, f'Pioneer_p3dx_ultrasonicSensor{i}', sim.simx_opmode_blocking)
        sensorHandles.append(sensorHandle)

    # Get the robot handle
    res, robotHandle = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx', sim.simx_opmode_blocking)

    # Set the robot's velocity
    sim.simxSetJointTargetVelocity(clientID, sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor', sim.simx_opmode_blocking)[1], 1, sim.simx_opmode_oneshot)
    sim.simxSetJointTargetVelocity(clientID, sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor', sim.simx_opmode_blocking)[1], 1, sim.simx_opmode_oneshot)

    # Avoid obstacles
    while True:
        # Get the sensor data
        sensorData = []
        for sensorHandle in sensorHandles:
            sensorData.append(getSensorData(sensorHandle))

        # Calculate the distance and direction of the obstacle
        minDistance = min(sensorData)
        minIndex = sensorData.index(minDistance)
        direction = (minIndex - 8) * math.pi / 8

        # Adjust the robot's velocity and direction
        if minDistance < 0.5:
            sim.simxSetJointTargetVelocity(clientID, sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor', sim.simx_opmode_blocking)[1], -1, sim.simx_opmode_oneshot)
            sim.simxSetJointTargetVelocity(clientID, sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor', sim.simx_opmode_blocking)[1], 1, sim.simx_opmode_oneshot)
        else:
            sim.simxSetJointTargetVelocity(clientID, sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor', sim.simx_opmode_blocking)[1], 1, sim.simx_opmode_oneshot)
            sim.simxSetJointTargetVelocity(clientID, sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor', sim.simx_opmode_blocking)[1], 1, sim.simx_opmode_oneshot)

        # Stop the robot if it gets stuck
        if minDistance is None:
            sim.simxSetJointTargetVelocity(clientID, sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor', sim.simx_opmode_blocking)[1], 0, sim.simx_opmode_oneshot)
            sim.simxSetJointTargetVelocity(clientID, sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor', sim.simx_opmode_blocking)[1], 0, sim.simx_opmode_oneshot)
            break

        # Rotate the robot to avoid the obstacle
        sim.simxSetJointTargetVelocity(clientID, sim.simxGetObjectHandle(client

# Origen: Conversación con Bing, 21/3/2023(1) Un robot que detecta y evita obstáculos. • AranaCorp. https://www.aranacorp.com/es/un-robot-que-detecta-y-evita-obstaculos/ Con acceso 21/3/2023.
# (2) Tutorial CoppeliaSim 2022 ( Simulación Robot Evita ... - YouTube. https://www.youtube.com/watch?v=_yPqYaBEpRw Con acceso 21/3/2023.
# (3) GitHub - MurilloJam/IA_aplicada_a_la_solucion_de_laberintos: Algoritmos .... https://github.com/MurilloJam/IA_aplicada_a_la_solucion_de_laberintos Con acceso 21/3/2023.
# (4) Simulación en entornos con robots manipuladores móviles. https://bing.com/search?q=algoritmo+en+python+para+esquivar+obstaculos+en+coppeliaSim Con acceso 21/3/2023.
# (5) Robot Esquiva Obstáculos : 8 Steps - Instructables. https://www.instructables.com/Robot-Esquiva-Obst%C3%A1culos/ Con acceso 21/3/2023.
# (6) Carro automatico esquiva obstaculos con sensores ultrasonicos. https://www.youtube.com/watch?v=zPwF2eGOrpc Con acceso 21/3/2023.
