#Librerias generales
from ctypes import *
import sys
from time import sleep, clock
from math import atan, atan2, sqrt, pi
from msvcrt import getch, kbhit

#Librerias de Phidgets
from Phidgets.Phidget import Phidget
from Phidgets.PhidgetException import PhidgetErrorCodes, PhidgetException
from Phidgets.Devices.Spatial import Spatial, SpatialEventData, TimeSpan
from Phidgets.Events.Events import SpatialDataEventArgs, AttachEventArgs, DetachEventArgs, ErrorEventArgs, CurrentChangeEventArgs, PositionChangeEventArgs, VelocityChangeEventArgs
from Phidgets.Devices.AdvancedServo import AdvancedServo
from Phidgets.Devices.Servo import ServoTypes
from Phidgets.Phidget import PhidgetLogLevel

#Crear un objeto AdvancedServo
try:
    advancedServo = AdvancedServo()
except RuntimeError as e:
    print("Runtime Exception: %s" % e.details)
    print("Exiting....")
    exit(1)

#Crear un objeto Spatial
try:
    spatial = Spatial()
    timeSpan = TimeSpan(0,0)
except RuntimeError as e:
    print("Runtime Exception: %s" % e.details)
    print("Exiting....")
    exit(1)

#Valores de los mototres
currentList = [0,0,0,0,0,0,0,0]
velocityList = [0,0,0,0,0,0,0,0]

# Define una funcion para manejar errores y asi usarla en los "try...except"
def LocalErrorCatcher(event):
    print("Phidget Exception: " + str(e.code) + " - " + str(e.details) )

#Funciones de visualizacion de informacion
def DisplayDeviceInfo2():
    print("|------------|----------------------------------|--------------|------------|")
    print("|-Conectado -|-              Tipo              -|- N. de Serie-|-  Version -|")
    print("|------------|----------------------------------|--------------|------------|")
    print("|- %8s -|- %30s -|- %10d -|- %8d -|" % (spatial.isAttached(), spatial.getDeviceName(), spatial.getSerialNum(), spatial.getDeviceVersion()))
    print("|------------|----------------------------------|--------------|------------|")
    print("N. de Acelerometros: %i" % (spatial.getAccelerationAxisCount()))
    print("N. de Giroscopos: %i" % (spatial.getGyroAxisCount()))
    print("N. de Brujulas: %i" % (spatial.getCompassAxisCount()))

def DisplayDeviceInfo1():
    print("|------------|----------------------------------|--------------|------------|")
    print("|- Conectado-|-              Tipo              -|-N. de Serie -|-  Version -|")
    print("|------------|----------------------------------|--------------|------------|")
    print("|- %8s -|- %30s -|- %10d -|- %8d -|" % (advancedServo.isAttached(), advancedServo.getDeviceName(), advancedServo.getSerialNum(), advancedServo.getDeviceVersion()))
    print("|------------|----------------------------------|--------------|------------|")
    print("Numero de motores: %i" % (advancedServo.getMotorCount()))

#Funciones de manejo de eventos
def Attached(e):
    attached = e.device
    print("Servo %i Conectado" % (attached.getSerialNum()))

def Detached(e):
    detached = e.device
    print("Servo %i Descnectado!" % (detached.getSerialNum()))

def Error(e):
    try:
        source = e.device
        print("Phidget Error %i: %s" % (source.getSerialNum(), e.eCode, e.description))
    except PhidgetException as e:
        print("Phidget Excepcion %i: %s" % (e.code, e.details))

def CurrentChanged(e):
    global currentList
    currentList[e.index] = e.current

def PositionChanged(e):
    source = e.device
    print("AdvancedServo %i: Motor %i Posicion: %f - Velocidad: %f - Consumo: %f" % (source.getSerialNum(), e.index, e.position, velocityList[e.index], currentList[e.index]))
    if advancedServo.getStopped(e.index) == True:
        print("Motor %i Parado" % (e.index))

def VelocityChanged(e):
    global velocityList
    velocityList[e.index] = e.velocity

def SpatialAttached(e):
    attached = e.device
    print("Spatial %i Conectado" % (attached.getSerialNum()))

def SpatialDetached(e):
    detached = e.device
    print("Spatial %i Desconectado" % (detached.getSerialNum()))

def SpatialError(e):
    try:
        source = e.device
        print("Spatial %i: Phidget Error %i: %s" % (source.getSerialNum(), e.eCode, e.description))
    except PhidgetException as e:
        print("Phidget Excepcion %i: %s" % (e.code, e.details))

def SpatialData(e):
    source = e.device
    print("Spatial %i: Cantidad de datos %i" % (source.getSerialNum(), len(e.spatialData)))
    for index, spatialData in enumerate(e.spatialData):
        print("=== Conjunto de datos: %i ===" % (index))
        if len(spatialData.Acceleration) > 0:
            print("Acceleracion> x: %6f  y: %6f  z: %6f" % (spatialData.Acceleration[0], spatialData.Acceleration[1], spatialData.Acceleration[2]))
        if len(spatialData.AngularRate) > 0:
            print("Velocidad Angular> x: %6f  y: %6f  z: %6f" % (spatialData.AngularRate[0], spatialData.AngularRate[1], spatialData.AngularRate[2]))
        if len(spatialData.MagneticField) > 0:
            print("Campo Magnetico> x: %6f  y: %6f  z: %6f" % (spatialData.MagneticField[0], spatialData.MagneticField[1], spatialData.MagneticField[2]))
        print("Intervalo de tiempo> Segundos Transcurridos: %i  Microsegundos desde el ultimo paquete: %i" % (spatialData.Timestamp.seconds, spatialData.Timestamp.microSeconds))
    
    print("------------------------------------------")


#Empieza el programa 

#Primero iniciamos los manejadores de eventos del AdvancedServo
try:
    advancedServo.setOnAttachHandler(Attached)
    advancedServo.setOnDetachHandler(Detached)
    advancedServo.setOnErrorhandler(Error)
    advancedServo.setOnCurrentChangeHandler(CurrentChanged)
    advancedServo.setOnPositionChangeHandler(PositionChanged)
    advancedServo.setOnVelocityChangeHandler(VelocityChanged)
except PhidgetException as e:
    print("Phidget Excepcion %i: %s" % (e.code, e.details))
    print("Saliendo....")
    exit(1)

print("Abriendo Objeto AdvancedServo....")

try:
    advancedServo.openPhidget()
except PhidgetException as e:
    print("Phidget Excepcion %i: %s" % (e.code, e.details))
    print("Saliendo....")
    exit(1)

print("Esperando la conexion....")

try:
    advancedServo.waitForAttach(10000)
except PhidgetException as e:
    print("Phidget Excepcion %i: %s" % (e.code, e.details))
    try:
        advancedServo.closePhidget()
    except PhidgetException as e:
        print("Phidget Excepcion %i: %s" % (e.code, e.details))
        print("Saliendo....")
        exit(1)
    print("Saliendo....")
    exit(1)
else:
    DisplayDeviceInfo1()

#Iniciamos los manejadores de eventos del Spatial
try:
    #logging example, uncomment to generate a log file
    #spatial.enableLogging(PhidgetLogLevel.PHIDGET_LOG_VERBOSE, "phidgetlog.log")

    spatial.setOnAttachHandler(SpatialAttached)
    spatial.setOnDetachHandler(SpatialDetached)
    spatial.setOnErrorhandler(SpatialError)
except PhidgetException as e:
    print("Phidget Excepcion %i: %s" % (e.code, e.details))
    print("Saliendo....")
    exit(1)

print("Abriendo Objeto Spatial....")

try:
    spatial.openPhidget()
except PhidgetException as e:
    print("Phidget Excepcion %i: %s" % (e.code, e.details))
    print("Saliendo....")
    exit(1)

print("Esperando la conexion....")

try:
    spatial.waitForAttach(10000)
except PhidgetException as e:
    print("Phidget Excepcion %i: %s" % (e.code, e.details))
    try:
        spatial.closePhidget()
    except PhidgetException as e:
        print("Phidget Excepcion %i: %s" % (e.code, e.details))
        print("Saliendo....")
        exit(1)
    print("Saliendo....")
    exit(1)
else:
    spatial.setDataRate(4*125)
    DisplayDeviceInfo2()


try:
    print("Ajustando el tipo de motor para un HITEC_HS322HD")
    advancedServo.setServoType(0, ServoTypes.PHIDGET_SERVO_HITEC_HS322HD)
    advancedServo.setServoType(1, ServoTypes.PHIDGET_SERVO_HITEC_HS322HD)
    print("Estado Speed Ramping del servo 0: %s" % advancedServo.getSpeedRampingOn(0))
    print("Estado Speed Ramping del servo 1: %s" % advancedServo.getSpeedRampingOn(1))
    print("Estado Servo 0 Detenido: %s" % advancedServo.getStopped(0))
    print("Estado Servo 1 Detenido: %s" % advancedServo.getStopped(1))
    print("Estado Servo 0 Engranado: %s" % advancedServo.getEngaged(0))
    print("Estado Servo 1 Engranado: %s" % advancedServo.getEngaged(1))
    
    print("Trabajando con motores 0 y 1...")
    
    print("Aceleracion ajustada a un cuarto de la maxima: %f" % (advancedServo.getAccelerationMax(0)/4))
    advancedServo.setAcceleration(0, (advancedServo.getAccelerationMax(0)/4))
    advancedServo.setAcceleration(1, (advancedServo.getAccelerationMax(1)/4))
    
    sleep(0.5)
    
    print("Velocidad ajustada a un cuarto de la maxima: %f" % (advancedServo.getVelocityMax(0)/4))
    advancedServo.setVelocityLimit(0, (advancedServo.getVelocityMax(0)/4))
    advancedServo.setVelocityLimit(1, (advancedServo.getVelocityMax(1)/4))
    sleep(0.5)
    
    print("Motores a posicion neutral (90 grados)...")
    advancedServo.setPosition(0, 90.00)
    advancedServo.setPosition(1, 90.00)
    advancedServo.setEngaged(0, True)
    advancedServo.setEngaged(1, True)
    sleep(0.5)
    
    print("Estado Servo 0 Engranado: %s" % advancedServo.getEngaged(0))
    print("Estado Servo 1 Engranado: %s" % advancedServo.getEngaged(1))
    
    print ("Ajuste posicion minima a 30 grados")
    advancedServo.setPositionMin(0, 30.00)
    advancedServo.setPositionMin(1, 30.00)
    sleep(0.5)
    
    print ("Ajuste posicion maxima a 150 grados")
    advancedServo.setPositionMax(0, 150.00)
    advancedServo.setPositionMax(1, 150.00)
    sleep(0.5)
    
    print ("Entrando en Modo Automatico")
    
    sleep (0.5)
    
    #ajustamos los parametros del Filtro Complementario
    K = 0.9
    K1 = 1 - K
    #Ajustamos los parámetros del PID
    KP = 0.1
    KD = 0.1
    KI = 0.0001
    
    #Declaramos varaibles que vamos a utilizar luego
    Ix = 0 
    Iy = 0
    UltimoX = 0
    UltimoX0 = 0
    UltimoY = 0
    UltimoY0 = 0
    i = 1
    IntervaloTiempo = 0
    
    print ('Todo listo.')
    print ('Pulsa Enter para empezar')
    print ('Pulsa Espacio para salir del bucle una vez dentro')
    chr = sys.stdin.read(1)
    

    #Comenzamos bucle de autonivelamiento 
    
    while not kbhit() or getch() != " ":
    
            
        #Medimos los valores actuales de los acelerometros en g=9.8 m/s**2
        accX = spatial.getAcceleration(0)
        accY = spatial.getAcceleration(1)
        accZ = spatial.getAcceleration(2)
        
        #Calculamos el giro (radianes) que nos dan los acelerometros
        rotacionX = atan2(accY, accZ)                       #Roll
        rotacionY = atan(-accX / sqrt(accY**2 + accZ**2))   #Pitch
        
        rotacionXGrados = rotacionX * 180 / pi
        rotacionYGrados = rotacionY * 180 / pi
        
        #medimos los valores actuales de los giroscopos (en grados/s) 
        gyroX = spatial.getAngularRate(0)
        gyroY = spatial.getAngularRate(1)
        
        #Leemos el reloj del procesador para calcular más tarde el intervalo de muestreo
        IntervaloTiempo0 = clock()
        
        #(Gyro * intervalo muestreo (dt)) nos da el valor del giro medido por el giroscopo en grados
        gyroXDelta = gyroX * IntervaloTiempo
        gyroYDelta = gyroY * IntervaloTiempo 
        
        #Pasamos los datos de los acelerometros y el giroscopio por un Filtro Complementario
        UltimoX = K * (UltimoX + gyroXDelta) + K1 * rotacionXGrados
        UltimoY = K * (UltimoY + gyroYDelta) + K1 * rotacionYGrados
        
        #Utilizamos el angulo calculado para el control PID
        Ix = Ix+UltimoX*KI
        PIDx = KP * UltimoX + KD * (UltimoX - UltimoX0)+ Ix
        Iy = Iy+UltimoY*KI
        PIDy = KP * UltimoY + KD * (UltimoY - UltimoY0) + Iy
        
        #Movemos los motores la cantidad que nos indica el PID si este no vale 0.0
        if PIDy != 0.0:
            try:
                advancedServo.setPosition(1, advancedServo.getPosition(1) + PIDy)
            except:
                print ('Has llegado a la posicion limite en el eje 1: %.2f grados') % advancedServo.getPosition(1)
                print
                advancedServo.setPosition(1, advancedServo.getPosition(1))
            if PIDx != 0.0:
                try:
                    advancedServo.setPosition(0, advancedServo.getPosition(0) + PIDx)
                except:
                    print ('Has llegado a la posicion limite en el eje 0: %.2f grados') % advancedServo.getPosition(0)
                    print
                    advancedServo.setPosition(0, advancedServo.getPosition(0))
            else:
                advancedServo.setPosition(0, advancedServo.getPosition(0))
        else:
            advancedServo.setPosition(1, advancedServo.getPosition(1))
    
        print float(rotacionXGrados), 'RotationX (grados)'
        print float(rotacionYGrados), 'RotationY (grados)'
        
        print float(UltimoX), 'lastX (grados): ' 
        print float(UltimoX0), 'lastX0 (grados): '
        print float(UltimoY), 'lastY (grados): ' 
        print float(UltimoX0), 'lastY0 (grados): ' 
        
        print float(gyroX), ' gyroX (grados/s)'
        print float(gyroY), ' gyroY (grados/s)'
        
        print float(gyroXDelta), 'gyroXDelta (grados)'
        print float(gyroYDelta), 'gyroYDelta (grados) '
        
        print float(IntervaloTiempo), 'IntervaloTiempo en s'
        print float(IntervaloTiempo0), 'IntervaloTiempo0 en s'
        
        print float(PIDx) , 'PIDx'
        print float(PIDy) , 'PIDy'
        
        print
        
        UltimoX0 = UltimoX
        UltimoY0 = UltimoY
        IntervaloTiempo = clock() - IntervaloTiempo0



    print("Desengranando los motores..." )
    advancedServo.setEngaged(0, False)
    advancedServo.setEngaged(1, False)
    
    print("Estado conexion Servo 0: %s" % advancedServo.getEngaged(0))
    print("Estado conexion Servo 1: %s" % advancedServo.getEngaged(1))
    
except PhidgetException as e:
    print("Phidget Excepcion %i: %s" % (e.code, e.details))
    try:
        spatial.closePhidget()
        advancedServo.setEngaged(0, False)
        advancedServo.setEngaged(1, False)
        advancedServo.closePhidget()
    except PhidgetException as e:
        print("Phidget Excepcion %i: %s" % (e.code, e.details))
        print("Saliendo....")
        exit(1)
    print("Saliendo....")
    exit(1)

print("Pulse Enter para salir del programa....")

chr = sys.stdin.read(1)

print("Cerrando Objeto Spatial...")

try:
    spatial.closePhidget()
except PhidgetException as e:
    print("Phidget Excepcion %i: %s" % (e.code, e.details))
    print("Saliendo....")
    exit(1)

print("Cerrando Objeto AdvancedServo...")

try:
    advancedServo.closePhidget()
except PhidgetException as e:
    print("Phidget Excepcion %i: %s" % (e.code, e.details))
    print("Saliendo....")
    exit(1)

print("Listo.")
exit(0)