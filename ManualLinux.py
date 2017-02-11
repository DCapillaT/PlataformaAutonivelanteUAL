#Librerias generales
from ctypes import *
import sys
import termios
import tty
from time import sleep

#Librerias de Phidgets
from Phidgets.Phidget import Phidget
from Phidgets.PhidgetException import PhidgetErrorCodes, PhidgetException
from Phidgets.Events.Events import *
from Phidgets.Phidget import PhidgetLogLevel
from Phidgets.Devices.AdvancedServo import AdvancedServo
from Phidgets.Devices.Servo import ServoTypes

#Crear un objeto AdvancedServo
try:
    advancedServo = AdvancedServo()
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

#Funcion de visualizacion de informacion
def DisplayDeviceInfo1():
    print("|------------|----------------------------------|--------------|------------|")
    print("|-Conectado -|-              Tipo              -|- N. Serie   -|-  Version -|")
    print("|------------|----------------------------------|--------------|------------|")
    print("|- %8s -|- %30s -|- %10d -|- %8d -|" % (advancedServo.isAttached(), advancedServo.getDeviceName(), advancedServo.getSerialNum(), advancedServo.getDeviceVersion()))
    print("|------------|----------------------------------|--------------|------------|")
    print("Number of motors: %i" % (advancedServo.getMotorCount()))

#Funciones de manejo de eventos
def Attached(e):
    attached = e.device
    print("Servo %i Conectado!" % (attached.getSerialNum()))

def Detached(e):
    detached = e.device
    print("Servo %i Desconectado!" % (detached.getSerialNum()))

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

#Modulos para mover motor 
    
def mover_0_pos():
    try:
        advancedServo.setPosition(0, advancedServo.getPosition(0) + avance_manual)
    except:
        print ('Has llegado a la posicion max en este eje: %.2f grados') % advancedServo.getPositionMax(0)
        print
        advancedServo.setPosition(0, advancedServo.getPositionMax(0))
    
def mover_0_neg():
    try:
        advancedServo.setPosition(0, advancedServo.getPosition(0) - avance_manual)
    except:
        print ('Has llegado a la posicion min en este eje: %.2f grados') % advancedServo.getPositionMin(0)
        print
        advancedServo.setPosition(0, advancedServo.getPositionMin(0))
def mover_1_pos():
    try:
        advancedServo.setPosition(1, advancedServo.getPosition(1) + avance_manual)
    except:
        print ('Has llegado a la posicion Max en este eje: %.2f grados') % advancedServo.getPositionMax(1)
        print
        advancedServo.setPosition(1, advancedServo.getPositionMax(1))
def mover_1_neg():
    try:
        advancedServo.setPosition(1, advancedServo.getPosition(1) - avance_manual)
    except:
        print ('Has llegado a la posicion min en este eje: %.2f grados') % advancedServo.getPositionMin(1)
        print
        advancedServo.setPosition(1, advancedServo.getPositionMin(1))

#Modulo getch() para Linux

def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

#Empieza el programa 

#Primero iniciamos los manejadores de eventos
try:
    advancedServo.setOnAttachHandler(Attached)
    advancedServo.setOnDetachHandler(Detached)
    advancedServo.setOnErrorhandler(Error)
    advancedServo.setOnCurrentChangeHandler(CurrentChanged)
    advancedServo.setOnPositionChangeHandler(PositionChanged)
    advancedServo.setOnVelocityChangeHandler(VelocityChanged)
except PhidgetException as e:
    print("Phidget Exception %i: %s" % (e.code, e.details))
    print("Exiting....")
    exit(1)

print("Abriendo Objeto AdvancedServo....")

try:
    advancedServo.openPhidget()
except PhidgetException as e:
    print("Phidget Exception %i: %s" % (e.code, e.details))
    print("Exiting....")
    exit(1)

print("Esperando la conexion....")

try:
    advancedServo.waitForAttach(10000)
except PhidgetException as e:
    print("Phidget Exception %i: %s" % (e.code, e.details))
    try:
        advancedServo.closePhidget()
    except PhidgetException as e:
        print("Phidget Exception %i: %s" % (e.code, e.details))
        print("Exiting....")
        exit(1)
    print("Exiting....")
    exit(1)
else:
    DisplayDeviceInfo1()


try:
    print("Setting the servo type for motor 0 to HITEC_HS322HD")
    advancedServo.setServoType(0, ServoTypes.PHIDGET_SERVO_HITEC_HS322HD)
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
    
    print ('Entrando en Modo Manual......')
    print ('Pulsa las wasd o las flechas del teclado para controlar la plataforma')
    print ('(pulsa x para salir)')
    
    sleep(1)
    
    avance_manual = int(raw_input('Introduce cuantos grados quieres avanzar por pulsacion(un numero entre 1 y 5, ambos inclusive, por favor): '))

    
    while 1 <= avance_manual <= 5:
        # Guardamos la entrada del teclado con getch en la variable char
        char = getch()

        # La plataforma movera el motor 0 hacia adelante
        if(char == "w"):
            mover_0_pos()

        # La plataforma movera el motor 0 hacia atras
        if(char == "s"):
            mover_0_neg()

        # El motor 1 se movera hacia adelante
        if(char == "a"):
            mover_1_pos()

        # el motor 1 se movera hacia atras
        if(char == "d"):
            mover_1_neg()

        # introducir la x nos sacara del bucle
        if(char == "x"):
            print("Saliendo del Modo Manual....")
            advancedServo.setEngaged(0, False)
            advancedServo.setEngaged(1, False)
            break
            
        # Limpiamos la variable char para guardar la siguiente tecla pulsada
        char = ""
    else:
        print("Desengranando los motores..." )
        advancedServo.setEngaged(0, False)
        advancedServo.setEngaged(1, False)
        
        print("Estado Servo 0 Desengranado: %s" % advancedServo.getEngaged(0))
        print("Estado Servo 1 Desengranado: %s" % advancedServo.getEngaged(1))
    
except PhidgetException as e:
    print("Excepcion de Phidget %i: %s" % (e.code, e.details))
    try:
        advancedServo.setEngaged(0, False)
        advancedServo.setEngaged(1, False)

        advancedServo.closePhidget()
    except PhidgetException as e:
        print("Excepcion de Phidget %i: %s" % (e.code, e.details))
        print("Saliendo del programa....")
        exit(1)
    print("Saliendo del programa....")
    exit(1)

print("Presione Enter para salir del programa....")

chr = sys.stdin.read(1)

print("Cerrando...")

try:
    advancedServo.closePhidget()
except PhidgetException as e:
    print("Excepcion Phidget %i: %s" % (e.code, e.details))
    print("Saliendo....")
    exit(1)

print("Listo.")
exit(0)





