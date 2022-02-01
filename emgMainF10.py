# -*- coding: utf-8 -*-
"""
Created on Sat Jul  3 21:49:42 2021

@author: AlejandroDaniel
"""

from __future__ import print_function
import enum
import re
import struct
import sys
import threading
import RPi.GPIO as GPIO
import time
import serial
import csv     #ADM
import numpy as np  #ADM
import os
import tty
from serial.tools.list_ports import comports
from common import *
from select import select # Para manejo de eventos del teclado
#from pynput.keyboard import Key, Controller # Para simular presionar tecla

# Librerias para Red Neuronal
import pandas as pd
from sklearn import model_selection
from sklearn.neural_network import MLPClassifier   
from pickle import load
from sklearn.preprocessing import MinMaxScaler
from sklearn.metrics import accuracy_score
from scipy import stats
#from pynput.keyboard import Key, Controller # Para simular presionar tecla
# Cargar modelo de Red Neuronal
mlp = load(open("/home/pi/NN2/modeloK8.pkl", 'rb'))
# Cargar escalador de características
scaler = load(open("/home/pi/NN2/escaladorK8.pkl", 'rb'))


contador=0
iduser=7
idtest=0
idmove=5
startTime = 0

usercount=0;
testcount=0;
movcount=0;

sensor=0
clasificador =False #True#False
captura = False
Diferentes = True
clasif_mod2 = False

# Para comunicación con el microcontrolador
micro0 = 11
micro1 = 13
micro2 = 15
micro_flag = 19
mov_anterior = 0
primer_mov = 0

# Para pulsadores e indicadores de inicio y apagado de sistema
iniciar_sis = 0
pin_inicio = 31 # 21 
safe_shut = 35  # 23
pin_help = 33   # 29
ayuda_sis = 0
led_inicio = 37
apagar = 0

# Número de datos a muestrear
no_datos = 1000     # para entrenamiento
Ndatosclasif = 180  # para clasificación en tiempo real

# Variables para guardar datos de sendores durante adquisición de datos
media_aux = [[0],[0],[0],[0],[0],[0],[0],[0]]
emg_buffer = [[0],[0],[0],[0],[0],[0],[0],[0]]
modaclases = [0]

names = ['iav1','iav2','iav3','iav4','iav5','iav6','iav7','iav8', 
         'mav1','mav2','mav3','mav4','mav5','mav6','mav7','mav8',
         'sqrt1','sqrt2','sqrt3','sqrt4','sqrt5','sqrt6','sqrt7','sqrt8', 
         'ssi1','ssi2','ssi3','ssi4','ssi5','ssi6','ssi7','ssi8', 
         'var1','var2','var3','var4','var5','var6','var7','var8',
         'wl1','wl2','wl3','wl4','wl5','wl6','wl7','wl8', 
         'movimiento']

# Crear matrices de ceros

for y in range(49):
    modaclases.append(0)

for y in range(8):
    for x in range(no_datos-1):
        media_aux[y].append(0)
        
for y in range(8):
    for x in range(Ndatosclasif):
        emg_buffer[y].append(0)
        
############################# Función para extraer características de señales ###############################

def featExtract(data_set, num_datos):
    
    #num_datos = 60
    wfl_data= [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
############################# VALOR INTEGRADO ABSOLUTO Y VALOR MEDIO ABSOLUTO ######################################
                
    emg_rect=abs(data_set)
#     print(emg_rect)
    
    iav_data=np.sum(emg_rect, axis=0)
#     print("Valor Integrado Absoluto")
#     print(iav_data)
    
    mav_data= np.mean(emg_rect, axis=0)
#     print("Valor Medio Absoluto")
#     print(mav_data)
                
######################### Raiz Media Cuadrada e Integral Simple Cuadrada  ############################################
        
    sq_data = emg_rect**2
    ssi_data = np.sum(sq_data, axis=0)
#     print("Integral Simple Cuadrada")
#     print(ssi_data)
    
    sumsq_data= np.mean(sq_data, axis=0)
    sqrt_data=np.sqrt(sumsq_data)
#     print("Raiz Cuadrada Media")
#     print(sqrt_data)
    
############################## Varianza ################################
                
    var_data=np.var(emg_rect, axis=0, ddof=1)
#     print("Varianza")
#     print(var_data)
                
############################# Longitud de Forma de Onda ###########################
    aux_add=0
    aux_subs=0
    
    for sd in range(8):
        for nd in range(num_datos-1):
            aux_subs = abs((data_set[nd+1][sd]) - (data_set[nd][sd]))
            aux_add=aux_add+aux_subs
        wfl_data[sd]=aux_add
        aux_add=0
        aux_subs=0 
#     print("Longitud de Forma de Onda")
#     print(wfl_data)
    
###################################################################################            
    features = np.concatenate([iav_data, mav_data, sqrt_data, ssi_data, var_data, wfl_data])             
    features = np.array([features])
    
    return(features)
    
############################################################################################
    
################# Función para guardar datos en un archivo CSV #############################
def saveData(datos, nombre):
    with open(nombre, mode='w') as utm:
        utm_writer=csv.writer(utm, delimiter=',')
        utm_writer.writerows(datos)
    print("Datos guardados")
    
############################################################################################
    
######################## Código para comunicación con microcontroladores ###################
    
def enviarMov(movi):
    
    if(movi == 1):          # 000
        GPIO.output(micro0, GPIO.LOW)
        GPIO.output(micro1, GPIO.LOW)
        GPIO.output(micro2, GPIO.LOW)
        print("Descanso")
        
    elif(movi == 2):        # 001
        GPIO.output(micro0, GPIO.HIGH)
        GPIO.output(micro1, GPIO.LOW)
        GPIO.output(micro2, GPIO.LOW)
        print("Cilindro")
        
    elif(movi == 3):        # 010
        GPIO.output(micro0, GPIO.LOW)
        GPIO.output(micro1, GPIO.HIGH)
        GPIO.output(micro2, GPIO.LOW)
        print("Fist")
        
    elif(movi == 4):        # 011
        GPIO.output(micro0, GPIO.HIGH)
        GPIO.output(micro1, GPIO.HIGH)
        GPIO.output(micro2, GPIO.LOW)
        print("Pinza")
        
    elif(movi == 5):        # 100
        GPIO.output(micro0, GPIO.LOW)
        GPIO.output(micro1, GPIO.LOW)
        GPIO.output(micro2, GPIO.HIGH)
        print("Like")
    
    elif(movi == 6):        # 101
        GPIO.output(micro0, GPIO.HIGH)
        GPIO.output(micro1, GPIO.LOW)
        GPIO.output(micro2, GPIO.HIGH)
        print("Cuernos")
        
    elif(movi == 7):        # 110
        GPIO.output(micro0, GPIO.LOW)
        GPIO.output(micro1, GPIO.HIGH)
        GPIO.output(micro2, GPIO.HIGH)
        print("Esfera")
        
    elif(movi == 8):        # 111
        GPIO.output(micro0, GPIO.HIGH)
        GPIO.output(micro1, GPIO.HIGH)
        GPIO.output(micro2, GPIO.HIGH)
        print("Indice")
        
############################################################################################
        
######################## Código para leer señal de microcontroladores ######################
        
def leerMicro(micro_flag):
    global clasificador, contador, startTime, primer_mov
    if(iniciar_sis == 1):
        print("Clasificar gesto")
        clasificador = True
        contador=0
        primer_mov = 1
        startTime=time.time()
    
############################################################################################

###################### Código para suspender o reanudar clasificación ######################

def iniciarSistema(pin_inicio):
    global iniciar_sis, primer_mov
    iniciar_sis += 1
    
    print("Boton 1 presionado")
    
    if(iniciar_sis == 1):
        print("Iniciar sistema")
    elif(iniciar_sis == 2 and primer_mov > 0):
        print("Pausar sistema")
        GPIO.output(led_inicio, GPIO.HIGH)
        time.sleep(0.3)
        GPIO.output(led_inicio, GPIO.LOW)
        
    elif (iniciar_sis == 3 and primer_mov > 0):
        iniciar_sis = 1
        primer_mov = 0
        print("Reniciar sistema")

###################### Código para detener sistema y apagar Raspberry ######################

def apagarSistema(safe_shut):
    global apagar, iniciar_sis
    
    print("Boton 2 presionado")
    
    if(iniciar_sis == 0):
        apagar = 1
        print("Apagar sistema")
        GPIO.output(led_inicio, GPIO.HIGH)
        time.sleep(0.3)
        GPIO.output(led_inicio, GPIO.LOW)
        GPIO.cleanup()
        os.system("shutdown now -h")
    else:
        print("Apagado seguro de sistema")
        iniciar_sis = 2
        apagar = 1
        
############################################################################################
    
########### Código para reanudar programa si se suspende inesperadamente ###################

def ayudaSistema(pin_help):
    global ayuda_sis
    
    print("Boton 3 presionado")
    ayuda_sis = 1

######################## Código para manejo de eventos del teclado #########################

class NotTTYException(Exception): pass

class TerminalFile:
    def __init__(self,infile):
        if not infile.isatty():
            raise NotTTYException()
        self.file=infile

        #prepare for getch
        self.save_attr=tty.tcgetattr(self.file)
        newattr=self.save_attr[:]
        newattr[3] &= ~tty.ECHO & ~tty.ICANON
        tty.tcsetattr(self.file, tty.TCSANOW, newattr)

    def __del__(self):
        #restoring stdin
        import tty  #required this import here
        tty.tcsetattr(self.file, tty.TCSADRAIN, self.save_attr)

    def getch(self):
        if select([self.file],[],[],0)[0]:
            c=self.file.read(1)
        else:
            c=''
        return c

############################################################################################

######################## Código para adquisición de señales EMG ############################


def multichr(ords):
    if sys.version_info[0] >= 3:
        return bytes(ords)
    else:
        return ''.join(map(chr, ords))

def multiord(b):
    if sys.version_info[0] >= 3:
        return list(b)
    else:
        return map(ord, b)

class Arm(enum.Enum):
    UNKNOWN = 0
    RIGHT = 1
    LEFT = 2

class XDirection(enum.Enum):
    UNKNOWN = 0
    X_TOWARD_WRIST = 1
    X_TOWARD_ELBOW = 2

class Pose(enum.Enum):
    REST = 0
    FIST = 1
    WAVE_IN = 2
    WAVE_OUT = 3
    FINGERS_SPREAD = 4
    THUMB_TO_PINKY = 5
    UNKNOWN = 255    
    
class Packet(object):
    def __init__(self, ords):
        self.typ = ords[0]
        self.cls = ords[2]
        self.cmd = ords[3]
        self.payload = multichr(ords[4:])

    def __repr__(self):
        return 'Packet(%02X, %02X, %02X, [%s])' % \
            (self.typ, self.cls, self.cmd,
             ' '.join('%02X' % b for b in multiord(self.payload)))


class BT(object):
    '''Implements the non-Myo-specific details of the Bluetooth protocol.'''
    def __init__(self, tty):
        self.ser = serial.Serial(port=tty, baudrate=9600, dsrdtr=1)
        self.buf = []
        self.lock = threading.Lock()
        self.handlers = []

    ## internal data-handling methods
    def recv_packet(self, timeout=None):
        t0 = time.time()
        self.ser.timeout = None
        while timeout is None or time.time() < t0 + timeout:
            if timeout is not None:
                self.ser.timeout = t0 + timeout - time.time()
            c = self.ser.read()
            if not c:
                return None

            ret = self.proc_byte(ord(c))
            if ret:
                if ret.typ == 0x80:
                    self.handle_event(ret)
                return ret

    def recv_packets(self, timeout=.5):
        res = []
        t0 = time.time()
        while time.time() < t0 + timeout:
            p = self.recv_packet(t0 + timeout - time.time())
            if not p: return res
            res.append(p)
        return res

    def proc_byte(self, c):
        if not self.buf:
            if c in [0x00, 0x80, 0x08, 0x88]:
                self.buf.append(c)
            return None
        elif len(self.buf) == 1:
            self.buf.append(c)
            self.packet_len = 4 + (self.buf[0] & 0x07) + self.buf[1]
            return None
        else:
            self.buf.append(c)

        if self.packet_len and len(self.buf) == self.packet_len:
            p = Packet(self.buf)
            self.buf = []
            return p
        return None

    def handle_event(self, p):
        for h in self.handlers:
            h(p)

    def add_handler(self, h):
        self.handlers.append(h)

    def remove_handler(self, h):
        try: self.handlers.remove(h)
        except ValueError: pass

    def wait_event(self, cls, cmd):
        res = [None]
        def h(p):
            if p.cls == cls and p.cmd == cmd:
                res[0] = p
        self.add_handler(h)
        while res[0] is None:
            self.recv_packet()
        self.remove_handler(h)
        return res[0]

    ## specific BLE commands
    def connect(self, addr):
        return self.send_command(6, 3, pack('6sBHHHH', multichr(addr), 0, 6, 6, 64, 0))

    def get_connections(self):
        return self.send_command(0, 6)

    def discover(self):
        return self.send_command(6, 2, b'\x01')

    def end_scan(self):
        return self.send_command(6, 4)

    def disconnect(self, h):
        return self.send_command(3, 0, pack('B', h))

    def read_attr(self, con, attr):
        self.send_command(4, 4, pack('BH', con, attr))
        return self.wait_event(4, 5)

    def write_attr(self, con, attr, val):
        self.send_command(4, 5, pack('BHB', con, attr, len(val)) + val)
        return self.wait_event(4, 1)

    def send_command(self, cls, cmd, payload=b'', wait_resp=True):
        s = pack('4B', 0, len(payload), cls, cmd) + payload
        self.ser.write(s)

        while True:
            p = self.recv_packet()

            ## no timeout, so p won't be None
            if p.typ == 0: return p

            ## not a response: must be an event
            self.handle_event(p)


class MyoRaw(object):
    '''Implements the Myo-specific communication protocol.'''

    def __init__(self, tty=None):
        if tty is None:
            tty = self.detect_tty()
        if tty is None:
            raise ValueError('Myo dongle not found!')

        self.bt = BT(tty)
        self.conn = None
        self.emg_handlers = []
        self.imu_handlers = []
        self.arm_handlers = []
#       self.pose_handlers = [] #leo
        self.battery_handlers = []      #ALVIPE

    def detect_tty(self):
        for p in comports():
            if re.search(r'PID=2458:0*1', p[2]):
                print('using device:', p[0])
                return p[0]

        return None

    def run(self, timeout=None):
        self.bt.recv_packet(timeout)

    def connect(self):
        ## stop everything from before
        self.bt.end_scan()
        self.bt.disconnect(0)
        self.bt.disconnect(1)
        self.bt.disconnect(2)

        ## start scanning
        print('scanning...')
        self.bt.discover()
        while True:
            p = self.bt.recv_packet()
            print('scan response:', p)

            if p.payload.endswith(b'\x06\x42\x48\x12\x4A\x7F\x2C\x48\x47\xB9\xDE\x04\xA9\x01\x00\x06\xD5'):
                addr = list(multiord(p.payload[2:8]))
                break
        self.bt.end_scan()

        ## connect and wait for status event
        conn_pkt = self.bt.connect(addr)
        self.conn = multiord(conn_pkt.payload)[-1]
        self.bt.wait_event(3, 0)

        ## get firmware version
        fw = self.read_attr(0x17)
        _, _, _, _, v0, v1, v2, v3 = unpack('BHBBHHHH', fw.payload)
        print('firmware version: %d.%d.%d.%d' % (v0, v1, v2, v3))

        self.old = (v0 == 0)

        if self.old:
            ## don't know what these do; Myo Connect sends them, though we get data
            ## fine without them
            self.write_attr(0x19, b'\x01\x02\x00\x00')
            self.write_attr(0x2f, b'\x01\x00')
            self.write_attr(0x2c, b'\x01\x00')
            self.write_attr(0x32, b'\x01\x00')
            self.write_attr(0x35, b'\x01\x00')

            ## enable EMG data
            self.write_attr(0x28, b'\x01\x00')
            ## enable IMU data
            self.write_attr(0x1d, b'\x01\x00')

            ## Sampling rate of the underlying EMG sensor, capped to 1000. If it's
            ## less than 1000, emg_hz is correct. If it is greater, the actual
            ## framerate starts dropping inversely. Also, if this is much less than
            ## 1000, EMG data becomes slower to respond to changes. In conclusion,
            ## 1000 is probably a good value.
            C = 1000
            emg_hz = 50
            ## strength of low-pass filtering of EMG data
            emg_smooth = 100

            imu_hz = 50

            ## send sensor parameters, or we don't get any data
            self.write_attr(0x19, pack('BBBBHBBBBB', 2, 9, 2, 1, C, emg_smooth, C // emg_hz, imu_hz, 0, 0))

        else:
            name = self.read_attr(0x03)
            print('device name: %s' % name.payload)

            ## enable IMU data
            self.write_attr(0x1d, b'\x01\x00')
            ## enable on/off arm notifications
            self.write_attr(0x24, b'\x02\x00')

            # self.write_attr(0x19, b'\x01\x03\x00\x01\x01')
            self.start_raw()
           # enable battery notifications               #ALVIPE
            self.write_attr(0x12, b'\x01\x10')          #ALVIPE
            
        
        ## add data handlers
        def handle_data(p):
            if (p.cls, p.cmd) != (4, 5): return

            c, attr, typ = unpack('BHB', p.payload[:4])
            pay = p.payload[5:]

            if attr == 0x27:
                vals = unpack('8HB', pay)
                ## not entirely sure what the last byte is, but it's a bitmask that
                ## seems to indicate which sensors think they're being moved around or
                ## something
                emg = vals[:8]
                print(emg)
                moving = vals[8]
                self.on_emg(emg, moving)
                
                 # Read notification handles corresponding to the for EMG characteristics      #ALVIPE
            elif attr == 0x2b or attr == 0x2e or attr == 0x31 or attr == 0x34:                        #ALVIPE
                '''According to http://developerblog.myo.com/myocraft-emg-in-the-bluetooth-protocol/    #ALVIPE
                each characteristic sends two secuential readings in each update,                      #ALVIPE
                so the received payload is split in two samples. According to the                      #ALVIPE
                Myo BLE specification, the data type of the EMG samples is int8_t.                    #ALVIPE
                '''                                                                                  #ALVIPE
                emg1 = struct.unpack('<8b', pay[:8])                                                    #ALVIPE
                emg2 = struct.unpack('<8b', pay[8:])                                                    #ALVIPE
                self.on_emg(emg1, 0)                                                                    #ALVIPE
                self.on_emg(emg2, 0)                                                                    #ALVIPE
            
            # Read IMU characteristic handle    
            elif attr == 0x1c:
                vals = unpack('10h', pay)
                quat = vals[:4]
                acc = vals[4:7]
                gyro = vals[7:10]
                self.on_imu(quat, acc, gyro)
            elif attr == 0x23:
                typ, val, xdir, _,_,_ = unpack('6B', pay)

                if typ == 1: # on arm
                    self.on_arm(Arm(val), XDirection(xdir))
                elif typ == 2: # removed from arm
                    self.on_arm(Arm.UNKNOWN, XDirection.UNKNOWN)
                #elif typ == 3: # pose        #leo
                #   self.on_pose(Pose(val)) #leo
        
            # Read battery characteristic handle        #ALVIPE
            elif attr == 0x11:                  #ALVIPE
                battery_level = ord(pay)                    #ALVIPE
                self.on_battery(battery_level)                  #ALVIPE
            else:
                print('data with unknown attr: %02X %s' % (attr, p))

        self.bt.add_handler(handle_data)


    def write_attr(self, attr, val):
        if self.conn is not None:
            self.bt.write_attr(self.conn, attr, val)

    def read_attr(self, attr):
        if self.conn is not None:
            return self.bt.read_attr(self.conn, attr)
        return None

    def disconnect(self):
        if self.conn is not None:
            self.bt.disconnect(self.conn)
                                                               #ALVIPE
    def sleep_mode(self, mode):                             #ALVIPE
        self.write_attr(0x19, pack('3B', 9, 1, mode))         #ALVIPE
                                                               #ALVIPE
    def power_off(self):                                       #ALVIPE
        self.write_attr(0x19, b'\x04\x00')                   #ALVIPE

    def start_raw(self):
        '''Sending this sequence for v1.0 firmware seems to enable both raw data and
        pose notifications.
        '''

        ''' To get raw EMG signals, we subscribe to the four EMG notification
        characteristics by writing a 0x0100 command to the corresponding handles.
        '''
        self.write_attr(0x2c, b'\x01\x00')  # Suscribe to EmgData0Characteristic           #RAW_ALVIPE
        self.write_attr(0x2f, b'\x01\x00')  # Suscribe to EmgData1Characteristic           #RAW_ALVIPE
        self.write_attr(0x32, b'\x01\x00')  # Suscribe to EmgData2Characteristic           #RAW_ALVIPE
        self.write_attr(0x35, b'\x01\x00')  # Suscribe to EmgData3Characteristic           #RAW_ALVIPE

        '''Bytes sent to handle 0x19 (command characteristic) have the following
        format: [command, payload_size, EMG mode, IMU mode, classifier mode]
        According to the Myo BLE specification, the commands are:
            0x01 -> set EMG and IMU
            0x03 -> 3 bytes of payload
            0x02 -> send 50Hz filtered signals
            0x01 -> send IMU data streams
            0x01 -> send classifier events
        '''
        self.write_attr(0x19, b'\x01\x03\x02\x01\x01')                                    #RAW_ALVIPE

        '''Sending this sequence for v1.0 firmware seems to enable both raw data and
        pose notifications.
        '''

        '''By writting a 0x0100 command to handle 0x28, some kind of "hidden" EMG
        notification characteristic is activated. This characteristic is not
        listed on the Myo services of the offical BLE specification from Thalmic
        Labs. Also, in the second line where we tell the Myo to enable EMG and
        IMU data streams and classifier events, the 0x01 command wich corresponds
        to the EMG mode is not listed on the myohw_emg_mode_t struct of the Myo
        BLE specification.
        These two lines, besides enabling the IMU and the classifier, enable the
        transmission of a stream of low-pass filtered EMG signals from the eight
        sensor pods of the Myo armband (the "hidden" mode I mentioned above).
        Instead of getting the raw EMG signals, we get rectified and smoothed
        signals, a measure of the amplitude of the EMG (which is useful to have
        a measure of muscle strength, but are not as useful as a truly raw signal).
        '''

        #self.write_attr(0x28, b'\x01\x00')                    #RAW_ALVIPE: COMENTADO
        #self.write_attr(0x19, b'\x01\x03\x01\x01\x01')        #RAW_ALVIPE: COMENTADO

    def mc_start_collection(self):
        '''Myo Connect sends this sequence (or a reordering) when starting data
        collection for v1.0 firmware; this enables raw data but disables arm and
        pose notifications.
        '''

        self.write_attr(0x28, b'\x01\x00')
        self.write_attr(0x1d, b'\x01\x00')
        self.write_attr(0x24, b'\x02\x00')
        self.write_attr(0x19, b'\x01\x03\x01\x01\x01')
        self.write_attr(0x28, b'\x01\x00')
        self.write_attr(0x1d, b'\x01\x00')
        self.write_attr(0x19, b'\x09\x01\x01\x00\x00')
        self.write_attr(0x1d, b'\x01\x00')
        self.write_attr(0x19, b'\x01\x03\x00\x01\x00')
        self.write_attr(0x28, b'\x01\x00')
        self.write_attr(0x1d, b'\x01\x00')
        self.write_attr(0x19, b'\x01\x03\x01\x01\x00')

    def mc_end_collection(self):
        '''Myo Connect sends this sequence (or a reordering) when ending data collection
        for v1.0 firmware; this reenables arm and pose notifications, but
        doesn't disable raw data.
        '''

        self.write_attr(0x28, b'\x01\x00')
        self.write_attr(0x1d, b'\x01\x00')
        self.write_attr(0x24, b'\x02\x00')
        self.write_attr(0x19, b'\x01\x03\x01\x01\x01')
        self.write_attr(0x19, b'\x09\x01\x00\x00\x00')
        self.write_attr(0x1d, b'\x01\x00')
        self.write_attr(0x24, b'\x02\x00')
        self.write_attr(0x19, b'\x01\x03\x00\x01\x01')
        self.write_attr(0x28, b'\x01\x00')
        self.write_attr(0x1d, b'\x01\x00')
        self.write_attr(0x24, b'\x02\x00')
        self.write_attr(0x19, b'\x01\x03\x01\x01\x01')

    def vibrate(self, length):
        if length in range(1, 4):
            ## first byte tells it to vibrate; purpose of second byte is unknown
            self.write_attr(0x19, pack('3B', 3, 1, length))
    def set_leds(self, logo, line):                                              #ALVIPE
        self.write_attr(0x19, pack('8B', 6, 6, *(logo + line)))                  #ALVIPE

    def add_emg_handler(self, h):
        self.emg_handlers.append(h)

    def add_imu_handler(self, h):
        self.imu_handlers.append(h)

#   def add_pose_handler(self, h):   #leo
#       self.pose_handlers.append(h)   #leo

    def add_arm_handler(self, h):
        self.arm_handlers.append(h)
        
    def add_battery_handler(self, h):           #ALVIPE
        self.battery_handlers.append(h)     #ALVIPE

    def on_emg(self, emg, moving):
        for h in self.emg_handlers:
            h(emg, moving)

    def on_imu(self, quat, acc, gyro):
        for h in self.imu_handlers:
            h(quat, acc, gyro)

#   def on_pose(self, p):             #leo
#       for h in self.pose_handlers:   #leo
#           h(p)                       #leo

    def on_arm(self, arm, xdir):
        for h in self.arm_handlers:
            h(arm, xdir)

    def on_battery(self, battery_level):                #ALVIPE
        for h in self.battery_handlers:                 #ALVIPE
            h(battery_level)                            #ALVIPE




if __name__ == '__main__':
    
    GPIO.setmode(GPIO.BOARD) # Modo de GPIO: numeración de pines
    
    # Definir modo de GPIO. Activar resistencia pull-down
    GPIO.setup(micro_flag, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    GPIO.setup(pin_inicio, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(safe_shut, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(pin_help, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(micro0,GPIO.OUT)
    GPIO.setup(micro1,GPIO.OUT)
    GPIO.setup(micro2,GPIO.OUT)
    GPIO.setup(led_inicio,GPIO.OUT)
    
    # Inicializar pines de comunicación
    GPIO.output(micro0, GPIO.LOW)
    GPIO.output(micro1, GPIO.LOW)
    GPIO.output(micro2, GPIO.LOW)
    
    GPIO.output(led_inicio, GPIO.HIGH)
    time.sleep(5)
    GPIO.output(led_inicio, GPIO.LOW)
    
    # Activar interrupción para pin de "iniciar sistema"
    # Detectar flanco de subida
    GPIO.add_event_detect(pin_inicio, GPIO.RISING, callback = iniciarSistema, bouncetime = 700)
    GPIO.add_event_detect(safe_shut, GPIO.RISING, callback = apagarSistema, bouncetime = 500)
    GPIO.add_event_detect(pin_help, GPIO.RISING, callback = ayudaSistema, bouncetime = 500)
    
    while(iniciar_sis == 0):
        time.sleep(3)
        print("IS: ", iniciar_sis)
        print("Esperando...")
    
    # Activar interrupción para pin de "leer nuevo movimiento"
    # Detectar flanco de subida
    GPIO.add_event_detect(micro_flag, GPIO.RISING, callback = leerMicro, bouncetime = 100)  
    
    # Para eventos del teclado
    #s=TerminalFile(sys.stdin)   
    
    # Crear objeto tipo MyoRaw para adquirir las señales emg
    m = MyoRaw(sys.argv[1] if len(sys.argv) >= 2 else None)
    
    def proc_emg(emg, moving, times=[]):
                    
        global clasificador, captura, contador, sensor, emg_buffer, media_aux, media
        global clasif_mod2, startTime, iniciar_sis, movcount
        global iduser, idtest, idmove
        
        if (clasificador & (contador < Ndatosclasif) & (iniciar_sis == 1)):
#             if(contador == 0):
#                 startTime=time.time()
            for sensor in range(8):
                emg_buffer [sensor] [contador] = emg[sensor]
            contador +=1
            
    
            if contador >= Ndatosclasif:
#                 m.vibrate(1)   
                endTime=time.time()
                print("----------------------------------------")
                print(" La captura de datos ha finalizado" )
#                 print("Los datos capturados son:")
                media_auxT=(np.array(emg_buffer)).astype(np.float)  # Verificar si funciona mejor como entero o float
                media_auxT=media_auxT.transpose()
#                 print(media_auxT)
                print("----------------------------------------")
                totalTime=endTime-startTime
                print("Tiempo: ", totalTime)
                print("----------------------------------------")
                print(" Numero de datos capturados:")
                print(contador)
                
               
                feat_win_1 = featExtract(media_auxT[0:60,:], 60)
                feat_win_2 = featExtract(media_auxT[60:120,:], 60)
                feat_win_3 = featExtract(media_auxT[120:180,:], 60)
                feat_win_4 = featExtract(media_auxT[30:90,:], 60)
                feat_win_5 = featExtract(media_auxT[90:150,:], 60)

                X_sample=np.concatenate((feat_win_1, feat_win_2, feat_win_3, feat_win_4, feat_win_5 ), axis=0)
                
                       
                # Normalizar el conjunto de características
                X_test_scaled = scaler.transform(X_sample)
                
                # Clasificar movimiento                
                movPred = mlp.predict(X_test_scaled)
                
                # Obtener la predicción que más se repite              
                gesto = stats.mode(movPred)
                print("Mov: ", movPred)
                
                #mov_actual = int(gesto[0])
                
                #print("MovF: ", mov_actual)
                
                if(GPIO.input(micro_flag) == GPIO.HIGH):
                    mov_actual = int(gesto[0])
                    print("MovF: ", mov_actual)
                    enviarMov(mov_actual)
               
                media_auxT = np.array(0) 
                feat_win_1 = np.array(0)
                feat_win_2 = np.array(0)
                feat_win_3 = np.array(0)
                feat_win_4 = np.array(0)
                feat_win_5 = np.array(0)
                X_sample=np.array(0)
                
                contador = 0
                
                #time.sleep(2)
                
                
                if(GPIO.input(micro_flag) == GPIO.HIGH):
                    if(movcount < 25):
                        clasificador = True
                        startTime=time.time()
                        movcount += 1
                        print("count", movcount)
                    else:
                        #time.sleep(4)
                        movcount = 0
                        clasificador = True
                        startTime=time.time()
                        print("count e", movcount)
                    
                else:
                    clasificador = False
                    
                if(clasif_mod2 == True):
                    clasif_mod2 = False
                    clasificador = False
#                 time.sleep(1)
                #clasificador = False 
                
    
        if captura & (contador <  no_datos):
        
            for sensor in range(8):
                media_aux [sensor] [contador] = emg[sensor]
        
            contador +=1
            if contador >= no_datos:
#                 m.vibrate(1)   
                endTime=time.time()
                print("----------------------------------------")
                print(" La captura de datos ha finalizado" )
                print("Los datos capturados son:")
                media_auxT=(np.array(media_aux)).astype(np.float)  # Verificar si funciona mejor como entero o float
                media_auxT=media_auxT.transpose()
                print(media_auxT)
                print("----------------------------------------")
                
                #idx=2
                fileName="u"+str(iduser)+"m"+str(idmove)+"t"+str(idtest)+".csv"
                saveData(media_auxT, fileName)
                
                totalTime=endTime-startTime
                print("Tiempo: ", totalTime)
                print("----------------------------------------")
                print(" Numero de datos capturados:")
                print(contador)
                
        ################## Dividir datos en ventanas ######################
               
                feat_win_1 = featExtract(media_auxT[0:60,:], 60)
                feat_win_2 = featExtract(media_auxT[60:120,:], 60)
                feat_win_3 = featExtract(media_auxT[120:180,:], 60)
                
        ################################## Matriz de características ######################
                
                feat_matrix = np.concatenate((feat_win_1, feat_win_2, feat_win_3), axis=0)
                print("Matriz de características")
                print(feat_matrix)
                
                fileNameC="u"+str(iduser)+"m"+str(idmove)+"t"+str(idtest)+"caract.csv"
                saveData(feat_matrix, fileNameC)
                print("Matriz de características guardada")
                print("Usuario ", iduser, " Movimiento ", idmove, " Prueba ", idtest)
    
                
                media_auxT = np.array(0) 
                media_aux = np.array(0) 
                feat_win_1 = np.array(0)
                feat_win_2 = np.array(0)
                feat_win_3 = np.array(0)
                feat_matrix = np.array(0) 
                
                
                if idmove==10 and idtest==6:
                    idmove=0
                    iduser=iduser+1
                    
                if idtest==6:
                    idtest=0
                    idmove=idmove+1
                
                contador = 0
                captura = False
                

#########################################################################################
                
        
        ## print framerate of received data
        times.append(time.time())
        if len(times) > 20:
            #print((len(times) - 1) / (times[-1] - times[0]))
            times.pop(0)
            
            
    def proc_battery(battery_level):                            #ALVIPE
        print("Nivel de bateria: %d" % battery_level)              #ALVIPE
        if battery_level < 5:                                   #ALVIPE
            m.set_leds([255, 0, 0], [255, 0, 0])                #ALVIPE
        else:                                                   #ALVIPE
            m.set_leds([128, 128, 255], [128, 128, 255])        #ALVIPE


    
    m.add_emg_handler(proc_emg)
    m.add_battery_handler(proc_battery)                         #ALVIPE
    m.connect()
    
    m.add_arm_handler(lambda arm, xdir: print('arm', arm, 'xdir', xdir))
    #m.add_pose_handler(lambda p: print('pose', p))

    m.sleep_mode(1)             #ALVIPE
    #m.set_leds([128, 128, 255], [128, 128, 255])  # purple logo and bar LEDs    #ALVIPE
    #m.vibrate(1)                #ALVIPE

    
    try:
#       global captura,clasificador
        print ("Pulse q para salir...")
        print ("Pulse s para capturar datos")
        print ("Pulse c para iniciar el clasificador")
        print ("Pulse x para volver a opciones")
        #GPIO.output(11,0)
        
        while True:
            m.run(1)
            #teclado = s.getch()
            teclado = 2
            
            if (primer_mov == 0):
                if(GPIO.input(micro_flag) == GPIO.HIGH):
                    print("Clasificar gesto")
                    m.vibrate(1) 
                    #clasif_mod2 = True
                    clasificador = True
                    contador=0
                    primer_mov = 1
                    startTime=time.time()
            
            if (teclado == " "):
                continue
            elif (teclado == "q"):
                raise KeyboardInterrupt()
            elif (teclado == "x"):
                captura = False
                clasificador = False
                contador=0
                print ("Pulse q para salir...")
                print ("Pulse s para capturar datos")
                print ("Pulse c para iniciar el clasificador")
            elif (teclado == "s"):

                idtest=idtest+1            
                
                print("Iniciando captura de datos: ")
                startTime=time.time()
                captura = True
                clasificador = False
                contador=0 

            elif (ayuda_sis == 1):
                 
                print("Clasificar gesto")
                clasif_mod2 = True
                clasificador = True
                contador=0
                startTime=time.time()
                ayuda_sis = 0
                
            elif (apagar == 1):
                GPIO.output(led_inicio, GPIO.HIGH)
                time.sleep(0.3)
                GPIO.output(led_inicio, GPIO.LOW)
                raise KeyboardInterrupt()
            

        
    except KeyboardInterrupt:
        pass
    
    finally:

        #GPIO.output(11,0)
        m.vibrate(1)    
        m.disconnect()
        print("Desconectado")
        GPIO.cleanup()
        
        if (apagar == 1):
            print("Apagar sistema")
            os.system("shutdown now -h")
        
        
        
        
        
        
        
        
        
        