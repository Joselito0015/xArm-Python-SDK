#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2022, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

"""
# Notice
#   1. Changes to this file on Studio will not be preserved
#   2. The next conversion will overwrite the file with the same name
# 
# xArm-Python-SDK: https://github.com/xArm-Developer/xArm-Python-SDK
#   1. git clone git@github.com:xArm-Developer/xArm-Python-SDK.git
#   2. cd xArm-Python-SDK
#   3. python setup.py install
"""
import sys
import math
import time
import queue
import datetime
import random
import traceback
import threading
from xarm import version
from xarm.wrapper import XArmAPI
import websocket
import threading
import json


def on_message(ws, message):
    # print(f"Mensaje recibido: {message}")
    #string to json
    json_message = json.loads(message)
    Parameters=json_message["state"]["desired"]
    print(Parameters)
    Action=Parameters["action"]

    if Action == "almacenar" or Action == "retirar":
        x=Parameters["X"]
        y=Parameters["Y"]
        Almacen=Parameters["Almacen"]
        #tipo=Parameters["type"]

        robot_main._vars['level'] = int(y)
        robot_main._vars['shelf'] = int(Almacen)
        robot_main._vars['position'] = int(x)

        if Action == "almacenar":
            robot_main._vars['Request'] = 1 #step
        elif Action == "retirar":
            robot_main._vars['Request'] = 2 #step

        robot_main._vars['Request_flag'] = 1 #step
    


        print(x,y,Action,Almacen)
        print(robot_main._vars['level'])
        print(robot_main._vars['shelf'])
        print(robot_main._vars['position'])
    elif Action == "stopped":
        robot_main._vars['PLC_Stop_Position'] = 1 #step
        robot_main._vars['Off_Home'] = 1 #step
        
        print("Motor encendido")


def on_error(ws, error):
    print(f"Error: {error}")

def on_close(ws):
    print("Conexión cerrada")


def on_open(ws):
    print("Conexión abierta")

def on_send(ws,type,X,Y,Almacen,action):
    print("Conexión abierta")
    # Puedes enviar un mensaje apenas se establezca la conexión
    payload={
    "state": {
        "desired": {
        "action": action, #MOVE|DONE
        "type":type,
        "status": "waiting",
        "Almacen": Almacen, #Almacen 1|2
        "X": X,
        "Y":Y
            }
        }
    }

    json_payload = json.dumps(payload)

    ws.send(json_payload)


def start_websocket(ws):
    websocket.enableTrace(False)  # Esto permite ver información de depuración
    time.sleep(5)
    ws.run_forever()


    
# Reemplaza 'wss://echo.websocket.org' con la URL del servidor WebSocket al que te quieres conectar
ws = websocket.WebSocketApp("ws://localhost:1880/ws/master",
                            on_message=on_message,
                            on_error=on_error,
                            on_close=on_close,
                            
                            )
ws.on_open = on_open

# Inicia el cliente WebSocket en un hilo separado para que no bloquee la ejecución del programa principal
websocket_thread = threading.Thread(target=start_websocket, args=(ws,))
websocket_thread.start()


class RobotMain(object):
    """Robot Main Class"""
    def __init__(self, robot, **kwargs):
        self.alive = True
        self._arm = robot
        self._tcp_speed = 100
        self._tcp_acc = 2000
        self._angle_speed = 20
        self._angle_acc = 500
        self._vars = {'y': 0, 'PLC_Run_Position': 0, 'level': 0, 'Off_Flag': 0, 'deep_l': 0, 'CurrectPosition': 0, 'p_x': 0, 'y_Buffer': 0, 'Off_RFID': 0, 'p_z': 0, 'Yaw_D': 0, 'p_y': 0, 'Off_Home': 0, 'Request_flag': 0, 'On_RFID': 0, 'x': 0, 'PLC_direction': 0, 'On_Home': 0, 'z_Buffer': 0, 'last_position': 0, 'x_Buffer': 0, 'PLC_Stop_Position': 0, 'p_yaw': 0, 'Request': 0, 'NewPosition': 0, 'shelf': 0, 'PLC_steps': 0, 'position': 0, 'z': 0, 'RunPlatform': 0}
        self._funcs = {
            "POS RFID": self.function_7,
            "RecogerPallet": self.function_4,
            "DepositarPallet": self.function_6,
            "Posicion_PLC": self.function_1,
            "FuncionPosicion": self.function_5,
            "POS_Zero": self.function_2,
            "Funcion_Home": self.function_3,
        }
        self._robot_init()
        time.sleep(10)
        on_send(ws,5,3,3,2,"MOVE")
        print("Mensaje enviado")
    # Robot init
    def _robot_init(self):
        self._arm.clean_warn()
        self._arm.clean_error()
        self._arm.motion_enable(True)
        self._arm.set_mode(0)
        self._arm.set_state(0)
        time.sleep(1)
        self._arm.register_error_warn_changed_callback(self._error_warn_changed_callback)
        self._arm.register_state_changed_callback(self._state_changed_callback)
        if hasattr(self._arm, 'register_count_changed_callback'):
            self._arm.register_count_changed_callback(self._count_changed_callback)

    # Register error/warn changed callback
    def _error_warn_changed_callback(self, data):
        if data and data['error_code'] != 0:
            self.alive = False
            self.pprint('err={}, quit'.format(data['error_code']))
            self._arm.release_error_warn_changed_callback(self._error_warn_changed_callback)

    # Register state changed callback
    def _state_changed_callback(self, data):
        if data and data['state'] == 4:
            self.alive = False
            self.pprint('state=4, quit')
            self._arm.release_state_changed_callback(self._state_changed_callback)

    # Register count changed callback
    def _count_changed_callback(self, data):
        if self.is_alive:
            self.pprint('counter val: {}'.format(data['count']))

    def _check_code(self, code, label):
        if not self.is_alive or code != 0:
            self.alive = False
            ret1 = self._arm.get_state()
            ret2 = self._arm.get_err_warn_code()
            self.pprint('{}, code={}, connected={}, state={}, error={}, ret1={}. ret2={}'.format(label, code, self._arm.connected, self._arm.state, self._arm.error_code, ret1, ret2))
        return self.is_alive

    @staticmethod
    def pprint(*args, **kwargs):
        try:
            stack_tuple = traceback.extract_stack(limit=2)[0]
            print('[{}][{}] {}'.format(time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(time.time())), stack_tuple[1], ' '.join(map(str, args))))
        except:
            print(*args, **kwargs)

    @property
    def arm(self):
        return self._arm

    @property
    def VARS(self):
        return self._vars

    @property
    def FUNCS(self):
        return self._funcs

    @property
    def is_alive(self):
        if self.alive and self._arm.connected and self._arm.error_code == 0:
            if self._arm.state == 5:
                cnt = 0
                while self._arm.state == 5 and cnt < 5:
                    cnt += 1
                    time.sleep(0.1)
            return self._arm.state < 4
        else:
            return False

    def function_1(self, arg_1, arg_2, arg_3):
        """
        Posicion_PLC
        """
        # Enviar PLC_Run_position al PLC
        self._vars['PLC_Run_Position'] = 1
        print("Se envió Señal Run_position al PLC{}".format(self._vars.get('On_Home', 0)))
        # enviar posición 
        time.sleep(1)
        # Recibir señal de finalizar del PLC
        while self.is_alive and self._vars.get('PLC_Stop_Position', 0) == 0:
            # Lectura de señal de finalizar del PLC
            print("Esperando señal de finalizar movimiento de stepper del PLC")
            print("Se recibió señal de finalizar mov stepper del PLC")
            time.sleep(4)
        self._vars['PLC_Run_Position'] = 0
        self._vars['PLC_Stop_Position'] = 0

    def function_2(self):
        """
        POS_Zero
        """
        code = self._arm.set_position(*[207,0,112,180,0,45], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1, wait=True)
        if not self._check_code(code, 'set_position'):
            return
        print("posicion cero")

    def function_5(self):
        """
        FuncionPosicion
        """
        for i in range(int(1)):
            if not self.is_alive:
                break
            if self._vars.get('shelf', 0) == 1:
                # Estante derecho
                self._vars['p_y'] = -207
                self._vars['p_yaw'] = -135
                self._vars['deep_l'] = -101
            elif self._vars.get('shelf', 0) == 2:
                # Estante izquierdo
                self._vars['p_y'] = 207
                self._vars['p_yaw'] = 135
                self._vars['deep_l'] = 101
            else:
                self._vars['p_y'] = 0
                self._vars['p_yaw'] = 45
            if self._vars.get('level', 0) == 1:
                self._vars['p_x'] = 0
                self._vars['p_z'] = 77
                print("Posiciones de Nivel 1 seteadas")
            elif self._vars.get('level', 0) == 2:
                self._vars['p_x'] = 0
                self._vars['p_z'] = 247
                print("Posiciones de Nivel 2 seteadas")
            elif self._vars.get('level', 0) == 3:
                self._vars['p_x'] = 0
                self._vars['p_z'] = 420
            elif self._vars.get('level', 0) == 4:
                self._vars['p_x'] = 0
                self._vars['p_z'] = 588
            elif self._vars.get('level', 0) == 5:
                self._vars['x'] = 0
                self._vars['p_z'] = 758
            elif self._vars.get('level', 0) == 6:
                self._vars['p_x'] = 0
                self._vars['p_z'] = 758
            else:
                self._vars['p_x'] = 207
                self._vars['p_z'] = 112

    def function_3(self):
        """
        Funcion_Home
        """
        # Enviar On_Home al PLC
        # self._vars['On_Home'] = 1
        print("Se envió Señal On_Home  a PLC{}".format(self._vars.get('On_Home', 0)))
        time.sleep(1)
        while self.is_alive and self._vars.get('Off_Home', 0) == 0:
            # Lectura de señal de finalizar del PLC
            print("Esperando señal Off_Home del PLC")
            print("Se recibió señal Off_Home del PLC")
            time.sleep(1)
        self._vars['On_Home'] = 0
        self._vars['Off_Home'] = 0

    def function_7(self):
        """
        POS RFID
        """
        code = self._arm.set_position(*[300,0,340,180,0,self._vars.get('Yaw_D', 0)], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1, wait=True)
        if not self._check_code(code, 'set_position'):
            return

    def function_6(self, arg_1, arg_2, arg_3):
        """
        DepositarPallet
        """
        code = self._arm.set_position(*[arg_1,arg_2,112,180,0,45], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1, wait=True)
        if not self._check_code(code, 'set_position'):
            return
        code = self._arm.set_position(*[arg_1,arg_2,arg_3,180,0,45], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1, wait=True)
        if not self._check_code(code, 'set_position'):
            return
        for i in range(int(1)):
            if not self.is_alive:
                break
            code = self._arm.set_tgpio_digital(0, 0, delay_sec=0)
            if not self._check_code(code, 'set_tgpio_digital'):
                return
            time.sleep(3)
        code = self._arm.set_position(*[arg_1,arg_2,(arg_3 + 40),180,0,45], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1, wait=True)
        if not self._check_code(code, 'set_position'):
            return
        time.sleep(3)
        self.function_2()
        if not self.is_alive:
            return

    def function_4(self, arg_1, arg_2, arg_3):
        """
        RecogerPosicionarRFID
        """
        code = self._arm.set_tgpio_digital(0, 0, delay_sec=0)
        if not self._check_code(code, 'set_tgpio_digital'):
            return
        code = self._arm.set_position(*[arg_1,arg_2,112,180,0,45], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1, wait=True)
        if not self._check_code(code, 'set_position'):
            return
        code = self._arm.set_position(*[arg_1,arg_2,arg_3,180,0,45], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1, wait=True)
        if not self._check_code(code, 'set_position'):
            return
        for i in range(int(1)):
            if not self.is_alive:
                break
            code = self._arm.set_tgpio_digital(0, 1, delay_sec=0)
            if not self._check_code(code, 'set_tgpio_digital'):
                return
            time.sleep(3)

    # Robot Main Run
    def run(self):
        try:
            for i in range(int(1)):
                if not self.is_alive:
                    break
                # Inicializar variables
                self._vars['x_Buffer'] = 207
                self._vars['y_Buffer'] = 0
                self._vars['z_Buffer'] = 42
                self._vars['On_Home'] = 0
                self._vars['Off_Home'] = 0
                self._vars['p_x'] = 207
                self._vars['p_y'] = 0
                self._vars['p_z'] = 112
                self._vars['p_yaw'] = 0
                self._vars['On_RFID'] = 0
                self._vars['Off_RFID'] = 0
                self._vars['Request_flag'] = False
                self._vars['Off_Flag'] = False
                self._vars['PLC_Stop_Position'] = 0
                self.function_2()
                if not self.is_alive:
                    return
                self.function_3()
                if not self.is_alive:
                    return
                while self.is_alive:
                    # Verificar si se recibió peticiones
                    # Recibir señal de petición
                    time.sleep(4)
                    # self._vars['Request_flag'] = True
                    # print("se recibio{}".format(self._vars.get('Request_flag', 0)))
                    # self._vars['Request'] = 2
                    if self._vars.get('Request_flag', 0):
                        if self._vars.get('Request', 0) == 1:
                            # Almacenar
                            for i in range(int(1)):
                                if not self.is_alive:
                                    break
                                print("Ingreso")
                                self.function_3()
                                if not self.is_alive:
                                    return
                                for i in range(int(1)):
                                    if not self.is_alive:
                                        break
                                    self._vars['x_Buffer'] = 207
                                    self._vars['y_Buffer'] = 0
                                    self._vars['z_Buffer'] = 42
                                    self._vars['p_x'] = 207
                                    self._vars['p_y'] = 0
                                    self._vars['p_z'] = 112
                                    self._vars['p_yaw'] = 0
                                    self._vars['PLC_Run_Position'] = 0
                                    self._vars['PLC_Stop_Position'] = 0
                                    code = self._arm.set_tgpio_digital(0, 0, delay_sec=0)
                                    if not self._check_code(code, 'set_tgpio_digital'):
                                        return
                                    # Posición Cero (Brazo)
                                    self.function_2()
                                    if not self.is_alive:
                                        return
                                    time.sleep(2)
                                    print("Robot en posición Cero")
                                    # Recoger pallet 
                                    self.function_4(self._vars.get('x_Buffer', 0),self._vars.get('y_Buffer', 0),self._vars.get('z_Buffer', 0))
                                    if not self.is_alive:
                                        return
                                    print("Pallet recogido")
                                    # Posición Cero (Brazo)
                                    self.function_2()
                                    if not self.is_alive:
                                        return
                                    time.sleep(2)
                                    print("Robot en posición Cero")
                                    # Obtener nivel de estante (1-6) y posición de stepper (1-6)
                                    self._vars['last_position'] = 1
                                    print("Nivel :{}".format(self._vars.get('level', 0)))
                                    print("Posición del stepper :{}".format(self._vars.get('position', 0)))
                                    # Función posición (PLC)
                                    self.function_1(self._vars.get('last_position', 0),self._vars.get('position', 0),self._vars.get('PLC_Run_Position', 0))
                                    if not self.is_alive:
                                        return
                                    print("To PLC: Steps{}".format(self._vars.get('PLC_steps', 0)))
                                    print("Direction{}".format(self._vars.get('PLC_direction', 0)))
                                    print("Run{}".format(self._vars.get('PLC_Run_Position', 0)))
                                    time.sleep(2)
                                    # FunciónPosición_FunciónNivel
                                    print("posición seteada X: {}".format(self._vars.get('p_x', 0)))
                                    print("posición seteada Y: {}".format(self._vars.get('p_y', 0)))
                                    print("posición seteada Z: {}".format(self._vars.get('p_z', 0)))
                                    print("posición seteada yaw: {}".format(self._vars.get('p_yaw', 0)))
                                    time.sleep(2)
                                    self.function_5()
                                    if not self.is_alive:
                                        return
                                    time.sleep(2)
                                    print("posición seteada X: {}".format(self._vars.get('p_x', 0)))
                                    print("posición seteada Y: {}".format(self._vars.get('p_y', 0)))
                                    print("posición seteada Z: {}".format(self._vars.get('p_z', 0)))
                                    print("posición seteada yaw: {}".format(self._vars.get('p_yaw', 0)))
                                    print("Estante:{}".format(self._vars.get('shelf', 0)))
                                    print("Posición del stepper:{}".format(self._vars.get('position', 0)))
                                    print("Nivel:{}".format(self._vars.get('level', 0)))
                                    code = self._arm.set_position(*[207,self._vars.get('p_x', 0),self._vars.get('p_z', 0),180,0,45], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1, wait=True)
                                    if not self._check_code(code, 'set_position'):
                                        return
                                    code = self._arm.set_position(*[self._vars.get('p_x', 0),self._vars.get('p_y', 0),self._vars.get('p_z', 0),180,0,self._vars.get('p_yaw', 0)], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1, wait=True)
                                    if not self._check_code(code, 'set_position'):
                                        return
                                    # cambiar signo dependiendo del lado del estante
                                    code = self._arm.set_position(*[self._vars.get('p_x', 0),(self._vars.get('p_y', 0) + self._vars.get('deep_l', 0)),self._vars.get('p_z', 0),180,0,self._vars.get('p_yaw', 0)], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1, wait=True)
                                    if not self._check_code(code, 'set_position'):
                                        return
                                    time.sleep(1)
                                    code = self._arm.set_position(*[self._vars.get('p_x', 0),(self._vars.get('p_y', 0) + self._vars.get('deep_l', 0)),(self._vars.get('p_z', 0) - 45),180,0,self._vars.get('p_yaw', 0)], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1, wait=True)
                                    if not self._check_code(code, 'set_position'):
                                        return
                                    for i in range(int(1)):
                                        if not self.is_alive:
                                            break
                                        code = self._arm.set_tgpio_digital(0, 0, delay_sec=0)
                                        if not self._check_code(code, 'set_tgpio_digital'):
                                            return
                                        time.sleep(3)
                                    code = self._arm.set_position(*[self._vars.get('p_x', 0),(self._vars.get('p_y', 0) + self._vars.get('deep_l', 0)),self._vars.get('p_z', 0),180,0,self._vars.get('p_yaw', 0)], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1, wait=True)
                                    if not self._check_code(code, 'set_position'):
                                        return
                                    code = self._arm.set_position(*[self._vars.get('p_x', 0),self._vars.get('p_y', 0),self._vars.get('p_z', 0),180,0,self._vars.get('p_yaw', 0)], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1, wait=True)
                                    if not self._check_code(code, 'set_position'):
                                        return
                                    code = self._arm.set_position(*[208,self._vars.get('p_x', 0),self._vars.get('p_z', 0),180,0,45], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1, wait=True)
                                    if not self._check_code(code, 'set_position'):
                                        return
                                    # Posición Cero (Brazo)
                                    self.function_2()
                                    if not self.is_alive:
                                        return
                                    time.sleep(2)
                                    print("Robot en posición Cero")
                            self._vars['Request_flag'] = False
                            print("Lectura final de petición Almacenamiento {}".format(self._vars.get('Request_flag', 0)))
                        elif self._vars.get('Request', 0) == 2:
                            # Retiro
                            for i in range(int(1)):
                                if not self.is_alive:
                                    break
                                self._vars['x_Buffer'] = 207
                                self._vars['y_Buffer'] = 0
                                self._vars['z_Buffer'] = 45.5
                                self._vars['p_x'] = 207
                                self._vars['p_y'] = 0
                                self._vars['p_z'] = 112
                                self._vars['p_yaw'] = 45

                                self._vars['PLC_Run_Position'] = 0
                                code = self._arm.set_tgpio_digital(0, 0, delay_sec=0)
                                if not self._check_code(code, 'set_tgpio_digital'):
                                    return
                                # Posición Cero (Brazo)
                                self.function_2()
                                if not self.is_alive:
                                    return
                                time.sleep(2)
                                print("Robot en posición Cero")
                                # Obtener nivel de estante (1-6) y posición de stepper (1-6)
                                # self._vars['level'] = 3
                                # self._vars['position'] = 3
                                self._vars['last_position'] = 1
                                # self._vars['shelf'] = 2
                                print("Nivel :{}".format(self._vars.get('level', 0)))
                                print("Posición del stepper :{}".format(self._vars.get('position', 0)))
                                # Función posición (PLC)
                                self.function_1(self._vars.get('last_position', 0),self._vars.get('position', 0),self._vars.get('PLC_Run_Position', 0))
                                if not self.is_alive:
                                    return
                                print("To PLC: Steps{}".format(self._vars.get('PLC_steps', 0)))
                                print("Direction{}".format(self._vars.get('PLC_direction', 0)))
                                print("Run{}".format(self._vars.get('PLC_Run_Position', 0)))
                                time.sleep(2)
                                # FunciónPosición_FunciónNivel
                                print("posición seteada X: {}".format(self._vars.get('p_x', 0)))
                                print("posición seteada Y: {}".format(self._vars.get('p_y', 0)))
                                print("posición seteada Z: {}".format(self._vars.get('p_z', 0)))
                                time.sleep(2)
                                self.function_5()
                                if not self.is_alive:
                                    return
                                time.sleep(2)
                                print("posición seteada X: {}".format(self._vars.get('p_x', 0)))
                                print("posición seteada Y: {}".format(self._vars.get('p_y', 0)))
                                print("posición seteada Z: {}".format(self._vars.get('p_z', 0)))
                                print("posición seteada yaw: {}".format(self._vars.get('p_yaw', 0)))
                                code = self._arm.set_position(*[207,self._vars.get('p_x', 0),self._vars.get('p_z', 0),180,0,45], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1, wait=True)
                                if not self._check_code(code, 'set_position'):
                                    return
                                code = self._arm.set_position(*[self._vars.get('p_x', 0),self._vars.get('p_y', 0),self._vars.get('p_z', 0),180,0,self._vars.get('p_yaw', 0)], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1, wait=True)
                                if not self._check_code(code, 'set_position'):
                                    return
                                code = self._arm.set_position(*[self._vars.get('p_x', 0),(self._vars.get('p_y', 0) + self._vars.get('deep_l', 0)),self._vars.get('p_z', 0),180,0,self._vars.get('p_yaw', 0)], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1, wait=True)
                                if not self._check_code(code, 'set_position'):
                                    return
                                code = self._arm.set_position(*[self._vars.get('p_x', 0),(self._vars.get('p_y', 0) + self._vars.get('deep_l', 0)),(self._vars.get('p_z', 0) - 45),180,0,self._vars.get('p_yaw', 0)], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1, wait=True)
                                if not self._check_code(code, 'set_position'):
                                    return
                                for i in range(int(1)):
                                    if not self.is_alive:
                                        break
                                    code = self._arm.set_tgpio_digital(0, 1, delay_sec=0)
                                    if not self._check_code(code, 'set_tgpio_digital'):
                                        return
                                    time.sleep(3)
                                code = self._arm.set_position(*[self._vars.get('p_x', 0),(self._vars.get('p_y', 0) + self._vars.get('deep_l', 0)),self._vars.get('p_z', 0),180,0,self._vars.get('p_yaw', 0)], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1, wait=True)
                                if not self._check_code(code, 'set_position'):
                                    return
                                code = self._arm.set_position(*[self._vars.get('p_x', 0),self._vars.get('p_y', 0),self._vars.get('p_z', 0),180,0,self._vars.get('p_yaw', 0)], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1, wait=True)
                                if not self._check_code(code, 'set_position'):
                                    return
                                code = self._arm.set_position(*[208,self._vars.get('p_x', 0),self._vars.get('p_z', 0),180,0,45], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=-1, wait=True)
                                if not self._check_code(code, 'set_position'):
                                    return
                                # Posición Cero (Brazo)
                                self.function_2()
                                if not self.is_alive:
                                    return
                                time.sleep(2)
                                print("Robot en posición Cero")
                                self.function_3()
                                if not self.is_alive:
                                    return
                                time.sleep(2)
                                self.function_6(self._vars.get('x_Buffer', 0),self._vars.get('y_Buffer', 0),self._vars.get('z_Buffer', 0))
                                if not self.is_alive:
                                    return
                                # Posición Cero (Brazo)
                                self.function_2()
                                if not self.is_alive:
                                    return
                                time.sleep(1)
                                print("Robot en posición Cero")
                        self._vars['Request_flag'] = False
                    print("Lectura final de petición {}".format(self._vars.get('Request_flag', 0)))
                    # Apagar Sistema
                    self._vars['Off_Flag'] = False
                    if self._vars.get('Off_Flag', 0):
                        print("Se apagó el sistema :{}".format(self._vars.get('Off_Flag', 0)))
                        self._vars['Off_Flag'] = False
                        print("Actualización de variable(Off_Flag):{}".format(self._vars.get('Off_Flag', 0)))
                        break
        except Exception as e:
            self.pprint('MainException: {}'.format(e))
        self.alive = False
        self._arm.release_error_warn_changed_callback(self._error_warn_changed_callback)
        self._arm.release_state_changed_callback(self._state_changed_callback)
        if hasattr(self._arm, 'release_count_changed_callback'):
            self._arm.release_count_changed_callback(self._count_changed_callback)


if __name__ == '__main__':
    RobotMain.pprint('xArm-Python-SDK Version:{}'.format(version.__version__))
    arm = XArmAPI('192.168.1.228', baud_checkset=False)
    robot_main = RobotMain(arm)
    robot_main.run()
