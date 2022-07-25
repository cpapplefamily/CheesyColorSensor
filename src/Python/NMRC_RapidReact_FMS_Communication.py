import os
import serial
import serial.tools.list_ports
import requests
import time
import _thread as thread
import websocket

FMS_IP = "10.0.100.05"
#FMS_IP = "192.168.1.92"
FMS_PORT = "8080"
FMS_SERVER = FMS_IP + ":" + FMS_PORT
ALLIANCE_COLOR = 'red' # Change accordingly
#ALLIANCE_COLOR = 'blue' # Change accordingly
USERNAME = 'admin'
PASSWORD = 'ProliantDL160'

goal_char_msg_map = {
    "S": '{ "type": "RU" }',
    "X": '{ "type": "RL" }',
    "Y": '{ "type": "BU" }',
    "H": '{ "type": "BL" }'
}

# Return the first arduino mega connected to PC
def find_arduino_port(): 
    for port in serial.tools.list_ports.comports():
        print('PortID: ' + port.hwid)
        if 'VID:PID=10C4:EA60' in port.hwid:   
            return port.device

# Retur na char recieved from arduino
def get_goal_char(connection):
    connection.reset_input_buffer()
    goal_char = connection.read(1).decode('UTF-8')
    return goal_char

def get_msg_from_goal_char(goal_char):
    return goal_char_msg_map[goal_char]

def get_on_ws_open_callback(connection):
    def on_ws_open(ws):
        print("Connected to FMS")

        def run(*args):
            while(True):
                goal_char = get_goal_char(connection)
                print(f'Info: recieved "{goal_char}"')

                if (goal_char in goal_char_msg_map):
                    print(f'Info: sent {get_msg_from_goal_char(goal_char)}')
                    ws.send(get_msg_from_goal_char(goal_char))
                else:
                    print('Error: unknown char recieved')
                #Give Sign of Life
                print('Info sent to Arduino')
                connection.write(1)
            

        thread.start_new_thread(run, ())
    
    return on_ws_open
    
def open_websocket(serial_connection):
    def reopen_websocket():
        open_websocket(serial_connection)

    res = requests.post(f'http://{FMS_SERVER}/login'
        , data={'username': USERNAME, 'password': PASSWORD}
        , allow_redirects=False
    )

    ws = websocket.WebSocketApp(f'ws://{FMS_SERVER}/panels/scoring/{ALLIANCE_COLOR}/websocket'
        , on_open=get_on_ws_open_callback(serial_connection)
        , on_close=reopen_websocket
        , cookie="; ".join(["%s=%s" %(i, j) for i, j in res.cookies.get_dict().items()])
    )

    ws.run_forever()

def main():
    
    while(True):
        print('Find arduino')
        connection = serial.Serial(find_arduino_port(), 9600)

        if (connection.is_open):
            print("Connected to arduino")
            break
        time.sleep(2)
    
    """Loops until a connection is made to the FMS.
    When connected opens a websocket 
    """
    #Wait for Network connection to FMS
    while(True):
        print(f'Check Network Connection {FMS_IP}')
        response = os.system("ping -c 1 " + FMS_IP)
        if response == 0:
            print(f'{FMS_IP} Found')
        else:
            print("Network Error")
        if(response == 0): break
        time.sleep(2)
        
    open_websocket(connection)

main()
