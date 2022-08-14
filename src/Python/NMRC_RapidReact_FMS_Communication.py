import os
import serial
import serial.tools.list_ports
import requests
import time
import _thread as thread
import websocket
import json

FMS_IP = "10.0.100.05"
#FMS_IP = "192.168.1.187"
FMS_PORT = "8080"
FMS_SERVER = FMS_IP + ":" + FMS_PORT
ALLIANCE_COLOR = 'red' # Change accordingly
#ALLIANCE_COLOR = 'blue' # Change accordingly
USERNAME = 'admin'
PASSWORD = 'ProliantDL160'

curent_matchState = '9'
last_matchState = '0'

goal_char_msg_map = {
    "S": '{ "type": "RU" }',
    "X": '{ "type": "RL" }',
    "Y": '{ "type": "BU" }',
    "H": '{ "type": "BL" }'
}

matchState_char_msg_map = {
    "0": '20',
    "1": '21',
    "2": '22',
    "3": '23',
    "4": '24',
    "5": '25',
    "6": '26',
    "7": '27',
    "8": '28',
    "9": '29'
}

# Return the first arduino mega connected to PC
def find_arduino_port(): 
    for port in serial.tools.list_ports.comports():
        print('PortID: ' + port.hwid)
        if 'VID:PID=10C4:EA60' in port.hwid:   
            return port.device

# Retur na char recieved from arduino
def get_goal_char(connection):
    #connection.reset_input_buffer()
    if (connection.inWaiting() > 0):
        goal_char = connection.read(1).decode('UTF-8')
    else:
        goal_char = "0"
    return goal_char

def get_msg_from_goal_char(goal_char):
    return goal_char_msg_map[goal_char]

def get_msg_from_MatchStat_char(MatchStat_char):
    return matchState_char_msg_map[MatchStat_char]

def int_to_bytes(x: int) -> bytes:
    return x.to_bytes((x.bit_length() + 7) // 8, 'big')

def get_on_ws_open_callback(connection):
    def on_ws_open(ws):
        print("Connected to FMS")

        def run(*args):
            global curent_matchState
            global last_matchState
            while(True):
                goal_char = get_goal_char(connection)
                if(goal_char != "0"):
                    print(f'Info: recieved "{goal_char}"')

                    if (goal_char in goal_char_msg_map):
                        print(f'Info: sent {get_msg_from_goal_char(goal_char)}')
                        ws.send(get_msg_from_goal_char(goal_char))
                    else:
                        print('Error: unknown char recieved')
                    #Give Sign of Life
                else:
                    if(curent_matchState != last_matchState):
                        print('MatchState Changed')

                        if (curent_matchState in matchState_char_msg_map):
                            print(f'Info: sent to Arduino {get_msg_from_MatchStat_char(curent_matchState)}')
                            connection.write(bytes(get_msg_from_MatchStat_char(curent_matchState), 'utf-8'))
                        else:
                            print('MatchState Error')
                        last_matchState = curent_matchState
            

        thread.start_new_thread(run, ())
    
    return on_ws_open

def on_message(ws, message):
    global curent_matchState
    global last_matchState
    # and returns dict.
    data = json.loads(message)

    print("JSON string = ", data)
    print()

    if(data['type'] == 'ping'):
        print('is ping')
        print("Curent MatchState: %s" % (curent_matchState))
        print("Last MatchState: %s" % (last_matchState))
        last_matchState = '9'

    if(data['type'] == 'matchTime'):
        print('is matchTime')
        curent_matchState = str(data['data']['MatchState'])
        print(curent_matchState)

    if(data['type'] == 'realtimeScore'):
        print('is realtimeScore')
        curent_matchState = str(data['data']['MatchState'])
        print("Curent MatchState: %s" % (curent_matchState))

def open_websocket(serial_connection):
    def reopen_websocket():
        open_websocket(serial_connection)

    res = requests.post(f'http://{FMS_SERVER}/login'
        , data={'username': USERNAME, 'password': PASSWORD}
        , allow_redirects=False
    )

    ws = websocket.WebSocketApp(f'ws://{FMS_SERVER}/panels/scoring/{ALLIANCE_COLOR}/websocket'
        , on_open=get_on_ws_open_callback(serial_connection)
        , on_message=on_message
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
