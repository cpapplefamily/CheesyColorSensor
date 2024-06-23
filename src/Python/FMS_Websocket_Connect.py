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
#ALLIANCE_COLOR = 'Red' # Change accordingly
ALLIANCE_COLOR = 'Blue' # Change accordingly
USERNAME = 'admin'
PASSWORD = 'ProliantDL160'

isWindows = False #Set when Testing with out RaspberryPi
usingArduino = True # When I Don't have an Arduino Connected
updateArduino = False
CoopActivated = False
curent_bankedAmpNotes = 0
last_bankedAmpNotes = 2 #this forces a message
AmplifiedTimePostWindow = False
curent_matchState = '9'
last_matchState = '0'
en_Serial_Print = True
AmplifiedTimeRemainingSec = 0
       

goal_char_msg_map = {
    "S": '{ "type": "S" }',
    "A": '{ "type": "A" }',
    "K": '{ "type": "amplifyButton" }',
    "C": '{ "type": "coopButton" }'
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

bankedAmpNotes_char_msg_map = {
    "0": '30',
    "1": '31',
    "2": '32'
}

speakerState_char_msg_map = {
    "0": '40',
    "1": '41',
    "2": '42'
}

coopState_char_msg_map = {
    "0": '50',
    "1": '51'
}

# Return the first arduino mega connected to PC
def find_arduino_port(): 
    global isWindows
    for port in serial.tools.list_ports.comports():
        print('PortID: ' + port.hwid)
        #fe201000.serial
        #if 'fe201000.serial' in port.hwid: 
        #if 'VID:PID=10C4:EA60' in port.hwid:   
        if(isWindows):
            return port.device
        else:
            return '/dev/ttyS0'
        
# Retur char recieved from arduino
def get_serial_char(usb_connection):
    #connection.reset_input_buffer() // Removed for none Blocking
    if (usb_connection.inWaiting() > 0):
        _char = usb_connection.read(1).decode('UTF-8')
    else:
        _char = ""
    return _char

def get_msg_from_goal_char(goal_char):
    return goal_char_msg_map[goal_char]

def get_msg_from_MatchState_char(MatchStat_char):
    return matchState_char_msg_map[MatchStat_char]

def get_msg_from_BankedAmpNotes_char(BankedAmpNotes_char):
    return bankedAmpNotes_char_msg_map[BankedAmpNotes_char]

def int_to_bytes(x: int) -> bytes:
    return x.to_bytes((x.bit_length() + 7) // 8, 'big')

def get_on_ws_open_callback(usb_connection):
    def on_ws_open(ws):
        print("Connected to FMS")

        def run(*args):
            global curent_matchState
            global last_matchState
            global CoopActivated
            global curent_bankedAmpNotes
            global last_bankedAmpNotes
            global AmplifiedTimePostWindow
            global AmplifiedTimeRemainingSec
            global updateArduino

            while(True):
                goal_char = get_serial_char(usb_connection)
                if(goal_char != ""):
                    print(f'Info: recieved "{goal_char}"')

                    if (goal_char in goal_char_msg_map):
                        print(f'Info: sent {get_msg_from_goal_char(goal_char)}')
                        ws.send(get_msg_from_goal_char(goal_char))
                    else:
                        print('Error: unknown char recieved')
                    #Give Sign of Life
                else:
                    if(updateArduino):
                        #print('update Arduino')
                        updateArduino = False
                    #priority is match state
                    if(curent_matchState != last_matchState):
                        print('MatchState Changed')
                        if (str(curent_matchState) in matchState_char_msg_map):
                            print(f'Info: sent to Arduino {get_msg_from_MatchState_char(str(curent_matchState))}')
                            message = get_msg_from_MatchState_char(str(curent_matchState)) + '\n'
                            usb_connection.write(bytes(message, 'utf-8'))
                        else:
                            print('MatchState Error')
                        last_matchState = curent_matchState
                    #next priority
                    elif(curent_bankedAmpNotes != last_bankedAmpNotes):
                        print('bankedAmpNotes Changed')
                        if (str(curent_bankedAmpNotes) in bankedAmpNotes_char_msg_map):
                            print(f'Info: sent to Arduino {get_msg_from_BankedAmpNotes_char(str(curent_bankedAmpNotes))}')
                            message = get_msg_from_BankedAmpNotes_char(str(curent_bankedAmpNotes)) + '\n'
                            usb_connection.write(bytes(message, 'utf-8'))
                        else:
                            print('bankedAmpNotes Error')
                        last_bankedAmpNotes = curent_bankedAmpNotes
            

        thread.start_new_thread(run, ())
    
    return on_ws_open
                
def on_message(ws, message):
    global curent_matchState
    global last_matchState
    global en_Serial_Print
    global CoopActivated
    global curent_bankedAmpNotes
    global AmplifiedTimePostWindow
    global AmplifiedTimeRemainingSec
    global updateArduino
    
    
    # and returns dict.
    data = json.loads(message)
    if(en_Serial_Print and False):
        print("JSON string = ", data)
        print()

    if(data['type'] == 'ping'):
        if(en_Serial_Print):
            #print("JSON string = ", data)
            print('is ping')
            print("Curent MatchState: %s" % (curent_matchState))
            print("Last MatchState: %s" % (last_matchState))
            print("Curent BankedAmpNotes: %s" % (curent_bankedAmpNotes))
            print("Last BankedAmpNotes: %s" % (last_bankedAmpNotes))
        updateArduino = True

    if(data['type'] == 'matchTime'):
        curent_matchState = data['data']['MatchState']
        updateArduino = True
        if(en_Serial_Print and False):
            print("JSON string = ", data)
            print('is matchTime')
            print("Curent MatchState: %s" % (curent_matchState))
        

    if(data['type'] == 'realtimeScore'):
        #print("JSON string = ", data)
        print()
        curent_matchState = data['data']['MatchState']
        allianceScore = data['data'][ALLIANCE_COLOR]['Score']
        curent_bankedAmpNotes = allianceScore["AmpSpeaker"]["BankedAmpNotes"]
        CoopActivated = allianceScore["AmpSpeaker"]["CoopActivated"]
        AmplifiedTimePostWindow = data['data'][ALLIANCE_COLOR]["AmplifiedTimePostWindow"]
        AmplifiedTimeRemainingSec = data['data'][ALLIANCE_COLOR]["AmplifiedTimeRemainingSec"]
        #print("allianceScore = ", allianceScore)
        #if(en_Serial_Print):
        updateArduino = True
        if(en_Serial_Print):
            print('is realtimeScore')
            print("Curent MatchState: %s" % (curent_matchState))
            #print("allianceScore: %s" % (allianceScore))
            print("BankedAmpNotes = ", curent_BankedAmpNotes)
            print("CoopertitionStatus = ", CoopActivated)
            print("AmplifiedTimePostWindow = ", AmplifiedTimePostWindow)
            print("Amplification Sec Remaining = ",AmplifiedTimeRemainingSec)

def open_websocket(serial_connection):
    def reopen_websocket():
        open_websocket(serial_connection)

    res = requests.post(f'http://{FMS_SERVER}/login'
        , data={'username': USERNAME, 'password': PASSWORD}
        , allow_redirects=False
    )

    ws = websocket.WebSocketApp(f'ws://{FMS_SERVER}/panels/scoring/{ALLIANCE_COLOR.lower()}/websocket'
        , on_open=get_on_ws_open_callback(serial_connection)
        , on_message=on_message
        , on_close=reopen_websocket
        , cookie="; ".join(["%s=%s" %(i, j) for i, j in res.cookies.get_dict().items()])
    )

    ws.run_forever()

def open_websocket2():
    def reopen_websocket():
        open_websocket2()

    res = requests.post(f'http://{FMS_SERVER}/login'
        , data={'username': USERNAME, 'password': PASSWORD}
        , allow_redirects=False
    )

    ws = websocket.WebSocketApp(f'ws://{FMS_SERVER}/panels/scoring/{ALLIANCE_COLOR.lower()}/websocket'
        #, on_open=get_on_ws_open_callback(serial_connection)
        , on_message=on_message
        , on_close=reopen_websocket
        , cookie="; ".join(["%s=%s" %(i, j) for i, j in res.cookies.get_dict().items()])
    )

    ws.run_forever()

def main():
    global usingArduino
    """Loops until a connection is made to the Arduino.
    When connected advances to check for Area
    """
    if (usingArduino):
        while(True):
            print('Find arduino')
            usb_connection = serial.Serial(find_arduino_port(), 9600)
            #usb_connection = serial.Serial("/dev/ttyS0", 9600)

            if (usb_connection.is_open):
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
    
    #Open Websocket
    if(usingArduino):
        open_websocket(usb_connection)
    else:
        open_websocket2()

main()
