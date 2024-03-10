import os
import serial
import serial.tools.list_ports
import requests
import time
import _thread as thread
import websocket
import json

#FMS_IP = "10.0.100.05"
FMS_IP = "192.168.1.187"
FMS_PORT = "8080"
FMS_SERVER = FMS_IP + ":" + FMS_PORT
ALLIANCE_COLOR = 'Red' # Change accordingly
#ALLIANCE_COLOR = 'Blue' # Change accordingly
USERNAME = 'admin'
PASSWORD = 'ProliantDL160'

cooperationStatus = False
amplificationCount = 0
amplificationStatus = False
ampAccumulatorDisable = False
current_matchState = '9'
last_matchState = '0'
current_ampCount = '0'
last_ampCount = '99'
current_coopStatus = '0'
last_coopStatus = '99'
current_speakerState = '0'
last_speakerState = '99'
en_Serial_Print = False


goal_char_msg_map = {
    "W": '{ "type": "W" }',
    "R": '{ "type": "R" }',
    "P": '{ "type": "amplificationActive" }',
    "O": '{ "type": "coopertitionStatus" }'
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

ampCount_char_msg_map = {
    "0": '30',
    "1": '31',
    "2": '32'
}

speakerState_char_msg_map = {
    "False": '40',
    "True": '41',
    "Delay": '42'
}

coopState_char_msg_map = {
    "False": '50',
    "True": '51'
}

# Return the first arduino mega connected to PC
def find_arduino_port(): 
    for port in serial.tools.list_ports.comports():
        print('PortID: ' + port.hwid)
        #fe201000.serial
        #if 'fe201000.serial' in port.hwid: 
        #if 'VID:PID=10C4:EA60' in port.hwid:   
        #    return port.device
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

def get_msg_from_MatchState_char(MatchState_char):
    return matchState_char_msg_map[MatchState_char]

def get_msg_from_AmpCount_char(AmpStat_char):
    return ampCount_char_msg_map[AmpStat_char]

def get_msg_from_CoopState_char(CoopState_char):
    return coopState_char_msg_map[CoopState_char]

def get_msg_from_SpeakerState_char(SpeakerState_char):
    return speakerState_char_msg_map[SpeakerState_char]

def int_to_bytes(x: int) -> bytes:
    return x.to_bytes((x.bit_length() + 7) // 8, 'big')

def get_on_ws_open_callback(usb_connection):
    def on_ws_open(ws):
        print("Connected to FMS")

        def run(*args):
            global current_matchState
            global last_matchState
            global current_ampCount
            global last_ampCount
            global current_coopStatus
            global last_coopStatus
            global current_speakerState
            global last_speakerState
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
                    #Send Match state changes to Arduino
                    if(current_matchState != last_matchState):
                        print('MatchState Changed')
                        if (current_matchState in matchState_char_msg_map):
                            print(f'Info: sent to Arduino {get_msg_from_MatchState_char(current_matchState)}')
                            usb_connection.write(bytes(get_msg_from_MatchState_char(current_matchState)+"\n", 'utf-8'))
                        else:
                            print('MatchState Error')
                        last_matchState = current_matchState

                    #Send Amp Count changes to Arduino
                    elif(current_ampCount != last_ampCount):
                        print('Amp Count Changed')
                        if (current_ampCount in ampCount_char_msg_map):
                            print(f'Info: sent to Arduino {get_msg_from_AmpCount_char(current_ampCount)}')
                            usb_connection.write(bytes(get_msg_from_AmpCount_char(current_ampCount)+"\n", 'utf-8'))
                        else:
                            print('AmpCount Error')
                        last_ampCount = current_ampCount
                    elif(current_coopStatus != last_coopStatus):
                        print('Coop Status Changed')
                        if (current_coopStatus in coopState_char_msg_map):
                            print(f'Info: sent to Arduino {get_msg_from_CoopState_char(current_coopStatus)}')
                            usb_connection.write(bytes(get_msg_from_CoopState_char(current_coopStatus)+"\n", 'utf-8'))
                        else:
                            print('coOpStatus Error')
                        last_coopStatus = current_coopStatus
                    elif(current_speakerState != last_speakerState):
                        print('speaker Status Changed')
                        if (current_speakerState in speakerState_char_msg_map):
                            print(f'Info: sent to Arduino {get_msg_from_SpeakerState_char(current_speakerState)}')
                            usb_connection.write(bytes(get_msg_from_SpeakerState_char(current_speakerState)+"\n", 'utf-8'))
                        else:
                            print('speakerState Error')
                        last_speakerState = current_speakerState
            

        thread.start_new_thread(run, ())
    
    return on_ws_open
                
def on_message(ws, message):
    global current_matchState
    global last_matchState
    global current_ampCount
    global last_ampCount
    global current_coopStatus
    global last_coopStatus
    global current_speakerState
    global last_speakerState
    global en_Serial_Print
    global cooperationStatus
    global amplificationStatus
    global ampAccumulatorDisable
    
    
    # and returns dict.
    data = json.loads(message)
    if(en_Serial_Print):
        print("JSON string = ", data)
        print()

    if(data['type'] == 'ping'):
        if(en_Serial_Print):
            print('is ping')
            print("current MatchState: %s" % (current_matchState))
            print("Last MatchState: %s" % (last_matchState))
            print("Amp Count = ", amplificationCount)
        last_matchState = '9' #force a refresh
        last_ampCount = '99' #force a refresh

    if(data['type'] == 'matchTime'):
        current_matchState = str(data['data']['MatchState'])
        amplificationSecRemaining = data['data']['RedAmplificationRemaining']
        if(en_Serial_Print):
            print('is matchTime')
            print(current_matchState)
            print("Amplification Sec Remaining = ",amplificationSecRemaining)
        else:
            print("Amplification Sec Remaining = ",amplificationSecRemaining)
        

    if(data['type'] == 'realtimeScore'):
        current_matchState = str(data['data']['MatchState'])
        p1 = data['data'][ALLIANCE_COLOR]['Score']
        current_ampCount = str(p1["AmplificationCount"])
        cooperationStatus = p1["CoopertitionStatus"]
        current_coopStatus = str(p1["CoopertitionStatus"])
        amplificationStatus = p1["AmplificationActive"]
        current_speakerState = str(p1["AmplificationActive"])
        ampAccumulatorDisable = p1["AmpAccumulatorDisable"]
        amplificationSecRemaining = p1["AmplificationSecRemaining"]
        #print("p1 = ", p1)
        if(en_Serial_Print):
            print('is realtimeScore')
            print("current MatchState: %s" % (current_matchState))
            print("current Amp Count = ", current_ampCount)
            print("CoopertitionStatus = ", cooperationStatus)
            print("Amp Status = ", amplificationStatus)
            print("Amp Accumulator Disabled = ",ampAccumulatorDisable)
            print("Amplification Sec Remaining = ",amplificationSecRemaining)
        else:
            print("current Amp Count = ", current_ampCount)
            print("CoopertitionStatus = ", cooperationStatus)
            print("Amp Status = ", amplificationStatus)
            

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

def main():
    
    """Loops until a connection is made to the Arduino.
    When connected advances to check for Area
    """
    while(True):
        print('Find arduino')
        usb_connection = serial.Serial(find_arduino_port(), 2400)
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
        
    open_websocket(usb_connection)

main()
