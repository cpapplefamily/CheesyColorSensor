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


serial_Msg = 0b00000000

updateArduino = False
current_coopActivated = False
curent_bankedAmpNotes = 0
current_amplifiedTimePostWindow = False
curent_matchState = '9'
en_Serial_Print = False
current_amplifiedTimeRemainingSec = 0
       
def get_variables_from_file(file_path):
    variables = {}
    try:
        with open(file_path, 'r') as file:
            lines = file.readlines()
            for line in lines:
                if ':' in line:
                    key, value = line.split(':', 1)
                    variables[key.strip()] = value.strip()
    except FileNotFoundError:
        print(f"The file {file_path} does not exist.")
    except Exception as e:
        print(f"An error occurred: {e}")
    return variables

goal_char_msg_map = {
    "S": '{ "type": "S" }',
    "A": '{ "type": "A" }',
    "K": '{ "type": "amplifyButton" }',
    "C": '{ "type": "coopButton" }'
}

# Return the first arduino mega connected to PC
def find_arduino_port(): 
    for port in serial.tools.list_ports.comports():
        print('PortID: ' + port.hwid)
        #fe201000.serial
        #if 'fe201000.serial' in port.hwid: 
        #if 'VID:PID=10C4:EA60' in port.hwid:   
        return '/dev/ttyS0'
        
# Return char recieved from arduino
def get_serial_char(usb_connection):
    #connection.reset_input_buffer() // Removed for none Blocking
    if (usb_connection.inWaiting() > 0):
        _char = usb_connection.read(1).decode('UTF-8')
    else:
        _char = ""
    return _char

# Return char recieved from arduino and convert it to the WebSocket Value Cheesy needs to score the action
def get_msg_from_goal_char(goal_char):
    return goal_char_msg_map[goal_char]


#create a Timer class that can be used to periodicaly send messages
class NonBlockingTimer:
    def __init__(self, interval):
        self.interval = interval
        self.start_time = None

    def start(self):
        self.start_time = time.time()

    def check(self):
        if self.start_time is None:
            return False
        return time.time() - self.start_time >= self.interval
    
    def reset(self):
        self.start_time = time.time()
    
# paralle Thread that connect us to the Arena via Websockets
def get_on_ws_open_callback(usb_connection):
    def on_ws_open(ws):
        print("Connected to FMS")

        def run(*args):
            global updateArduino
            message_Interval = NonBlockingTimer(.5)  # .5-second timer
            message_Interval.start()
            # Watch for Serial Data and translate for the Websocket Message
            while(True):
                goal_char = get_serial_char(usb_connection)
                if(goal_char != ""):
                    print(f'Info: recieved "{goal_char}"')
                    if (goal_char in goal_char_msg_map):
                        print(f'Info: sent {get_msg_from_goal_char(goal_char)}')
                        ws.send(get_msg_from_goal_char(goal_char))
                    else:
                        print('Error: unknown char recieved')
                else:
                    if message_Interval.check():
                        #if(updateArduino):
                        #print('update Arduino')
                        message = str(serial_Msg) + '\n'
                        print(f'Info: sent to Arduino {serial_Msg}')
                        #usb_connection.write(bytes(message, 'utf-8'))
                        usb_connection.write((str(serial_Msg) + '\n').encode())
                        message_Interval.reset()
                        updateArduino = False
                time.sleep(0.01)  # Sleep for a short while to prevent CPU overload

        thread.start_new_thread(run, ())
    
    return on_ws_open
                
def on_message(ws, message):
    global ALLIANCE_COLOR 
    global USERNAME 
    global PASSWORD 
    global curent_matchState
    global en_Serial_Print
    global current_coopActivated
    global curent_bankedAmpNotes
    global current_amplifiedTimePostWindow
    global current_amplifiedTimeRemainingSec
    global updateArduino
    global serial_Msg
    
    # Clear The Message for the Amp
    serial_Msg = 0b00000000

    # and returns dict.
    data = json.loads(message)
    if(en_Serial_Print and False):
        print("JSON string = ", data)
        print()

    if(data['type'] == 'ping'):
        print('is ping')
        if(en_Serial_Print):
            #print("JSON string = ", data)
            print("Curent MatchState: %s" % (curent_matchState))
            print("Curent BankedAmpNotes: %s" % (curent_bankedAmpNotes))
            print("Curent CoopActivated: %s" % (current_coopActivated))

    if(data['type'] == 'matchTime'):
        curent_matchState = data['data']['MatchState']
        if(en_Serial_Print and False):
            print("JSON string = ", data)
            print('is matchTime')
            print("Curent MatchState: %s" % (curent_matchState))
        

    if(data['type'] == 'realtimeScore'):
        curent_matchState = data['data']['MatchState']
        allianceScore = data['data'][ALLIANCE_COLOR]['Score']
        curent_bankedAmpNotes = allianceScore["AmpSpeaker"]["BankedAmpNotes"]
        current_coopActivated = allianceScore["AmpSpeaker"]["CoopActivated"]
        current_amplifiedTimePostWindow = data['data'][ALLIANCE_COLOR]["AmplifiedTimePostWindow"]
        current_amplifiedTimeRemainingSec = data['data'][ALLIANCE_COLOR]["AmplifiedTimeRemainingSec"]
        if(en_Serial_Print):
            print('is realtimeScore')
            print("Curent MatchState: %s" % (curent_matchState))
            print("BankedAmpNotes = ", curent_bankedAmpNotes)
            print("CoopertitionStatus = ", current_coopActivated)
            print("AmplifiedTimePostWindow = ", current_amplifiedTimePostWindow)
            print("Amplification Sec Remaining = ",current_amplifiedTimeRemainingSec)
    
    #fill in the message
    if(curent_matchState < 6):
        serial_Msg = serial_Msg | int(curent_bankedAmpNotes)
        if(current_amplifiedTimeRemainingSec > 0):
            serial_Msg = serial_Msg | 0b00000100
        if(current_coopActivated):
            serial_Msg = serial_Msg | 0b00001000
    shifted_curent_matchState = (curent_matchState << 4) & 0xF0
    serial_Msg = serial_Msg | shifted_curent_matchState
    updateArduino = True

# Production use for opening a Websocket
def open_websocket(serial_connection):
    global ALLIANCE_COLOR 
    global USERNAME 
    global PASSWORD 

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
    global ALLIANCE_COLOR 
    global USERNAME 
    global PASSWORD 

    #load configs
    file_path = 'src/Python/config.txt'
    variables_dict = get_variables_from_file(file_path)
    ALLIANCE_COLOR = variables_dict.get('ALLIANCE_COLOR')
    USERNAME = variables_dict.get('USERNAME')
    PASSWORD = variables_dict.get('PASSWORD')

    """Loops until a connection is made to the Arduino.
    When connected advances to check for Area
    """
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
    open_websocket(usb_connection)

main()
