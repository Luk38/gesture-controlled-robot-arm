import sys
#sys.path.append("/usr/lib/ultraleap-hand-tracking-service")
sys.path.append(r"D:\Ultraleap\LeapSDK")
#import cffi
from leapc_cffi import _leapc_cffi
import threading
import socket
import json

ffi = _leapc_cffi.ffi
lib = _leapc_cffi.lib

connectionHandle = ffi.new("LEAP_CONNECTION *")

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
target = ("127.0.0.1", 4999)  # target address and port

def OpenConnection():
    global _isRunning
    if(_isRunning):
        return connectionHandle
    if(connectionHandle[0] or lib.LeapCreateConnection(ffi.NULL, connectionHandle) == lib.eLeapRS_Success):
        result = lib.LeapOpenConnection(connectionHandle[0])
        print("OpenConnection result:", result)
        if(result == lib.eLeapRS_Success):
            _isRunning = True
    return connectionHandle

def CloseConnection():
    global _isRunning
    if not _isRunning:
        return
    _isRunning = False
    lib.LeapCloseConnection(connectionHandle[0])

def DestroyConnection():
    CloseConnection()
    lib.LeapDestroyConnection(connectionHandle[0])
    sock.close()


IsConnected = False

def OnConnect():
    print("Connected")

def OnConnectionLost():
    print("Connection lost")

def OnFrame(frame):
    if (frame.info.frame_id % 60 == 0):
        print("Frame: ", frame.info.frame_id, "with ", frame.nHands)
    # Sende nur jedes zweite Frame (bei 120Hz → 60Hz)
    # if frame.info.frame_id % 2 != 0:
    #     return
    for h in range(frame.nHands):
        hand = frame.pHands[h]
        data = {
            "id": hand.id,
            "type": "left" if hand.type == lib.eLeapHandType_Left else "right",
            "x": hand.palm.position.x,
            "y": hand.palm.position.y,
            "z": hand.palm.position.z,
            # "velocity_x": hand.palm.velocity.x, #millimeters per second
            # "velocity_y": hand.palm.velocity.y,
            # "velocity_z": hand.palm.velocity.z,
            "grab_strength": hand.grab_strength,
            "pinch_strength": hand.pinch_strength,
            "orientation": {
                "x": hand.palm.orientation.x,
                "y": hand.palm.orientation.y,
                "z": hand.palm.orientation.z,
                "w": hand.palm.orientation.w
            }
            
        }
        sock.sendto(json.dumps(data).encode(), target)

def handleConnectionEvent(connection_event):
    global IsConnected
    IsConnected = True
    OnConnect()

def handleConnectionLostEvent(connection_lost_event):
    global IsConnected
    IsConnected = False
    OnConnectionLost()

def handleTrackingEvent(tracking_event):
    OnFrame(tracking_event)

def serviceMessageLoop():
    global _isRunning
    while _isRunning:
        timeout = 1000
        msg = ffi.new("LEAP_CONNECTION_MESSAGE *")
        result = lib.LeapPollConnection(connectionHandle[0], timeout, msg)

        if result != lib.eLeapRS_Success:
            continue
        
        if(msg.type == lib.eLeapEventType_Connection):
            handleConnectionEvent(msg.connection_event)          
        elif(msg.type == lib.eLeapEventType_ConnectionLost):
            handleConnectionLostEvent(msg.connection_lost_event)
        elif(msg.type == lib.eLeapEventType_Tracking):
            handleTrackingEvent(msg.tracking_event)
        else:
            print("Unknown message type", msg.type)

def main():
    global _isRunning
    _isRunning = False
    OpenConnection()
    print("OpenConnection successful:", lib.eLeapRS_Success, "Connection running:", _isRunning)

    loop_thread = threading.Thread(target=serviceMessageLoop)
    loop_thread.start()

    input("Drücke Enter zum Beenden...\n")
    DestroyConnection()
    print("Connection destroyed, running:", _isRunning)
    loop_thread.join()

    return 0

if __name__ == "__main__":
    main()

# import time

# _last_send = 0
# _SEND_INTERVAL = 1.0 / 60  # 60Hz

# def OnFrame(frame):
#     global _last_send
#     now = time.time()
#     if now - _last_send < _SEND_INTERVAL:
#         return
#     _last_send = now
#     for h in range(frame.nHands):
#         hand = frame.pHands[h]
#         data = {
#             # ... wie gehabt ...
#         }
#         sock.sendto(json.dumps(data).encode(), target)