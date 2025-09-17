import sys
sys.path.append("/usr/lib/ultraleap-hand-tracking-service")
#sys.path.append(r"D:\Ultraleap\LeapSDK")
#import cffi
from leapc_cffi import _leapc_cffi
import threading
import socket
import json

ffi = _leapc_cffi.ffi
lib = _leapc_cffi.lib

connectionHandle = ffi.new("LEAP_CONNECTION *")

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
target = ("127.0.0.1", 5005)  # target address and port

_isRunning = False
_loop_thread = None

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

selected_hand_id = None
number_of_hands = 0

def OnFrame(frame):
    global selected_hand_id, number_of_hands
    if (number_of_hands != frame.nHands):
        print("tracked hands:", frame.nHands)
        number_of_hands = frame.nHands

    # Sende nur jedes zweite Frame / frame rate limit
    # if frame.info.frame_id % 2 != 0:
    #     return

    if frame.nHands == 0:
        selected_hand_id = None
        return

    hands = [frame.pHands[i] for i in range(frame.nHands)]

    hand = None

    if selected_hand_id is not None:
        for h in hands:
            if h.id == selected_hand_id:
                hand = h
                break

    if hand is None:
        hand = hands[0]

    if selected_hand_id is None:
        selected_hand_id = hand.id

    if selected_hand_id != hand.id:
        selected_hand_id = hand.id

    data = {
        # "id": hand.id,
        # "type": "left" if hand.type == lib.eLeapHandType_Left else "right",
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

def startTracking(target_adress = None):
    """Start the hand tracking service and send data to the specified target address."""
    global _isRunning, _loop_thread, target
    if target_adress is not None:
        target = target_adress
    if _isRunning:
        print("Tracking is already running.")
        return
    OpenConnection()
    print("OpenConnection successful:", lib.eLeapRS_Success, "Connection running:", _isRunning)

    _loop_thread = threading.Thread(target=serviceMessageLoop)
    _loop_thread.start()
    print("Hand tracking started.")

def stopTracking(join=True):
    """Stoppt Handtracking und schließt Ressourcen."""
    global _loop_thread
    DestroyConnection()
    t = _loop_thread
    _loop_thread = None
    if join and t is not None:
        t.join(timeout=2.0)
        print("Hand tracking thread stopped.")

def main():
    startTracking()
    input("Drücke Enter zum Beenden...\n")
    stopTracking()
    return 0

if __name__ == "__main__":
    main()
