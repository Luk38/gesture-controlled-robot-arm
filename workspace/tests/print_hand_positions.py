import socket
import json

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("127.0.0.1", 5005))

while True:
    data, addr = sock.recvfrom(1024)
    hand_data = json.loads(data.decode())

    # Palm position
    print(
             "    Hand id {} is a {} hand with position ({:.2f}, {:.2f}, {:.2f})".format(
                 hand_data["id"],
                 hand_data["type"],
                 hand_data["x"],
                 hand_data["y"],
                 hand_data["z"]
             )
        )
    
    # # Palm orientation
    # print(
    #          "    Hand id {} orientation is ({:.2f}, {:.2f}, {:.2f}, {:.2f})".format(
    #              hand_data["id"],
    #              hand_data["orientation"]["x"],
    #              hand_data["orientation"]["y"],
    #              hand_data["orientation"]["z"],
    #              hand_data["orientation"]["w"]
    #          )
    #     )
    #print(hand_data['orientation'])

    # Palm velocity
    # print(
    #         " Hand id {} is moving {:.2f} mm/s in x direction," \
    #         " {:.2f} mm/s in y direction" \
    #         " and {:.2f} mm/s in z direction".format(
    #             hand_data["id"],
    #             hand_data["velocity_x"],
    #             hand_data["velocity_y"],
    #             hand_data["velocity_z"]
    #         )
    # )

    # Pinch strength
    # print(" Hand id {} pinch {:.2f}".format(
    #     hand_data["id"],
    #     hand_data["pinch_strength"]
    # ))

    # Grab strength
    # print(" Hand id {} grab {:.2f}".format(
    #     hand_data["id"],
    #     hand_data["grab_strength"]
    # ))