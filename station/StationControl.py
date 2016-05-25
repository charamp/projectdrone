import socket
import time

list_of_active_drone = []
number_of_all_drone = 3
list_of_all_drone = ["192.168.8.11", "192.168.8.12", "192.168.8.13"]

def checkActiveDrone():
    global list_of_active_drone
    global number_of_all_drone
    backup = list_of_active_drone
    list_of_active_drone = []
    for x in list_of_all_drone:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            result = s.connect_ex((x, 9999))
            if result == 0:
                list_of_active_drone.append(x)
        except socket.error as msg:
            continue

    number_of_all_drone = len(list_of_active_drone)

    if len(backup) == len(list_of_active_drone):
        return False # not change
    else:
        return True # change

def sendCommand(ip, message):
    port = 9999
    buffer_size = 1024
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        s.connect((ip,port))
    except socket.error as msg:
        print "Request timeout."
        time.sleep(2)
    try:
        s.send("root "+message)
        print "OK"
        s.close()
    except socket.error as msg:
        time.sleep(2)

print "\n\t ======================================"
print "\t|                                      |"
print "\t|            Station Control           |"
print "\t|                                      |"
print "\t ======================================"
print "\t type \"help\" to show list of command\n"

while 1:
    print "INPUT >> ",
    inp = raw_input()
    if inp == "": 
        continue
    if inp.split(" ")[0] == "exit":
        exit()
    if inp.split(" ")[0] == "help":
        print "\t ---------- Manual ----------\n"
        print "\t set d [ip]"
        print "\t\t -set initial designated drone"
        print "\t set home [ip] [latitude] [longitude]"
        print "\t\t -set home location of drone"
        print "\t set station [latitude] [longitude]"
        print "\t\t -set station location"
        print "\t set node [latitude] [longitude]"
        print "\t\t -set node location"
        print "\t help"
        print "\t\t -show list of command"
        continue

    checkActiveDrone()

    if inp.split(" ")[1] == "d" or inp.split(" ")[1] == "home":
        if (inp.split(" ")[2] not in list_of_active_drone):
            print inp.split(" ")[2]+" offline"
        else:
            sendCommand(inp.split(" ")[2], inp.lower())
    else:
        for ip in list_of_active_drone:
            sendCommand(ip, inp.lower())
            time.sleep(1)
