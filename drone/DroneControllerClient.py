from getDistance import *
import serial, time
import math
import socket
import itertools
import os
import fcntl
import struct

class coordinate(object):
    def __init__(self,lat,lon):
        self.lat = lat
        self.lon = lon

def getMyIP(ifname):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    return socket.inet_ntoa(fcntl.ioctl(
        s.fileno(),
        0x8915,
        struct.pack('256s', ifname[:15])
    )[20:24])

list_of_ip = ["192.168.8.11", "192.168.8.12", "192.168.8.13"]
status = "W"
designated_drone = "192.168.8.11"
max_number_of_all_drone = 3
number_of_all_drone = 3
list_of_active_drone = []
my_ip = getMyIP("wlan0") #"192.168.8.11"
other_ip = [ip for ip in list_of_ip if ip != my_ip] #["192.168.8.12", "192.168.8.13"]
station_point = coordinate(13.846324, 100.567100)
node_point = coordinate(13.846213, 100.567004)
update_times = 0
state_update = True
first_ND = True
comeback_home = False
comeback_time = time.time()

def calculateWP():
    list_of_waypoint = []
    for i in xrange(int(number_of_drone_for_create_connection)):
        averate_lat = ( (station_point.lat - node_point.lat)/(number_of_drone_for_create_connection+1)*(i+1) ) + min([station_point.lat, node_point.lat])
        averate_lon = ( (station_point.lon - node_point.lon)/(number_of_drone_for_create_connection+1)*(i+1) ) + min([station_point.lon, node_point.lon])
        list_of_waypoint.append([round(averate_lat,6), round(averate_lon,6)])
    for i in xrange(int(number_of_all_drone - number_of_drone_for_create_connection)):
        #arctan = math.atan((node_point.lon - station_point.lon)/(node_point.lat - station_point.lat))
        arctan = math.atan((node_point.lat - station_point.lat)/(node_point.lon - station_point.lon))
        middle_lat = (station_point.lat + node_point.lat)/2
        middle_lon = (station_point.lon + node_point.lon)/2
        plus_lat = -0.00005 * (0.25*((-2)*(math.pow(-1,i+1))*(i+1) - (math.pow(-1,i+1)) + 1)) * math.cos(math.pi - arctan)
        plus_lon = -0.00005 * (0.25*((-2)*(math.pow(-1,i+1))*(i+1) - (math.pow(-1,i+1)) + 1)) * math.sin(math.pi - arctan)
        list_of_waypoint.append([round(middle_lat+plus_lat,6), round(middle_lon+plus_lon,6)])
    return list_of_waypoint

def combinationDroneAndWaypoint(list_of_waypoint):
    my_gps = readGPS()
    list_of_gps_drone = readGPSFromOtherDrone()+[[my_ip,str(my_gps.lat),str(my_gps.lon),str(update_times)]]
    for ip,lat,lon,update in list_of_gps_drone:
        print "[MESSAGE] LIST_OF_GPS_DRONE : "+ip+" => LAT : "+lat+" LONG : "+lon
    min_of_max_distance = float("inf")

    maching_all = [zip(x,list_of_gps_drone) for x in itertools.permutations(list_of_waypoint,len(list_of_gps_drone))]
    maching_all_add_distance = []
    for match in maching_all:
        combination = []
        max_distance = -float("inf")
        this_match_distance = []
        for wp,dp in match:
            wlat,wlon = wp
            ip,dlat,dlon,update = dp
            distance = math.sqrt( math.pow((float(wlat) - float(dlat)),2) + math.pow((float(wlon) - float(dlon)),2) )
            this_match_distance.append(distance)
        maching_all_add_distance.append([match,sorted(this_match_distance, reverse=True)])

    if len(maching_all_add_distance[0][1]) == 1:
        maching_all_add_distance.sort(key=lambda x: (x[1][0]))
    elif len(maching_all_add_distance[0][1]) == 2:
        maching_all_add_distance.sort(key=lambda x: (x[1][0], x[1][1]))
    elif len(maching_all_add_distance[0][1]) == 3:
        maching_all_add_distance.sort(key=lambda x: (x[1][0], x[1][1], x[1][2]))
    
    result = []
    point,value = maching_all_add_distance[0]
    for wp, dp in point:
        wlat, wlon = wp
        ip, dlat, dlon, update = dp
        result.append([ip, wlat, wlon])

    return result

def genaratePacket(header, message):
    return header+" "+message

def checkActiveDrone():
    global list_of_active_drone
    global number_of_all_drone
    backup = list_of_active_drone
    list_of_active_drone = []
    for x in other_ip:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            result = s.connect_ex((x, 9999))
            if result == 0:
                list_of_active_drone.append(x)
        except socket.error as msg:
            continue

    number_of_all_drone = len(list_of_active_drone+[my_ip])

    if len(backup) == len(list_of_active_drone):
        return False # not change
    else:
        return True # change

def checkStatus():
	status = open("status.txt", "r")
	return status.read().split(" ")

def readGPSFromOtherDrone():
    Dlist = []
    for x in list_of_active_drone:
        f = open('gps_'+str(x)+'.txt', 'r')
        Dlist.append(f.read().split(","))
    return Dlist

def readGPS():
    check = open("command.txt", "r")
    """
    while check.read() != "":
        print "command.txt not empty !"
        check.close()
        time.sleep(1)
        check = open("command.txt", "r")
    """
    command = open("command.txt", "w")
    command.write("status")
    command.close()

    time.sleep(1)

    f = open("my_gps.txt", "r")
    gps = f.read().split(",")
    f.close()
    lat = float(gps[2].split(":")[1])/10000000
    lon = float(gps[3].split(":")[1])/10000000
    return coordinate(lat,lon)

def readGPSStation():
    try:
        st = open("station.txt", "r")
        lat,lon = st.read().split(",")    
        return coordinate(float(lat), float(lon))
    except:
        return coordinate(13.846324, 100.567100)

def readGPSNode():
    try:
        nd = open("node.txt", "r")
        lat, lon = nd.read().split(",")
        return coordinate(float(lat), float(lon))
    except:
        return coordinate(13.846213, 100.567004)

def sendPacket(ip,message):
    count = 0
    while 1:
        if count == 2: return False
        port = 9999
        buffer_size = 1024
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            s.connect((ip,port))
        except socket.error as msg:
            print "[MESSAGE] "+ip+" Request timeout."
            count += 1
            time.sleep(2)
            continue
        try:
            s.send(message)
            print "[MESSAGE] Success send to: " + ip
            s.close()
            break
        except socket.error as msg:
            time.sleep(2)
            continue

    return True

def createWaypointFile(lat,lon):
    index = 0 

    f = open('wp.txt', 'w')
    f.write("")
    f.close()

    f = open('wp.txt', 'a')
    f.write("QGC WPL 110\n")

    home_lat, home_lon = lat,lon

    try:
        home = open("home_"+my_ip+".txt", "r")
        home_lat, home_lon = home.read().split(",")
        home.close
    except:
        home_lat, home_lon = lat, lon

    f.write(str(index)+"\t1\t0\t16\t0\t0\t0\t0\t"+home_lat+"\t"+home_lon+"\t10.000000\t1\n")
    index+=1
    f.write(str(index)+"\t0\t0\t16\t0\t0\t0\t0\t"+lat+"\t"+lon+"\t10.000000\t1\n")
    index+=1
    f.write(str(index)+"\t0\t0\t17\t0\t0\t0\t0\t"+lat+"\t"+lon+"\t10.000000\t1\n")
    f.close()

def createComebackHomeFile():
    try:
        print "home_"+my_ip+".txt"
        home = open("home_"+my_ip+".txt", "r")
        home_lat, home_lon = home.read().split(",")
        home.close
    except:
        print "NOT SET HOME POINT"
        return 0

    index = 0

    f = open('wp.txt', 'w')
    f.write("")
    f.close()

    f = open('wp.txt', 'a')
    f.write("QGC WPL 110\n")

    f.write(str(index)+"\t1\t0\t16\t0\t0\t0\t0\t"+home_lat+"\t"+home_lon+"\t10.000000\t1\n")
    index+=1
    f.write(str(index)+"\t0\t0\t16\t0\t0\t0\t0\t"+home_lat+"\t"+home_lon+"\t10.000000\t1\n")
    index+=1
    f.write(str(index)+"\t0\t0\t17\t0\t0\t0\t0\t"+home_lat+"\t"+home_lon+"\t10.000000\t1\n")
    f.close()

def sendCommandToMavproxy(command):
    f = open("command.txt", "w")
    f.write("\n".join(command))
    f.close()    

def writeStatus(s):
    f = open("status.txt", "w")
    f.write(s)
    f.close

def waitFirstACK():
    count = 0
    if list_of_active_drone == []:
        return True
    while True:
        for x in list_of_active_drone:
            try:
                f = open('gps_'+str(x)+'.txt', 'r')
            except:
                count = 0
                break
            data = f.read().split(",")
            if int(data[3]) < 2:
                return True
        count += 1
        if count == 3:
            return False
            break
        time.sleep(1)

writeStatus('W')
while 1:

    time.sleep(2)

    status = checkStatus()

    if status[0] == "D":
        designated_drone = status[1]
        checkActive = checkActiveDrone()
        if checkActive == True:
            first_ND = True
            update_times = 0
            state_update = True
            comeback_home = False
            if designated_drone not in list_of_active_drone + [my_ip]:
                if min(list_of_active_drone + [my_ip]) == my_ip:
                    writeStatus("D "+my_ip)
                else:
                    writeStatus("W")
            else:
                writeStatus("D "+designated_drone)

    status = checkStatus()

    if status[0] == "D" and status[1] == my_ip:

        print '[MESSAGE] STATUS : D'
        designated_drone = my_ip
        message = genaratePacket("public", my_ip)
        public_message_station = readGPSStation()
        public_message_node = readGPSNode()
        message_station = genaratePacket("station", str(public_message_station.lat)+","+str(public_message_station.lon))
        message_node = genaratePacket("node", str(public_message_node.lat)+","+str(public_message_node.lon))
        for x in list_of_active_drone:
            sendPacket(x, message+" "+message_station+" "+message_node)

        if first_ND == True:
            if waitFirstACK() == True:
                first_ND = False
            else:
                continue

        if comeback_home == True:
            if time.time() - comeback_time > 3:
                for ip in list_of_active_drone+[my_ip]:
                    if ip == my_ip:
                        createComebackHomeFile()
                        sendCommandToMavproxy(["mode auto", "wp load wp.txt", "wp set 1"])
                        print "[MESSAGE] MOVE LOOPBACK ! (HOME)"
                    else:
                        message = genaratePacket("move", "home")
                        if sendPacket(ip, message) == False:
                            break
                writeStatus("W")
                comeback_home = False

        new_station_point = readGPSStation()
        if getDistance(new_station_point.lat, new_station_point.lon, station_point.lat, station_point.lon) < 1 and state_update :
            print "[MESSAGE] First Same Station Point"
            station_point = new_station_point
            state_update = False
        elif getDistance(new_station_point.lat, new_station_point.lon, station_point.lat, station_point.lon) < 1 :
            print "[MESSAGE] Same Station Point"
            continue
        else:
            print "[MESSAGE] Moving Station Point"
            station_point = new_station_point
            state_update = True
            continue

        node_point = readGPSNode()
        station_node_distance = getDistance(station_point.lat, station_point.lon, node_point.lat, node_point.lon)
        if station_node_distance < 40:
            number_of_drone_for_create_connection = 1
        else:
            number_of_drone_for_create_connection = math.ceil((station_node_distance-40+1)/40)
        if number_of_drone_for_create_connection > number_of_all_drone:
            if comeback_home == False:
                comeback_home = True
                comeback_time = time.time()
            print "[MESSAGE] Impossible Network Form"
            continue
        else:
            comeback_home = False

        list_of_waypoint = calculateWP()
        result_combination = combinationDroneAndWaypoint(list_of_waypoint)
        for ip,lat,lon in result_combination:
            print "[MESSAGE] RESULT : "+ip+" => LAT : "+str(lat)+" LONG : "+str(lon)
        for ip,lat,lon in result_combination:
            if ip == my_ip:
                createWaypointFile(str(lat),str(lon))
                sendCommandToMavproxy(["mode auto", "wp load wp.txt", "wp set 1"])
                print "[MESSAGE] MOVE LOOPBACK ! "+str(lat)+","+str(lon)
            else:
                message = genaratePacket("move", str(lat)+","+str(lon))
                if sendPacket(ip, message) == False:
                    break

    elif status[0] == "D" and status[1] != my_ip:
        print '[MESSAGE] STATUS : ND'
        designated_drone = status[1]
        currentGPS = readGPS()
        message = genaratePacket("forward", str(my_ip)+","+str(currentGPS.lat)+","+str(currentGPS.lon)+","+str(update_times))
        update_times += 1
        if sendPacket(designated_drone, message) == False:
            continue

    elif status[0] == 'W':
        print '[MESSAGE] STATUS : W'

