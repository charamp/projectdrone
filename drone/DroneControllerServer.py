import socket
import sys
from thread import *
import time
import fcntl
import struct

def getMyIP(ifname):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    return socket.inet_ntoa(fcntl.ioctl(
        s.fileno(),
        0x8915,
        struct.pack('256s', ifname[:15])
    )[20:24])

my_ip = getMyIP("wlan0")

def createWaypointFile(lat,lon):
    index = 0 

    f = open('wp.txt', 'w')
    f.write("")
    f.close()

    f = open('wp.txt', 'a')
    f.write("QGC WPL 110\n")

    home_lat, home_lon = lat,lon

    try:
        print "home_"+my_ip+".txt"
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

def writeCommandtxt():
    f = open('command.txt', 'w')
    command = [
                "wp load wp.txt",
                "wp set 1"
              ]
    f.write("\n".join(command))
    f.close() 

def processCommand(command):
    if command[0] == "forward":
        ip,lat,lon,update = command[1].split(",")
        writeGPS = open("gps_"+ip+".txt", "w")
        writeGPS.write(command[1])
        writeGPS.close()
    elif command[0] == "move":
        if command[1] == "home":
            createComebackHomeFile()
            writeCommandtxt()
        else:
            lat,lon = command[1].split(",")
            createWaypointFile(lat,lon)
            writeCommandtxt()
    elif command[0] == "public":
        st = open("status.txt", "w")
        st.write("D "+command[1])
        st.close()

        station = open("station.txt", "w")
        station.write(command[3])
        station.close()

        node = open("node.txt", "w")
        node.write(command[5])
        node.close()

    elif command[0] == "root":
        if command[1] == "set" and command[2] == "d":
            st = open("status.txt","w")
            st.write("D "+command[3])
            st.close()
        elif command[1] == "set" and command[2] == "home":
            home = open("home_"+command[3]+".txt", "w")
            home.write(command[4]+","+command[5])
            home.close()
        elif command[1] == "set" and command[2] == "station":
            station = open("station.txt", "w")
            station.write(command[3]+","+command[4])
            station.close()
        elif command[1] == "set" and command[2] == "node":
            node = open("node.txt", "w")
            node.write(command[3]+","+command[4])
            node.close()

def clientthread(conn, addr):
        while True:
            data = conn.recv(1024)
            reply = 'ACK: ' + data
            if not data:
                break
            print data + ' from ' + addr[0]

            processCommand(data.split(" "))

            conn.sendall(reply)
        conn.close()
 
host = ''
port = 9999
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
print 'Socket created'
try:
        s.bind((host, port))
except socket.error as msg:
        print 'Bind failed. Error Code : ' + str(msg[0]) + ' Message ' + msg[1]
        sys.exit()
print 'Socket bind complete'
s.listen(10)
print 'Socket now listening'
while 1:
        conn, addr = s.accept()
        print 'Connected with ' + addr[0] + ':' + str(addr[1])
        start_new_thread(clientthread, (conn, addr))
s.close()