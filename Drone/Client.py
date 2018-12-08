import socket
import play_animation
import time
import ntplib
from threading import Thread
sock = socket.socket()
serv = '192.168.1.10'  # Change to server ip
port = 35001
sock.connect((serv, port))
sock.send(bytes('left', 'utf-8'))
command = ''


def receive():
    global command
    try:
        while True:
            data = str(sock.recv(1024))
            try:
                if b'programm' in data:
                    anim = open('anim.csv', 'w')
                    data = data[data.index(b'programm') + 8:]
                    anim.write(data)

                    while True:
                        data = str(sock.recv(1024))

                        if b'stop' in data:
                            anim.write(data[:data.index(b'stop')])
                            break
                        else:
                            anim.write(data)

                else:
                    try:
                        sq = data.split('$$')
                        for i in range(len(sq) - 1):
                            print(sq[i])
                            command = sq[i]

                    except Exception as e:
                        print(e)
            
            except:
                print('er')
        sock.close()
    except KeyboardInterrupt:
        print("Shutting down")

        led.off()
        sock.close()


def time_synch():
    c = ntplib.NTPClient()
    response = c.request('ntp1.stratum2.ru')
    return response.tx_time-time.time()


def pl_anim():
    global command
    play_animation.read_animation_file()
    dtime = time_synch() - time.time()
    ok = 0
    while True:
        
        if 'begin_anim' in command:
            t_st = int(command[command.index('('):])
            ok=1
        if t_st == dtime+time.time and ok==1:
            break
        if 'synch' in command:
            dtime=time_synch() - time.time()
            print(dtime)

    play_animation.takeoff()
    for frame in play_animation.frame():
        if command != 'pause':
            time.sleep(0.1)
            play_animation.do_next_animation(frame)


if __name__ == "__main__":
    t_0 = Thread(target=receive)
    t_0.daemon = True
    t_0.start()

    t_1 = Thread(target=pl_anim)
    t_1.daemon = True
    t_1.start()
    
    
    
    




