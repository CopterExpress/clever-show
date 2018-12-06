import socket
import play_animation
sock = socket.socket()
serv = '192.168.1.10'
port = 1234
sock.connect((serv, port))
sock.send(bytes('left','utf-8'))
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
                            xm.write(data[:data.index(b'stop')])

                            t_0 = Thread(target=animation)
                            t_0.daemon = True
                            t_0.start()

                            time.sleep(2)
                            break
                        else:
                            xm.write(data)

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



def main():
    pass




if __name__==__main__:
    main()
