import asyncio
import logging

import lib.protocols as p
import lib.messages as messages

logging.basicConfig(
        level=logging.DEBUG,
        format="%(asctime)s [%(name)-7.7s] [%(threadName)-19.19s] [%(levelname)-7.7s]  %(message)s",
        handlers=[
            logging.StreamHandler()
        ])

async def server():
    loop = asyncio.get_event_loop()
    clients = asyncio.Queue()

    def factory(*args):
        protocol = p.PeerProtocol()
        clients.put_nowait(protocol)
        return protocol

    server = await loop.create_server(factory, host='', port=8181)

    client1 = await clients.get()
    await client1.connected
    msg = await client1.receive_message()
    print(msg.header, msg.content)

    msg = messages.PendingMessage(msg.content + " from server")
    await msg.send_to(client1)

    await client1.closed
    server.close()
    await server.wait_closed()

async def client():
    loop = asyncio.get_event_loop()

    transport, protocol = await loop.create_connection(lambda: p.PeerProtocol(None, None),
                                                       host='', port=8181)

    await protocol.send(messages.PendingMessage("greetings"))
    print((await protocol.receive_message()).content)

    await protocol.close()

async def main():
    s = asyncio.create_task(server())
    c = asyncio.create_task(client())
    await asyncio.gather(s, c)

asyncio.run(main(), debug=True)
