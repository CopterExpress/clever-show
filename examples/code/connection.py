import asyncio
import logging

import lib.connections as c
import lib.messages as messages
import lib.utils as utils

logging.basicConfig(
        level=logging.DEBUG,
        format=utils.logger_format,
        handlers=[
            logging.StreamHandler()
        ])

async def server():
    server, new_connections = await c.create_server(8181)
    connection1: c.Connection = await new_connections
    #await connection1.send(messages.Request("greetings"))
    await asyncio.sleep(2)
    t1 = asyncio.create_task(asyncio.wait_for(connection1.send(messages.Request("greetings")), 5))
    t2 = asyncio.create_task(connection1.send(messages.Request("hello", args=(1, 3))))

    print(await asyncio.gather(t1, t2, return_exceptions=True))
    await connection1.close()
    server.close()
    await server.wait_closed()

async def client():
    connection = await c.connect(8181, "")
    request = await connection.receive("hello")
    print(request.message.content)
    request2 = await connection.receive()
    print(request2.message.content)

    await request.reply("hi")

    # await connection.close()

async def main():
    s = asyncio.create_task(server())
    c = asyncio.create_task(client())
    await asyncio.gather(s, c)

asyncio.run(main(), debug=False)
