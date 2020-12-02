import asyncio

import pytest

from lib.utils import KeyQueue

def test_queue_sync():
    q = KeyQueue()
    val1 = "test1"
    val2 = "test2"
    key1 = "key1"
    q.put_nowait(val1)
    q.put_nowait(val2, key1)

    assert q.get_nowait() == val1
    assert q.get_nowait(key1) == val2

@pytest.mark.asyncio
async def test_queue_putting_async():
    q = KeyQueue()
    val1 = "test1"
    val2 = "test2"
    key1 = "key1"
    await q.put(val1)
    await q.put(val2, key1)

    assert await q.get(key1) == val2
    assert await q.get() == val1
