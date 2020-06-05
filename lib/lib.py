
def b_partial(func, *args, **kwargs):  # call argument blocker partial
    return lambda *a: func(*args, **kwargs)
