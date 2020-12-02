import os
import sys
import pytest
current_dir = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, os.path.realpath(os.path.join(current_dir, os.pardir)))
#
# from lib.mes import CallbackManager
#
# def test_callback_registration():
#     def f(arg):
#         return arg
#     c = CallbackManager()
#     c.action_callback("123")(f)
#     t = "test"
#     assert t == c.action_callbacks["123"](t)
