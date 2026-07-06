from xarm.wrapper import XArmAPI

arm = XArmAPI('192.168.1.221')  # right arm IP

print("connect:", arm.clean_warn())
print("clear_error:", arm.clean_error())
print("motion_enable:", arm.motion_enable(True))
print("set_mode:", arm.set_mode(0))
print("set_state:", arm.set_state(0))
print("state:", arm.get_state())
print("err_code:", arm.get_err_warn_code())