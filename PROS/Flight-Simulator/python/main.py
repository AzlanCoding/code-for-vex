import subprocess, os
import vgamepad as vg

global gamepad
gamepad = vg.VX360Gamepad()
def press(button, value):
    global gamepad
    if value == 1:
        gamepad.press_button(button=button)
    else:
        gamepad.release_button(button=button)

def thresh(value):
    if value > 1:
        return 1
    elif value < -1:
        return -1
    else:
        return value

def execute(command):
    global gamepad
    process = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    while True:
        output = process.stdout.readline().decode()
        if output == '' and process.poll() is not None:
            break
        elif output[:5] == "input":
            #Q=L1
            #W=L2
            #S=R1
            #E=R2
            l = False
            r = False
            data = output[5:-2].split(" ")
            print(data)
            for items in data:
                button = items[0]
                value = int(items[1:])
                if button == "p" or button == "r":
                    pass#LeftJoystick
                elif button == "X":
                    press(vg.XUSB_BUTTON.XUSB_GAMEPAD_X,value)
                elif button == "Y":
                    press(vg.XUSB_BUTTON.XUSB_GAMEPAD_Y,value)
                elif button == "A":
                    press(vg.XUSB_BUTTON.XUSB_GAMEPAD_A,value)
                elif button == "B":
                    press(vg.XUSB_BUTTON.XUSB_GAMEPAD_B,value)
                elif button == "U":
                    press(vg.XUSB_BUTTON.XUSB_GAMEPAD_DPAD_UP,value)
                elif button == "D":
                    press(vg.XUSB_BUTTON.XUSB_GAMEPAD_DPAD_DOWN,value)
                elif button == "L":
                    press(vg.XUSB_BUTTON.XUSB_GAMEPAD_DPAD_LEFT,value)
                elif button == "R":
                    press(vg.XUSB_BUTTON.XUSB_GAMEPAD_DPAD_RIGHT,value)
                elif button == "Q":
                    press(vg.XUSB_BUTTON.XUSB_GAMEPAD_LEFT_SHOULDER,value)
                    if value == 1:
                        l = True
                elif button == "W" and value == 1:
                    gamepad.left_trigger(value=255)
                elif button == "W" and value == 0:
                    gamepad.left_trigger(value=0)
                elif button == "S":
                    press(vg.XUSB_BUTTON.XUSB_GAMEPAD_RIGHT_SHOULDER,value)
                    if value == 1:
                        r = True
                elif button == "E" and value == 1:
                    gamepad.right_trigger(value=255)
                elif button == "E" and value == 0:
                    gamepad.right_trigger(value=0)
            gamepad.update()
            pitch = thresh(int(data[0][1:])/30)
            roll = thresh(int(data[1][1:])/100)
            gamepad.left_joystick_float(x_value_float=roll, y_value_float=pitch)
            if l == True and r == False:
                gamepad.right_joystick_float(x_value_float=-1, y_value_float=0)
            elif l == False and r == True:
                gamepad.right_joystick_float(x_value_float=1, y_value_float=0)
            else:
                gamepad.right_joystick_float(x_value_float=0, y_value_float=0)
            
            
        else:
            #print(output)
            rc = process.poll()
    return rc
execute([os.path.join(os.environ["APPDATA"]+os.sep,"Code\\User\\globalStorage\\sigbots.pros\\install\\pros-cli-windows\\pros.exe"),"terminal"])
