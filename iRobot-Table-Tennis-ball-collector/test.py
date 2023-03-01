import keyboard
import time
num = 0
while True:
    print(num)
    num += 1 
    if keyboard.is_pressed("d"):
        print("i was pressed")
        state = 'InitialSetup'
        time.sleep(3)
        break