import serial.tools.list_ports
import os.path

Import("env")

ports = serial.tools.list_ports.comports()

port_int = {}

current_flash = 0

if os.path.isfile('current_flash.txt'):
    with open('current_flash.txt') as file:
        current_flash = int(file.readline())

for i, port in enumerate(sorted(ports)):
    port_int[i] = port.device
    print(i, port.device)
if current_flash not in port_int:
    current_flash = 0

new_file = ""

with open("src/main.cpp", "rt") as fin:
    for line in fin:
        if line.startswith('unsigned int module_index = '):
            new_file += f'unsigned int module_index = {current_flash + 1};\n'
        else:
            new_file += line
with open("src/main.cpp", "wt") as fout:
    fout.write(new_file)

with open('current_flash.txt', 'w+') as file:
    file.write(str(current_flash + 1))

# for port, desc, hwid in sorted(ports):
#     print("{}: {} [{}]".format(port, desc, hwid))

env['UPLOAD_PORT'] = port_int[current_flash]
