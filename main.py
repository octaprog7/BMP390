# This is a sample Python script.
import bmp390
# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.


def print_hi(name):
    # Use a breakpoint in the code line below to debug your script.
    print(f'Hi, {name}')  # Press Ctrl+F8 to toggle the breakpoint.


# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    s = set()
    for index, values in enumerate(bmp390._calibration_regs_addr()):
        print(index, values)
        tmp = str(values[1]+values[2])
        s.add(tmp)
# See PyCharm help at https://www.jetbrains.com/help/pycharm/
    print(s)
