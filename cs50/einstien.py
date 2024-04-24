#!/usr/bin/python3

c = 3 * (10**8)


def energy():
    global c
    m = int(input("m = "))
    print("E", m * (c**2))
    return m * c ** 2


if __name__ == '__main__':
    energy()
