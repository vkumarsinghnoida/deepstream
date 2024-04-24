#!/usr/bin/python3

def remv(a):
    out = ""
    vowels = ['a', 'e', 'i', 'o', 'u', 'A', 'E', 'I', 'O', 'U']
    for x in a:
        if x in vowels:
            pass
        else:
            out += x

    return out


def main():
    print("Output:", remv(input("Input: ")))


if __name__ == "__main__":
    main()
