#!/usr/bin/python3

def camel(a):
    eng = ['A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L',
           'M', 'N', 'O', 'P', 'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X', 'Y', 'Z']
    sml = ['a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l',
           'm', 'n', 'o', 'p', 'q', 'r', 's', 't', 'u', 'v', 'w', 'x', 'y', 'z']
    out = ""
    for x in a:
        if x in eng:
            out += '_' + sml[eng.index(x)]
        else:
            out += x

    return out


def main():
    print("snake_case:", camel(input("camelcase: ")))


if __name__ == "__main__":
    main()
