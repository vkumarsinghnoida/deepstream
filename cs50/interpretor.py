#!/usr/bin/python3

def solve(a):
    l = a.split()
    if l[2] != '0':
        return eval(a)
    else:
        pass


def main():
    print(solve(input("Expression: ")))


if __name__ == "__main__":
    main()
