#!/usr/bin/python3

def main():
    dollars = dollars_to_float(input("How much was the meal? "))
    percent = percent_to_float(
        input("What percentage would you like to tip? "))
    tip = dollars * percent
    print(f"Leave ${tip:.2f}")


def dollars_to_float(d):
    d = float(d.strip('$'))
    print(d)
    return d


def percent_to_float(p):
    print(float(p.strip('%')))
    return float(p.strip('%')) * 0.01


if __name__ == '__main__':
    main()
