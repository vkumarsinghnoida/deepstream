#!/usr/bin/python3

def coke():
    amt = 0
    due = 50
    print("Amount due:", due)
    while amt < 50:
        amt += int(input("Insert coin: "))
        if amt <= 50:
            print("Amount due:", 50 - amt)
        else:
            print("Changed owed:", abs(50 - amt))


if __name__ == "__main__":
    coke()
