#!/usr/bin/python3
from indoor import indoor

a = input("Greeting: ").strip()

if 'hello' in a or 'hello' == indoor(a) and (a[0] != 'h' or a[0] != 'H'):
    print("$0")
elif a[0] == 'h' or a[0] == 'H':
    print("$20")
else:
    print("$100")
