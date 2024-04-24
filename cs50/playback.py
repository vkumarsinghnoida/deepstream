#!/usr/bin/python3

a = input()
out = ""

for x in a:
    if x == " ":
        x = "..."
    else:
        pass
    out += x

print(out)
